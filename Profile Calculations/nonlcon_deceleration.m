function [c,ceq] = nonlcon_deceleration(control_variables, curvature, prev_velocity, max_cornering_vel, carData)

global sector_dist

del = control_variables(1);
beta = control_variables(2);
brakePressure = control_variables(3).*100;
wheel_rot_fl = control_variables(4);
wheel_rot_fr = control_variables(5);
wheel_rot_rl = control_variables(6);
wheel_rot_rr = control_variables(7);
Vx = control_variables(8); % Constraints for Vx defined in nonlcon

DF_total = 0.5*1.225*carData.Aero.CLA*Vx^2;
DF_front = carData.Aero.rAeroBalance*DF_total;
DF_rear = (1-carData.Aero.rAeroBalance)*DF_front;
Fd = 0.5*1.225*carData.Aero.CDA*Vx^2;

kt = curvature;
yaw_rate = Vx*kt;
ay = kt*Vx^2;

g = 9.81;

% Braking Torque
% Brake Model with brake pressure as input and brake bias
Front_Brake_Force = 2*(brakePressure * carData.Brakes.rBrakeBias .*10^5 .*carData.Brakes.areaPiston .*carData.Brakes.nPistonFront) * carData.Brakes.muBrakePad * carData.Brakes.radBrakeDisc / carData.Chassis.radWheel; %N
Rear_Brake_Force = 2*(brakePressure*(1-carData.Brakes.rBrakeBias) .*10^5 .*carData.Brakes.areaPiston .*carData.Brakes.nPistonFront) * carData.Brakes.muBrakePad * carData.Brakes.radBrakeDisc / carData.Chassis.radWheel; %N
ax_brake = -(Front_Brake_Force + Rear_Brake_Force + Fd) / carData.Chassis.mass;

% Slip Angles
alpha_fl = ((Vx*tan(deg2rad(beta)) + carData.Chassis.frontMomentArm*yaw_rate) / (Vx + yaw_rate*carData.Chassis.trackWidth*0.5)) - deg2rad(del);
alpha_fr = ((Vx*tan(deg2rad(beta)) + carData.Chassis.frontMomentArm*yaw_rate) / (Vx - yaw_rate*carData.Chassis.trackWidth*0.5)) - deg2rad(del);
alpha_rl = (Vx*tan(deg2rad(beta)) - carData.Chassis.rearMomentArm*yaw_rate) / (Vx + yaw_rate*carData.Chassis.trackWidth*0.5);
alpha_rr = (Vx*tan(deg2rad(beta)) - carData.Chassis.rearMomentArm*yaw_rate) / (Vx - yaw_rate*carData.Chassis.trackWidth*0.5);

% Slip Ratios
% Simplification for Pure Lateral Slip at Apex
kappa_fl = (wheel_rot_fl*carData.Chassis.radWheel - Vx)/Vx;
kappa_fr = (wheel_rot_fr*carData.Chassis.radWheel - Vx)/Vx;
kappa_rl = (wheel_rot_rl*carData.Chassis.radWheel - Vx)/Vx;
kappa_rr = (wheel_rot_rr*carData.Chassis.radWheel - Vx)/Vx;

% Lateral Load Transfer
del_w_f = (carData.Chassis.SprungMass * ay * carData.Suspension.heightCG2rollAxis * carData.Suspension.mechanicalBalance/carData.Chassis.trackWidth)...
    + (carData.Chassis.sprungMassFront *ay * carData.Suspension.rollCentreFront / carData.Chassis.trackWidth) + (carData.Chassis.unsprungMass * carData.Chassis.heightUnsprungCOG * ay / carData.Chassis.trackWidth);
del_w_r = (carData.Chassis.SprungMass * ay * carData.Suspension.heightCG2rollAxis * (1-carData.Suspension.mechanicalBalance)/carData.Chassis.trackWidth)...
    + (carData.Chassis.sprungMassRear * ay * carData.Suspension.rollCentreRear / carData.Chassis.trackWidth) + (carData.Chassis.unsprungMass * carData.Chassis.heightUnsprungCOG * ay / carData.Chassis.trackWidth);

% Longitudinal Load Transfer
longLT = carData.Chassis.mass * ax_brake * carData.Chassis.heightSprungCOG / (2 * carData.Chassis.wheelBase);

% Wheel Loads 
w_fl = (carData.Chassis.massFront * g / 2) + (del_w_f) - longLT + (DF_front/2);
w_fr = (carData.Chassis.massFront * g / 2) - (del_w_f) - longLT + (DF_front/2);
w_rl = (carData.Chassis.massRear * g / 2) + (del_w_r) + longLT + (DF_rear/2);
w_rr = (carData.Chassis.massRear * g / 2) - (del_w_r) + longLT + (DF_rear/2);

% Wheel Forces
[fy_fl, fx_fl] = MF52_Combined(kappa_fl,alpha_fl,w_fl,0);
[fy_fr, fx_fr] = MF52_Combined(kappa_fr,alpha_fr,w_fr,0);
[fy_rl, fx_rl] = MF52_Combined(kappa_rl,alpha_rl,w_rl,0);
[fy_rr, fx_rr] = MF52_Combined(kappa_rr,alpha_rr,w_rr,0);

% Vehicle Acceleration
ay_out = (fy_fl + fy_fr + fy_rl + fy_rr) / carData.Chassis.mass;
ax_out = (fx_fl + fx_fr + fx_rl + fx_rr - Fd) / carData.Chassis.mass;

next_vel = abs(sqrt(Vx^2 + 2*(ax_out)*sector_dist));

% Inequality Constraints
c(1) = Vx - max_cornering_vel; % equal or below Maximum Cornering Velocity for the existing point
c(2) = abs(alpha_fl) - abs(deg2rad(10)); % Measured Slip Angle Limits
c(3) = abs(alpha_fr) - abs(deg2rad(10));
c(4) = abs(alpha_rl) - abs(deg2rad(10));
c(5) = abs(alpha_rr) - abs(deg2rad(10));
c(6) = -w_fl; % Positive Wheel Loads
c(7) = -w_fr;
c(8) = -w_rl;
c(9) = -w_rr;
c(10) = abs(kappa_fl) - 0.1; % Measured Slip Ratio Limits
c(11) = abs(kappa_fr) - 0.1;
c(12) = abs(kappa_rl) - 0.1;
c(13) = abs(kappa_rr) - 0.1;
c(14) = -brakePressure;

ceq(1) = (carData.Chassis.frontMomentArm*(fy_fl + fy_fr) - carData.Chassis.rearMomentArm*(fy_rl + fy_rr))/carData.Chassis.yawInertia; % Yaw Acceleration = 0, for quasi-steady state
ceq(2) = (0.5*carData.Chassis.trackWidth*(fx_fl + fx_rl) - 0.5*carData.Chassis.trackWidth*(fx_fr + fx_rr))/carData.Chassis.yawInertia; % Not used to solution instability
ceq(2) = sign(kt) + sign(alpha_fl); % Control sign of slip angle
ceq(3) = sign(kt) + sign(alpha_fr);
ceq(4) = sign(kt) + sign(alpha_rl);
ceq(5) = sign(kt) + sign(alpha_rr);
ceq(6) = ay - ay_out; % Converge between input and out Ay
ceq(7) = ax_brake - ax_out;
ceq(8) = next_vel - prev_velocity;

end