function [c,ceq] = nonlcon_cornering(control_variables, curvature,carData)

% Input Values
Vx = control_variables(1);
del = control_variables(2);
beta = control_variables(3);

kt = curvature;

yaw_rate = Vx*kt;
ay = kt*Vx^2;

g = 9.81;

DF_total = 0.5*1.225*carData.Aero.CLA*Vx^2;
DF_front = carData.Aero.rAeroBalance*DF_total;
DF_rear = (1-carData.Aero.rAeroBalance)*DF_front;

% Slip Angles
alpha_fl = ((Vx*tan(deg2rad(beta)) + carData.Chassis.frontMomentArm*yaw_rate) / (Vx + yaw_rate*carData.Chassis.trackWidth*0.5)) - deg2rad(del);
alpha_fr = ((Vx*tan(deg2rad(beta)) + carData.Chassis.frontMomentArm*yaw_rate) / (Vx - yaw_rate*carData.Chassis.trackWidth*0.5)) - deg2rad(del);
alpha_rl = (Vx*tan(deg2rad(beta)) - carData.Chassis.rearMomentArm*yaw_rate) / (Vx + yaw_rate*carData.Chassis.trackWidth*0.5);
alpha_rr = (Vx*tan(deg2rad(beta)) - carData.Chassis.rearMomentArm*yaw_rate) / (Vx - yaw_rate*carData.Chassis.trackWidth*0.5);

% Slip Ratios
% Simplification for Pure Lateral Slip at Apex
kappa_fl = 0;
kappa_fr = 0;
kappa_rl = 0;
kappa_rr = 0;

% Lateral Load Transfer
del_w_f = (carData.Chassis.SprungMass * ay * carData.Suspension.heightCG2rollAxis * carData.Suspension.mechanicalBalance/carData.Chassis.trackWidth)...
    + (carData.Chassis.sprungMassFront *ay * carData.Suspension.rollCentreFront / carData.Chassis.trackWidth) + (carData.Chassis.unsprungMass * carData.Chassis.heightUnsprungCOG * ay / carData.Chassis.trackWidth);
del_w_r = (carData.Chassis.SprungMass * ay * carData.Suspension.heightCG2rollAxis * (1-carData.Suspension.mechanicalBalance)/carData.Chassis.trackWidth)...
    + (carData.Chassis.sprungMassRear * ay * carData.Suspension.rollCentreRear / carData.Chassis.trackWidth) + (carData.Chassis.unsprungMass * carData.Chassis.heightUnsprungCOG * ay / carData.Chassis.trackWidth);

% Wheel Loads
w_fl = (carData.Chassis.massFront * g / 2) + (del_w_f) + (DF_front/2); % - (mass*ax*h_cg/(2*wb));
w_fr = (carData.Chassis.massFront * g / 2) - (del_w_f) + (DF_front/2); % - (mass*ax*h_cg/(2*wb));
w_rl = (carData.Chassis.massRear * g / 2) + (del_w_r) + (DF_rear/2); % + (mass*ax*h_cg/(2*wb));
w_rr = (carData.Chassis.massRear * g / 2) - (del_w_r) + (DF_rear/2); % + (mass*ax*h_cg/(2*wb));

% Wheel Forces
[fy_fl, ~] = MF52_Combined(kappa_fl,alpha_fl,w_fl,0);
[fy_fr, ~] = MF52_Combined(kappa_fr,alpha_fr,w_fr,0);
[fy_rl, ~] = MF52_Combined(kappa_rl,alpha_rl,w_rl,0);
[fy_rr, ~] = MF52_Combined(kappa_rr,alpha_rr,w_rr,0);

% Vehicle Acceleration
ay_out = (fy_fl + fy_fr + fy_rl + fy_rr)/carData.Chassis.mass;

c(1) = Vx - carData.Powertrain.vMax; % Below Maximum Forward Velocity
c(2) = abs(alpha_fl) - abs(deg2rad(10)); % Measured Slip Angle Limits
c(3) = abs(alpha_fr) - abs(deg2rad(10));
c(4) = abs(alpha_rl) - abs(deg2rad(10));
c(5) = abs(alpha_rr) - abs(deg2rad(10));
c(6) = -w_fl; % Positive Wheel Loads
c(7) = -w_fr;
c(8) = -w_rl;
c(9) = -w_rr;

ceq(1) = (carData.Chassis.frontMomentArm*(fy_fl + fy_fr) - carData.Chassis.rearMomentArm*(fy_rl + fy_rr))/carData.Chassis.yawInertia; % Yaw Acceleration = 0, for quasi-steady state
ceq(2) = sign(kt) + sign(alpha_fl); % Control sign of slip angle
ceq(3) = sign(kt) + sign(alpha_fr);
ceq(4) = sign(kt) + sign(alpha_rl);
ceq(5) = sign(kt) + sign(alpha_rr);
ceq(6) = ay - ay_out; % Converge between input and out Ay
% ceq(7) = (kt*Vx^2) - ay; % Relationship between Vx and Ay
% ceq(8) = Vx*kt - yaw_rate; % Relationship between Vx and Yaw Rate