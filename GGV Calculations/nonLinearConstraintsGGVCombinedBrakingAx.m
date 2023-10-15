function [c,ceq] = nonLinearConstraintsGGVCombinedBrakingAx(carData, control_variables, velocity, Ax)

% Input Values
Vx = velocity;
ax_brake = Ax;

del = control_variables(1);
beta = control_variables(2);
yawRate = control_variables(3);
kappa_fl = control_variables(4);
kappa_fr = control_variables(5);
kappa_rl = control_variables(6);
kappa_rr = control_variables(7);

DF_total = 0.5*1.225*carData.Aero.CLA*Vx^2;
DF_front = carData.Aero.rAeroBalance*DF_total;
DF_rear = (1-carData.Aero.rAeroBalance)*DF_front;

ay = Vx*yawRate;

g = 9.81;

% Slip Angles
alpha_fl = ((Vx*tan(deg2rad(beta)) + carData.Chassis.frontMomentArm*yawRate) / (Vx + yawRate*carData.Chassis.trackWidth*0.5)) - deg2rad(del);
alpha_fr = ((Vx*tan(deg2rad(beta)) + carData.Chassis.frontMomentArm*yawRate) / (Vx - yawRate*carData.Chassis.trackWidth*0.5)) - deg2rad(del);
alpha_rl = (Vx*tan(deg2rad(beta)) - carData.Chassis.rearMomentArm*yawRate) / (Vx + yawRate*carData.Chassis.trackWidth*0.5);
alpha_rr = (Vx*tan(deg2rad(beta)) - carData.Chassis.rearMomentArm*yawRate) / (Vx - yawRate*carData.Chassis.trackWidth*0.5);

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

% SR 0 Offsets
[~, fx_fl0] = MF52_Combined(0,0,w_fl,0);
[~, fx_fr0] = MF52_Combined(0,0,w_fr,0);
[~, fx_rl0] = MF52_Combined(0,0,w_rl,0);
[~, fx_rr0] = MF52_Combined(0,0,w_rr,0);

[fy_fl, fx_fl] = MF52_Combined(kappa_fl,alpha_fl,w_fl,0);
fx_fl = fx_fl - fx_fl0;

[fy_fr, fx_fr] = MF52_Combined(kappa_fr,alpha_fr,w_fr,0);
fx_fr = fx_fr - fx_fr0;

[fy_rl, fx_rl] = MF52_Combined(kappa_rl,alpha_rl,w_rl,0);
fx_rl = fx_rl - fx_rl0;

[fy_rr, fx_rr] = MF52_Combined(kappa_rr,alpha_rr,w_rr,0);
fx_rr = fx_rr - fx_rr0;

% Vehicle Acceleration
ay_out = (fy_fl + fy_fr + fy_rl + fy_rr)/carData.Chassis.mass;
ax_out = (fx_fl + fx_fr + fx_rl + fx_rr)/carData.Chassis.mass;

% Inequality Constraints
c(1) = abs(alpha_fl) - abs(deg2rad(8)); % Measured Slip Angle Limits
c(2) = abs(alpha_fr) - abs(deg2rad(8));
c(3) = abs(alpha_rl) - abs(deg2rad(8));
c(4) = abs(alpha_rr) - abs(deg2rad(8));
c(5) = -w_fl; % Positive Wheel Loads
c(6) = -w_fr;
c(7) = -w_rl;
c(8) = -w_rr;

ceq(1) = (carData.Chassis.frontMomentArm*(fy_fl + fy_fr) - carData.Chassis.rearMomentArm*(fy_rl + fy_rr))/carData.Chassis.yawInertia;
% ceq(2) = (0.5*carData.Chassis.trackWidth*(fx_rl) - 0.5*carData.Chassis.trackWidth*(fx_rr))/carData.Chassis.yawInertia;
ceq(3) = ay - ay_out; 
ceq(4) = ax_brake - ax_out;
ceq(5) = sign(fx_rl) - sign(fx_rr);
ceq(6) = sign(fy_fl) - sign(fy_rl);

end