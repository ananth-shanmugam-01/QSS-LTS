clear; clc;

% Symbolic Input Variables
load('carData.mat')

x = sym('x',[1 4],'real');
% Initialise Objective Function Output

% Inputs - 
% Vx = control_variables(1);
% del = control_variables(2);
% beta = control_variables(3);
% kt = curvature;

yaw_rate = x(1)*x(4);
ay = x(4)*x(1)^2;

g = 9.81;

DF_total = 0.5*1.225*carData.Aero.CLA*x(1)^2;
DF_front = carData.Aero.rAeroBalance*DF_total;
DF_rear = (1-carData.Aero.rAeroBalance)*DF_front;

% Slip Angles
alpha_fl = ((x(1)*tan(deg2rad(x(3))) + carData.Chassis.frontMomentArm*yaw_rate) / (x(1) + yaw_rate*carData.Chassis.trackWidth*0.5)) - deg2rad(x(2));
alpha_fr = ((x(1)*tan(deg2rad(x(3))) + carData.Chassis.frontMomentArm*yaw_rate) / (x(1) - yaw_rate*carData.Chassis.trackWidth*0.5)) - deg2rad(x(2));
alpha_rl = (x(1)*tan(deg2rad(x(3))) - carData.Chassis.rearMomentArm*yaw_rate) / (x(1) + yaw_rate*carData.Chassis.trackWidth*0.5);
alpha_rr = (x(1)*tan(deg2rad(x(3))) - carData.Chassis.rearMomentArm*yaw_rate) / (x(1) - yaw_rate*carData.Chassis.trackWidth*0.5);

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

Vx_out = ay_out./yaw_rate;

optOut = sym(-Vx_out); % Maximise velocity through corner

c = sym(zeros(1,21));

c(1) = Vx_out - carData.Powertrain.vMax; % Below Maximum Forward Velocity

c(2) = alpha_fl - deg2rad(10); % Measured Slip Angle Limits
c(3) = -alpha_fl - deg2rad(10);
c(4) = alpha_fl + deg2rad(10);
c(5) = -alpha_fl + deg2rad(10);

c(6) = alpha_fl - deg2rad(10); % Measured Slip Angle Limits
c(7) = -alpha_fl - deg2rad(10);
c(8) = alpha_fl + deg2rad(10);
c(9) = -alpha_fl + deg2rad(10);

c(10) = alpha_fl - deg2rad(10); % Measured Slip Angle Limits
c(11) = -alpha_fl - deg2rad(10);
c(12) = alpha_fl + deg2rad(10);
c(13) = -alpha_fl + deg2rad(10);

c(14) = alpha_fl - deg2rad(10); % Measured Slip Angle Limits
c(15) = -alpha_fl - deg2rad(10);
c(16) = alpha_fl + deg2rad(10);
c(17) = -alpha_fl + deg2rad(10);

c(18) = -w_fl; % Positive Wheel Loads
c(19) = -w_fr;
c(20) = -w_rl;
c(21) = -w_rr;

gradc = jacobian(c,x).'; % .' performs transpose
hessc = cell(1, length(c));

currdir = [pwd filesep]; % You might need to use currdir = pwd 

% Calculate Hessians and assign to function file for Hessian of inequality constraints
for i = 1:length(c)
    hessc{i} = jacobian(gradc(:,i),x);
    ii = num2str(i);
    thename = ['hessc',ii];
    filename = [currdir,thename,'.m'];
    matlabFunction(hessc{i},'file',filename,'vars',{x});
end

ceq = sym(zeros(1,7));
ceq(1) = Vx_out - x(1);
ceq(2) = (carData.Chassis.frontMomentArm*(fy_fl + fy_fr) - carData.Chassis.rearMomentArm*(fy_rl + fy_rr))/carData.Chassis.yawInertia; % Yaw Acceleration = 0, for quasi-steady state
ceq(3) = sign(x(4)) + sign(alpha_fl); % Control sign of slip angle
ceq(4) = sign(x(4)) + sign(alpha_fr);
ceq(5) = sign(x(4)) + sign(alpha_rl);
ceq(6) = sign(x(4)) + sign(alpha_rr);
ceq(7) = ay - ay_out; % Converge between input and out Ay

gradceq = jacobian(ceq,x).'; % .' performs transpose
hessceq = cell(1, length(ceq));
for i = 1:length(ceq)
    hessceq{i} = jacobian(gradceq(:,i),x);
    ii = num2str(i);
    thename = ['hessceq',ii];
    filename = [currdir,thename,'.m'];
    matlabFunction(hessceq{i},'file',filename,'vars',{x});
end

% Create Constraint Function File
filename = [currdir,'constraintFunction.m'];
matlabFunction(c,ceq,gradc,gradceq,'file',filename,'vars',{x},...
    'outputs',{'c','ceq','gradc','gradceq'});



