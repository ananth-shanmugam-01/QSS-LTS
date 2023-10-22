function optAx = GGVforwardAcc(carData,control_variables, velocity)

% Control Variables
Vx = velocity;
Ax = control_variables(1); % Wide Open Throttle
throttle_position = 1;

DF_total = 0.5*1.225*carData.Aero.CLA*Vx^2;
DF_front = carData.Aero.rAeroBalance*DF_total;
DF_rear = (1-carData.Aero.rAeroBalance)*DF_front;

Fd = 0.5*1.225*carData.Aero.CDA*Vx^2;

ay = 0; % Straight Line

g = 9.81;

% Motor Output
motor_rot_vel = (velocity/carData.Chassis.radWheel) * carData.Powertrain.rGear *60/(2*pi);
Ft = throttle_position * carData.Powertrain.effPU * carData.Powertrain.nDrive * interp1(carData.Powertrain.RPM,carData.Powertrain.torqueMotor,motor_rot_vel,'spline') * carData.Powertrain.rGear/carData.Chassis.radWheel;

ax_tractive = (Ft - Fd) / carData.Chassis.mass;

% Lateral Load Transfer
del_w_f = (carData.Chassis.SprungMass * ay * carData.Suspension.heightCG2rollAxis * carData.Suspension.mechanicalBalance/carData.Chassis.trackWidth)...
    + (carData.Chassis.sprungMassFront* ay * carData.Suspension.rollCentreFront / carData.Chassis.trackWidth) + (carData.Chassis.unsprungMass * carData.Chassis.heightUnsprungCOG * ay / carData.Chassis.trackWidth);
del_w_r = (carData.Chassis.SprungMass * ay * carData.Suspension.heightCG2rollAxis * (1-carData.Suspension.mechanicalBalance)/carData.Chassis.trackWidth)...
    + (carData.Chassis.sprungMassRear * ay * carData.Suspension.rollCentreRear / carData.Chassis.trackWidth) + (carData.Chassis.unsprungMass * carData.Chassis.heightUnsprungCOG * ay / carData.Chassis.trackWidth);

% Longitudinal Load Transfer
longLT = carData.Chassis.mass * ax_tractive * carData.Chassis.heightSprungCOG / (2 * carData.Chassis.wheelBase);

% Wheel Loads 
w_fl = (carData.Chassis.massFront * g / 2) + (del_w_f) - longLT + (DF_front/2);
w_fr = (carData.Chassis.massFront * g / 2) - (del_w_f) - longLT + (DF_front/2);
w_rl = (carData.Chassis.massRear * g / 2) + (del_w_r) + longLT + (DF_rear/2);
w_rr = (carData.Chassis.massRear * g / 2) - (del_w_r) + longLT + (DF_rear/2);

% Wheel Forces
% [MUY,MUX] = FrictionEllipseModel(FZ,inputMUY,inputMUX)
[fyFL,fxFL] = FrictionEllipseModel(carData,w_fl,0,0);
[fyFR,fxFR] = FrictionEllipseModel(carData,w_fr,0,0);
[fyRL,fxRL] = FrictionEllipseModel(carData,w_rl,0,0);
[fyFR,fxRR] = FrictionEllipseModel(carData,w_rr,0,0);

% Vehicle Acceleration
ax_out = min((Ft - Fd),(fxRL + fxRR))/carData.Chassis.mass; % Power Limited or Grip Limited

optAx = -Ax;