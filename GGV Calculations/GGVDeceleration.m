function optAx = GGVDeceleration(carData,control_variables, velocity)

% Control Variables
Vx = velocity;
brakePressure = control_variables(1)*100;

DF_total = 0.5*1.225*carData.Aero.CLA*Vx^2;
DF_front = carData.Aero.rAeroBalance*DF_total;
DF_rear = (1-carData.Aero.rAeroBalance)*DF_front;
Fd = 0.5*1.225*carData.Aero.CDA*Vx^2;

ay = 0;

g = 9.81;

% Braking Torque
% Brake Model with brake pressure as input and brake bias
Front_Brake_Force = 2*(brakePressure * carData.Brakes.rBrakeBias .*10^5 .*carData.Brakes.areaPiston .*carData.Brakes.nPistonFront) * carData.Brakes.muBrakePad * carData.Brakes.radBrakeDisc / carData.Chassis.radWheel; %N
Rear_Brake_Force = 2*(brakePressure*(1-carData.Brakes.rBrakeBias) .*10^5 .*carData.Brakes.areaPiston .*carData.Brakes.nPistonFront) * carData.Brakes.muBrakePad * carData.Brakes.radBrakeDisc / carData.Chassis.radWheel; %N

FBrakeFL = Front_Brake_Force/2;
FBrakeFR = Front_Brake_Force/2;
FBrakeRL = Rear_Brake_Force/2;
FBrakeRR = Rear_Brake_Force/2;

ax_brake = -(Front_Brake_Force + Rear_Brake_Force + Fd) / carData.Chassis.mass;

% Lateral Load Transfer
del_w_f = (carData.Chassis.SprungMass * ay * carData.Suspension.heightCG2rollAxis * carData.Suspension.mechanicalBalance/carData.Chassis.trackWidth)...
    + (carData.Chassis.sprungMassFront* ay * carData.Suspension.rollCentreFront / carData.Chassis.trackWidth) + (carData.Chassis.unsprungMass * carData.Chassis.heightUnsprungCOG * ay / carData.Chassis.trackWidth);
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
% [MUY,MUX] = FrictionEllipseModel(FZ,inputMUY,inputMUX)
[fyFL,fxFL] = FrictionEllipseModel(carData,w_fl,0,0);
[fyFR,fxFR] = FrictionEllipseModel(carData,w_fr,0,0);
[fyRL,fxRL] = FrictionEllipseModel(carData,w_rl,0,0);
[fyFR,fxRR] = FrictionEllipseModel(carData,w_rr,0,0);

% Minimum Force
fxFL = min(FBrakeFL,fxFL);
fxFR = min(FBrakeFR,fxFR);
fxRL = min(FBrakeRL,fxRL);
fxRR = min(FBrakeRR,fxRR);

% Vehicle Acceleration
ax_out = -(fxFL + fxFR + fxRL + fxRR + Fd)/carData.Chassis.mass;

optAx = ax_out;