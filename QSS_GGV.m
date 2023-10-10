clear; clc; close all;

%% GGV Calculations

% Vehicle Model
carData = struct;
constants = struct; 
g = 9.81;

% Chassis Properties
carData.Chassis.mass = 274;
carData.Chassis.unsprungMass = 14.7;
carData.Chassis.SprungMass = carData.Chassis.mass - 4*(carData.Chassis.unsprungMass);
carData.Chassis.heightUnsprungCOG = 0.196;
carData.Chassis.heightSprungCOG = 0.280;
carData.Chassis.weightDist = 0.45;
carData.Chassis.radWheel = 0.196; % Loaded radius
carData.Chassis.wheelBase = 1.535;
carData.Chassis.trackWidth = 1.23;
carData.Chassis.massFront = carData.Chassis.mass*carData.Chassis.weightDist;
carData.Chassis.massRear = carData.Chassis.mass*(1-carData.Chassis.weightDist);
carData.Chassis.sprungMassFront = carData.Chassis.SprungMass*carData.Chassis.weightDist;
carData.Chassis.sprungMassRear = carData.Chassis.SprungMass*(1-carData.Chassis.weightDist);
carData.Chassis.frontMomentArm = carData.Chassis.wheelBase*(1 - carData.Chassis.weightDist);
carData.Chassis.rearMomentArm = carData.Chassis.wheelBase*(carData.Chassis.weightDist);
carData.Chassis.yawInertia = 120; % kgm^2

% Aerodynamic Properties
carData.Aero.CLA = -0.06;
carData.Aero.CDA = 0.6;
carData.Aero.rAeroBalance = 0.55;

% Suspension Properties
carData.Suspension.heightCG2rollAxis = 0.230;
carData.Suspension.rollCentreFront = 0.052;
carData.Suspension.rollCentreRear = 0.051;
carData.Suspension.mechanicalBalance = 0.4;

% Brake Properties
carData.Brakes.muBrakePad = 0.4;
carData.Brakes.nPistonFront = 4;
carData.Brakes.nPistonRear = 2;
carData.Brakes.diamPiston = 0.0254;
carData.Brakes.areaPiston = (0.25*pi*carData.Brakes.diamPiston^2);
carData.Brakes.radBrakeDisc = 0.1836/2;
carData.Brakes.rBrakeBias = 0.5;

% Powertrain Model
[carData.Powertrain.RPM, carData.Powertrain.torqueMotor] = powerCurveInterpolation('Data Files\AMK_21Nm.txt');
carData.Powertrain.rGear = 15.55;
carData.Powertrain.effPU = 0.8;
carData.Powertrain.nDrive = 2; % number of drive wheels, switch case later?

% Vmax Calculations, FS style
carData.Powertrain.vMax = (carData.Powertrain.RPM(end)/carData.Powertrain.rGear)*0.10472*carData.Chassis.radWheel; % Converted to m/s 

%% Velocity Range

velocityRange = linspace(2,carData.Powertrain.vMax, 10);

%% Forward Acceleration
axAcc = zeros(length(velocityRange),1);
for i = 1:numel(velocityRange)

    % Vx = velocity;
    % throttle_position = control_variables(1);
    % wheel_rot_fl = control_variables(2);
    % wheel_rot_fr = control_variables(3);
    % wheel_rot_rl = control_variables(4);
    % wheel_rot_rr = control_variables(5);
    
    iterVel = velocityRange(i);
    iterInitWheelVel = iterVel/carData.Chassis.radWheel;

    vMaxWheel = carData.Powertrain.vMax/carData.Chassis.radWheel;

    x0 = [0.8, iterInitWheelVel, iterInitWheelVel, iterInitWheelVel, iterInitWheelVel];
    
    Aeq = [];
    beq = [];
    lb = [0, iterInitWheelVel, iterInitWheelVel, iterInitWheelVel, iterInitWheelVel];
    ub = [1, vMaxWheel, vMaxWheel, vMaxWheel, vMaxWheel];
    [control_inputs, fnval, exitflag] = fmincon(@(x) GGVforwardAcc(carData, x, iterVel),x0,[],[],Aeq,beq,lb,ub,@(x) nonLinearConstraintsGGVforwardAcc(carData,x,iterVel)); 
    axAcc(i) = -fnval;
    
end

%% Deceleration

brAcc = zeros(length(velocityRange),1);
for i = 1:numel(velocityRange)

    % Control Variables
    % Vx = velocity;
    % brakePressure = control_variables(1);
    % wheel_rot_fl = control_variables(2);
    % wheel_rot_fr = control_variables(3);
    % wheel_rot_rl = control_variables(4);
    % wheel_rot_rr = control_variables(5);

    iterVel = velocityRange(i);
    iterInitWheelVel = iterVel/carData.Chassis.radWheel;

    vMaxWheel = carData.Powertrain.vMax/carData.Chassis.radWheel;

    x0 = [0.3, iterInitWheelVel, iterInitWheelVel, iterInitWheelVel, iterInitWheelVel];

    Aeq = [];
    beq = [];
    lb = [0, 0, 0, 0, 0];
    ub = [1, vMaxWheel, vMaxWheel, vMaxWheel, vMaxWheel];
    [control_inputs, fnval, exitflag] = fmincon(@(x) GGVDeceleration(carData, x, iterVel),x0,[],[],Aeq,beq,lb,ub,@(x) nonLinearConstraintsGGVDeceleration(carData,x,iterVel)); 
    brAcc(i) = fnval;

end

%% Cornering Envelope

AxTol = 0.5; % To retain the characteristic that max Ax has 0 Ay
AxRange = linspace(0,axAcc(5)+AxTol,50);

latAcc = zeros(length(AxRange),1);

for i = 1:numel(AxRange)

    % Input Values
    % Vx = velocity;
    % ax_tractive = Ax;
    % 
    % del = control_variables(1);
    % beta = control_variables(2);
    % yawRate = control_variables(3);
    % wheel_rot_fl = control_variables(4);
    % wheel_rot_fr = control_variables(5);
    % wheel_rot_rl = control_variables(6);
    % wheel_rot_rr = control_variables(7);

    iterVel = velocityRange(5);
    iterAx = AxRange(i);
    iterInitWheelVel = iterVel/carData.Chassis.radWheel;

    vMaxWheel = carData.Powertrain.vMax/carData.Chassis.radWheel;

    x0 = [12, -2, 1, iterInitWheelVel, iterInitWheelVel, iterInitWheelVel, iterInitWheelVel];
    Aeq = [];
    beq = [];
    lb = [0, 0, 0, 0, 0, 0, 0];
    ub = [27, 0, 2, vMaxWheel, vMaxWheel, vMaxWheel, vMaxWheel];
    [control_inputs, fnval, exitflag] = fmincon(@(x) GGVCombinedPositiveAx(carData, x, iterVel, iterAx),x0,[],[],Aeq,beq,lb,ub,@(x) nonLinearConstraintsGGVCombinedPositiveAx(carData, x, iterVel, iterAx)); 
    % GGVCombinedPositiveAx(carData, control_variables, velocity, Ax)
    latAcc(i) = fnval;

end















