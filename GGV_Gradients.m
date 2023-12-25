clear; clc
addpath(genpath("C:\Users\admin\Desktop\Git Repository\QSS-LTS-F3"))

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
carData.Chassis.weightDist = 0.5;
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
carData.Aero.CLA = 3.8;
carData.Aero.CDA = 1.2;
carData.Aero.rAeroBalance = 0.5;

% Suspension Properties
carData.Suspension.heightCG2rollAxis = 0.230;
carData.Suspension.rollCentreFront = 0.052;
carData.Suspension.rollCentreRear = 0.051;
carData.Suspension.mechanicalBalance = 0.5;

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
carData.Powertrain.vMaxRPM = (carData.Powertrain.RPM(end)/carData.Powertrain.rGear)*0.10472*carData.Chassis.radWheel; % Calculate RPM based top speed limit
carData.Powertrain.vMaxDrag = calcDragLimitVel(carData); % Calculate Drag Based top speed limit
carData.Powertrain.vMax = min(carData.Powertrain.vMaxRPM,carData.Powertrain.vMaxDrag); % Select minimum of two as speed limit 

%% Create Objective and Constraint Functions

tic

[accelObjectiveFunction, accelConstraintFunction] = forwardAccelerationCalc(carData);
[brakingObjectiveFunction, brakingConstraintFunction] = BrakingDecelerationCalc(carData);
[combinedAccelerationObjectiveFunction,combinedAccelerationConstraintFunction] = combinedAccelerationCalc(carData);
[combinedDecelerationObjectiveFunction, combinedDecelerationConstraintFunction] = combinedDecelerationCalc(carData);

toc
disp('Objective and Constraint Function Calculated')

%% Calculate GGV

velocityRange = linspace(10,carData.Powertrain.vMax-1, 10);

options = optimoptions(@fmincon,...
    'Algorithm','interior-point',...
    'SpecifyObjectiveGradient',true,...
    'SpecifyConstraintGradient',true,...
    "SubproblemAlgorithm","cg",...
    'OptimalityTolerance', 0.025,...
    'ConstraintTolerance',0.4,...
    'MaxIterations', 700,...
    'MaxFunctionEvaluations', 3000,...
    'HessianApproximation', 'lbfgs',...
    'ScaleProblem', true,...
    'Display','iter'); % % 'OptimalityTolerance', 0.1,

% Forward Acceleration
tic
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

    x0 = [iterVel, 0.1, iterInitWheelVel, iterInitWheelVel, iterInitWheelVel, iterInitWheelVel];
    
    Aeq = [1, 0, 0, 0, 0, 0];
    beq = [iterVel];
    lb = [0, 0, 0, 0, 0, 0];
    ub = [inf, 1,vMaxWheel, vMaxWheel, vMaxWheel, vMaxWheel];

    [xfinal,fval,exitflag,output] = fmincon(accelObjectiveFunction,x0,...
    [],[],Aeq,beq,lb,ub,accelConstraintFunction,options); % ConstraintFunction
    axAcc(i) = -fval;
    
end
toc
disp('Forward Acceleration Envelope Complete')

% Braking Deceleration
tic
axDecc = zeros(length(velocityRange),1);
for i = 1:numel(velocityRange)

    % brakePressure = x(1).*100;
    % wheel_rot_fl = x(2);
    % wheel_rot_fr = x(3);
    % wheel_rot_rl = x(4);
    % wheel_rot_rr = x(5);
    % Vx = x(6);
    
    iterVel = velocityRange(i);
    iterInitWheelVel = iterVel/carData.Chassis.radWheel;

    vMaxWheel = carData.Powertrain.vMax/carData.Chassis.radWheel;

    x0 = [50, iterInitWheelVel, iterInitWheelVel, iterInitWheelVel, iterInitWheelVel, iterVel];
    
    Aeq = [0, 0, 0, 0, 0, 1];
    beq = [iterVel];
    lb = [0,iterInitWheelVel/2, iterInitWheelVel/2, iterInitWheelVel/2, iterInitWheelVel/2, 0];
    ub = [100,vMaxWheel, vMaxWheel, vMaxWheel, vMaxWheel, inf];

    [xfinal,fval,exitflag,output] = fmincon(brakingObjectiveFunction,x0,...
    [],[],Aeq,beq,lb,ub,brakingConstraintFunction,options); % ConstraintFunction
    axDecc(i) = fval;
    
end
toc
disp('Braking Deceleration Envelope Complete')
%% Combined Forward Acceleration

AxPoints = 5;

clear GGV
GGV = zeros([10,20, 3]);

GGVAccVel = [];
GGVAccAy = [];
GGVAccAx = [];

GGVBrVel = [];
GGVBrAy = [];
GGVBrAx = [];

tic
for i = 1:numel(velocityRange)

    iterVel = velocityRange(i); % Iteration Velocity
    
    % Forward Acceleration ------------------------
    
    AxTol = 0.5; % Starting Point
    AxRange = linspace(AxTol,axAcc(i),AxPoints)'; % 5 Calculated Points between zero and maximum forward acceleration at that point
    FrlatAcc = zeros(length(AxRange),1);
    
    for j = 1:numel(AxRange)
     
        % Input Values
        %     del = x(1);
        %     beta = x(2);
        %     wheel_rot_fl = x(3);
        %     wheel_rot_fr = x(4);
        %     wheel_rot_rl = x(5);
        %     wheel_rot_rr = x(6);
        %     Vx = x(7);
        %     Ax = x(8);
        %     Ay = x(9);     
        
        iterAx = AxRange(j);
        iterInitWheelVel = iterVel/carData.Chassis.radWheel;
    
        vMaxWheel = carData.Powertrain.vMax/carData.Chassis.radWheel;
    
        x0 = [12, -1, iterInitWheelVel, iterInitWheelVel, iterInitWheelVel, iterInitWheelVel, iterVel, iterAx, 5];
        Aeq = [0, 0, 0, 0, 0, 0, 1, 0, 0;
               0, 0, 0, 0, 0, 0, 0, 1, 0];
        beq = [iterVel, iterAx];
        lb = [0, -2, 0, 0, 0, 0, 0, 0, 0];
        ub = [20, 0, vMaxWheel, vMaxWheel, vMaxWheel, vMaxWheel, inf, inf, inf];
        [xfinal,fval,exitflag,output] = fmincon(combinedAccelerationObjectiveFunction,x0,...
        [],[],Aeq,beq,lb,ub,combinedAccelerationConstraintFunction,options); % ConstraintFunction 
        FrlatAcc(j) = -fval;
    
    end
    
    GGVforwardVel = [iterVel.*ones(2*length(AxRange),1)];
    GGVforwardAx = [AxRange; flipud(AxRange)];
    GGVforwardAy = [FrlatAcc; -flipud(FrlatAcc)];

    GGVAccVel = [GGVAccVel;GGVforwardVel;iterVel];
    GGVAccAy = [GGVAccAy;GGVforwardAy;0];
    GGVAccAx = [GGVAccAx;GGVforwardAx;AxRange(end)];

    % Braking Deceleration ------------------------
    
    AxTol = 0; % Starting Point
    AxRange = linspace(AxTol,axDecc(i),AxPoints)'; % 5 Calculated Points between zero and maximum forward acceleration at that point
    BrlatAcc = zeros(length(AxRange),1);
    
    for j = 1:numel(AxRange)
     
        % Input Values
        %     del = x(1);
        %     beta = x(2);
        %     wheel_rot_fl = x(3);
        %     wheel_rot_fr = x(4);
        %     wheel_rot_rl = x(5);
        %     wheel_rot_rr = x(6);
        %     Vx = x(7);
        %     Ax = x(8);
        %     Ay = x(9);     
        
        iterAx = AxRange(j);
        iterInitWheelVel = iterVel/carData.Chassis.radWheel;
    
        vMaxWheel = carData.Powertrain.vMax/carData.Chassis.radWheel;
    
        x0 = [12, -1, iterInitWheelVel, iterInitWheelVel, iterInitWheelVel, iterInitWheelVel, iterVel, iterAx, 0.1];
        Aeq = [0, 0, 0, 0, 0, 0, 1, 0, 0;
               0, 0, 0, 0, 0, 0, 0, 1, 0];
        beq = [iterVel, iterAx];
        lb = [0, -2, iterInitWheelVel/2, iterInitWheelVel/2, iterInitWheelVel/2, iterInitWheelVel/2, 0, -inf, 0];
        ub = [20, 0, vMaxWheel, vMaxWheel, vMaxWheel, vMaxWheel, inf, 0, inf];
        [xfinal,fval,exitflag,output] = fmincon(combinedDecelerationObjectiveFunction,x0,...
        [],[],Aeq,beq,lb,ub,combinedDecelerationConstraintFunction,options); % ConstraintFunction 
        BrlatAcc(j) = -fval;
    
    end
    
    GGVBrakeVel = [iterVel.*ones(2*length(AxRange),1)];
    GGVBrakeAx = [(AxRange); flipud(AxRange)];
    GGVBrakeAy = [(BrlatAcc); -flipud(BrlatAcc)];

    GGV(i,:,1) = [GGVforwardVel;GGVBrakeVel] ; % Vel
    GGV(i,:,2) = [GGVforwardAx;GGVBrakeAx] ; % Ax
    GGV(i,:,3) = [GGVforwardAy;GGVBrakeAy] ; % Ay

    GGVBrVel = [GGVBrVel;GGVBrakeVel;iterVel];
    GGVBrAy = [GGVBrAy;GGVBrakeAy;0];
    GGVBrAx = [GGVBrAx;GGVBrakeAx;AxRange(end)];


end
toc

GGVAcceleration = [GGVAccVel, GGVAccAy, GGVAccAx];
GGVDeceleration = [GGVBrVel, GGVBrAy, GGVBrAx];

disp('Combined Acceleration Envelope Complete')

%%
save('GGV.mat',"GGV")
save('GGVAcceleration.mat',"GGVAcceleration")
save('GGVDeceleration.mat',"GGVDeceleration")


figure
surf(GGV(:,:,3),GGV(:,:,2),GGV(:,:,1))
xlabel('Ay')
ylabel('Ax')
zlabel('V')






