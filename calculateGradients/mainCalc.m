%% Calculating Gradients and Hessians

clear;
clc;
close all;
addpath(genpath("C:\Users\admin\Desktop\Git Repository\QSS-LTS-F3"))
%% TrackData
tic;
global sector_dist
sector_dist = 1;
% [ay_meas, dist, vel_ms, time, track_dist, curvature_spline, curvature] = logged_data('track_data.txt',sector_dist);

[~, ~, ~, ~, trackDist, trackCurvature, ~] = logged_data('track_data.txt',sector_dist);

% trackDist = (1:1:110)';
% trackCurvature = [(1/9.125).*ones(numel(trackDist)/2,1); (-1/9.125).*ones(numel(trackDist)/2,1)];

%% Converting Vehicle Properties to Struct 
% Avoiding the use of global parameters

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

%% Calculate Gradients and Hessians for Pure Cornering Case

options = optimoptions(@fmincon,'Algorithm','sqp',...
    'SpecifyObjectiveGradient',true, 'SpecifyConstraintGradient',true,'HessianFcn',@hessfinal,...
    'FunctionTolerance',0.1,'ConstraintTolerance',0.0005,'MaxIterations',5000,...
    'Display','final'); % ,'SpecifyObjectiveGradient',true, 'SpecifyConstraintGradient',true,'HessianFcn',@hessfinal,'HessianFcn',@hessfinal,
clear LSP
LSP = zeros(numel(trackCurvature),1);

LSP(1) = 29;
for i = 2:numel(trackCurvature)
    %i = 164;
        test_curvature = trackCurvature(i);
    
    if test_curvature > 0 % Right Hand Corner
        min_steering = 0;
        max_steering = 27;
        beta_min = -1;
        beta_max = 0;
        curvature_min = test_curvature;
        curvature_max = inf;
        start_steering = 5;
    else
        min_steering = -27;
        max_steering = 0;
        beta_min = 0;
        beta_max = 1;
        curvature_min = -test_curvature;
        curvature_max = -inf;
        start_steering = -5;
    end 
    
    x0 = [LSP(i-1), start_steering, 0, test_curvature];
    
    Aeq = [0, 0, 0, 1];
    beq = [test_curvature];
    lb = [0, min_steering, beta_min,test_curvature];
    ub = [carData.Powertrain.vMax, max_steering, beta_max, test_curvature];

    [xfinal,fval,exitflag,output] = fmincon(@objectiveFunction,x0,...
    [],[],Aeq,beq,lb,ub,@constraintFunction,options);

    [c, ceq] = nonlinconstraint(xfinal,carData);

    LSP(i) = -fval;

    disp([num2str(i) '/' num2str(numel(trackCurvature))])

end

figure
plot(trackDist,LSP)























