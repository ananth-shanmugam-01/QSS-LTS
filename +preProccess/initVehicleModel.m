%% Initialise Car Model

function carData = initVehicleModel()

    carData = struct;
    g = 9.81;
    
    % Chassis Properties
    carData.Chassis.mass = 304;
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

    % Tyre Properties
    carData.Tyre.Name = 'Hoosier R20 16x6-10';
    carData.Tyre.Source = 'Fitting';
    carData.Tyre.FZ0 = 1100;
    carData.Tyre.LFZO = 1.2;
    % Scaling Factors
    
    carData.Tyre.LGAY    =   1;
    carData.Tyre.LHY     =   1;
    carData.Tyre.LVY     =   1;
    carData.Tyre.LCY     =   1;
    carData.Tyre.LEY     =   1;
    carData.Tyre.LHX     =   0.4;
    carData.Tyre.LVX     =   1;
    carData.Tyre.LGX     =   1;
    carData.Tyre.LCX     =   1;
    carData.Tyre.LEX     =   1;
    carData.Tyre.LXAL    =   1;
    
    carData.Tyre.LKY     =   1; % 1
    carData.Tyre.LKX     =   1; %  0.7
    carData.Tyre.LMUY    =   0.6; % 0.38 Changed to Fit
    carData.Tyre.LMUX    =   0.6; % 0.25 Changed to Fit
    
    % Longitudinal Coefficients
    carData.Tyre.PCX1   =   1.2602;
    carData.Tyre.PDX1   =   2.354;
    carData.Tyre.PDX2   =   -0.015401;
    carData.Tyre.PDX3   =   -0.76992;
    carData.Tyre.PEX1   =  -1.0845;
    carData.Tyre.PEX2   =   2.3203;
    carData.Tyre.PEX3   =   3.2136;
    carData.Tyre.PEX4   =   -1.7027;
    carData.Tyre.PKX1   =   39.334;
    carData.Tyre.PKX2   =   -0.37146;
    carData.Tyre.PKX3   =   0.37752;
    carData.Tyre.PHX1   =   0.025058;
    carData.Tyre.PHX2   =   -0.038843;
    carData.Tyre.PVX1   =   -0.00045953;
    carData.Tyre.PVX2   =   0.0013401;
    
    % Combined Longitudinal Coefficients
    carData.Tyre.RBX1   =   7.4574;
    carData.Tyre.RBX2   =   -8.8044;
    carData.Tyre.RCX1   =   1.5974;
    carData.Tyre.REX1   =   0.22918;
    carData.Tyre.REX2   =   -0.5217;
    carData.Tyre.RHX1   =   0;
    
    % Lateral Coefficients
    carData.Tyre.PCY1   =   1.4;
    carData.Tyre.PDY1   =   2.4;
    carData.Tyre.PDY2   =   -0.4507889;
    carData.Tyre.PDY3   =   20;
    carData.Tyre.PEY1   =   0.01;
    carData.Tyre.PEY2   =   0.05;
    carData.Tyre.PEY3   =   10;
    carData.Tyre.PEY4   =   0;
    carData.Tyre.PKY1   =   -27.3678;
    carData.Tyre.PKY2   =   1.242483;
    carData.Tyre.PKY3   =   3;
    carData.Tyre.PHY1   =   -0.00002845241;
    carData.Tyre.PHY2   =   -0.0000329537;
    carData.Tyre.PHY3   =   0.1416031;
    carData.Tyre.PVY1   =   0;
    carData.Tyre.PVY2   =   -0.009009;
    carData.Tyre.PVY3   =   -0.5;
    carData.Tyre.PVY4   =   -1;
    
    % Combined Lateral Coefficients
    carData.Tyre.RBY1   =   26.3099;
    carData.Tyre.RBY2   =   20.3304;
    carData.Tyre.RBY3   =   -0.015204;
    carData.Tyre.RCY1   =   0.96889;
    carData.Tyre.REY1   =   0.53522;
    carData.Tyre.REY2   =   0.69602;
    carData.Tyre.RHY1   =   0;
    carData.Tyre.RHY2   =   0;
    carData.Tyre.RVY1   =   0;
    carData.Tyre.RVY2   =   0;
    carData.Tyre.RVY3   =   0;
    carData.Tyre.RVY4   =   0;
    carData.Tyre.RVY5   =   0;
    carData.Tyre.RVY6   =   0;
    
    % Aerodynamic Properties
    carData.Aero.CLA = 3.8;
    carData.Aero.CDA = 1.2;
    carData.Aero.rAeroBalance = 0.5;
    
    % Suspension Properties
    carData.Suspension.heightCG2rollAxis = 0.230;
    carData.Suspension.rollCentreFront = 0.052;
    carData.Suspension.rollCentreRear = 0.051;
    
    carData.Suspension.mechanicalBalance = 0.5;
    % Suspension Setup

    carData.Suspension.aCamberFront = 0; % [deg], negative is leaning inwards
    carData.Suspension.aCamberRear = 0; % [deg], negative is leaning inwards
    carData.Suspension.aToeStaticFront = 0; % [deg], negative is toe inwards, wheel pointing inwards to chassis
    carData.Suspension.aToeStaticRear = 0; % [deg], negative is toe inwards, wheel pointing inwards to chassis
    
    % Brake Properties
    carData.Brakes.muBrakePad = 0.4;
    carData.Brakes.nPistonFront = 4;
    carData.Brakes.nPistonRear = 2;
    carData.Brakes.diamPiston = 0.0254;
    carData.Brakes.areaPiston = (0.25*pi*carData.Brakes.diamPiston^2);
    carData.Brakes.radBrakeDisc = 0.1836/2;
    carData.Brakes.rBrakeBias = 0.55;
    
    % Powertrain Model
    [carData.Powertrain.RPM, carData.Powertrain.torqueMotor] = powerCurveInterpolation('AMK_21Nm.txt');
    carData.Powertrain.rGear = 15.55;
    carData.Powertrain.effPU = 0.6;
    carData.Powertrain.nDrive = 2; % number of drive wheels, switch case later?
    carData.Powertrain.vMaxRPM = (carData.Powertrain.RPM(end)/carData.Powertrain.rGear)*0.10472*carData.Chassis.radWheel; % Calculate RPM based top speed limit
    carData.Powertrain.vMaxDrag = calcDragLimitVel(carData); % Calculate Drag Based top speed limit
    carData.Powertrain.vMax = min(carData.Powertrain.vMaxRPM,carData.Powertrain.vMaxDrag); % Select minimum of two as speed limit 

    function [rpmInterp, torqInterp] = powerCurveInterpolation(filename)

        % Interpolation for Powertrain Model
        % Smooth Powertrain Curves would be nice but not here :( 
        % 1 RPM = 0.10472 rad/s
        
        motor_data = load(filename);
        AMK_21Nm_Trq = motor_data(:,2); % + (linspace(0, 1, length(motor_data(:,2)))*1E-10)'; % Creating fake unique values
        AMK_Motor_RPM = motor_data(:,1)';
        AMK_Motor_Torque = AMK_21Nm_Trq';
        rpmInterp = linspace(AMK_Motor_RPM(1),AMK_Motor_RPM(end),1000)';
        torqInterp = interp1(AMK_Motor_RPM,AMK_Motor_Torque,rpmInterp);

    end

    % Crude Drag Limited Velocity Check
    function dragLimitVel = calcDragLimitVel(carData)

        wheel_avg_vel = 0:1:carData.Powertrain.vMaxRPM;
        motor_rot_vel = wheel_avg_vel .* carData.Powertrain.rGear *60/(2*pi) / carData.Chassis.radWheel;
        Ft = carData.Powertrain.effPU * carData.Powertrain.nDrive .* interp1(carData.Powertrain.RPM,carData.Powertrain.torqueMotor,motor_rot_vel,'spline') * carData.Powertrain.rGear/carData.Chassis.radWheel;
        Fd = 0.5*1.225*carData.Aero.CDA.*wheel_avg_vel.^2;
        Ax = sign((Ft - Fd)./carData.Chassis.mass);
        Ax = diff(Ax);
            if isempty(find(Ax == -2))
                wheel_avg_vel = 0:1:carData.Powertrain.vMaxRPM*2;
                motor_rot_vel = wheel_avg_vel .* carData.Powertrain.rGear *60/(2*pi) / carData.Chassis.radWheel;
                Ft = carData.Powertrain.effPU * carData.Powertrain.nDrive .* interp1(carData.Powertrain.RPM,carData.Powertrain.torqueMotor,motor_rot_vel,'spline') * carData.Powertrain.rGear/carData.Chassis.radWheel;
                Fd = 0.5*1.225*carData.Aero.CDA.*wheel_avg_vel.^2;
                Ax = sign((Ft - Fd)./carData.Chassis.mass);
                Ax = diff(Ax);       
                dragLimitVel = wheel_avg_vel(find(Ax == -2)); 
            else 
                dragLimitVel = wheel_avg_vel(find(Ax == -2));
            end

    end

end