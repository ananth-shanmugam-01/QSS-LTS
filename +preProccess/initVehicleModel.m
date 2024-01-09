%% Initialise Car Model

function carData = initVehicleModel()

    carData = struct;
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
    [carData.Powertrain.RPM, carData.Powertrain.torqueMotor] = powerCurveInterpolation('AMK_21Nm.txt');
    carData.Powertrain.rGear = 15.55;
    carData.Powertrain.effPU = 0.4;
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