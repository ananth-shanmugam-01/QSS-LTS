function outputs = carModel(inputs, carData)

    %%%%%%%
    % 15/04/24 - attempt at single car model code
    % Aim is for all vehicle physics to be covered in a single function
    % inputs should be all control inputs and states
    % outputs are all variables of interest 
    % algebraic loops are also addressed at this level (_res)
    %%%%%%%
    import casadi.*
    
    % Control Inputs and States
    Vx              = inputs.Vx;
    delta           = inputs.delta;
    beta            = inputs.beta;
    yaw_rate        = inputs.yaw_rate;
    throttle_position = inputs.throttle_position;
    brake_pressure  = inputs.brake_pressure;
    wheel_rot_fl    = inputs.wheel_rot_fl;
    wheel_rot_fr    = inputs.wheel_rot_fr;
    wheel_rot_rl    = inputs.wheel_rot_rl;
    wheel_rot_rr    = inputs.wheel_rot_rr;
    
    
    g = 9.81;
    DF_total = 0.5*1.225*carData.Aero.CLA*Vx^2;
    DF_front = carData.Aero.rAeroBalance*DF_total;
    DF_rear = (1-carData.Aero.rAeroBalance)*DF_front;
    Fd = 0.5*1.225*carData.Aero.CDA*Vx^2;
    
    ay_control = Vx*yaw_rate;
    
    % Motor Tractive Force
    torqueInterp = interpolant('LUT','bspline',{[carData.Powertrain.RPM]},carData.Powertrain.torqueMotor); % CasADi feature
    wheel_avg_vel = 0.5*(wheel_rot_rl + wheel_rot_rr);
    motor_rot_vel = wheel_avg_vel * carData.Powertrain.rGear *60/(2*pi);
    F_tractive = throttle_position * carData.Powertrain.effPU * carData.Powertrain.nDrive * torqueInterp(motor_rot_vel) * carData.Powertrain.rGear/carData.Chassis.radWheel;
    
    % Braking Decelerative Force
    Front_Brake_Force = 2*(brake_pressure * 100 * carData.Brakes.rBrakeBias .*10^5 .*carData.Brakes.areaPiston .*carData.Brakes.nPistonFront) * carData.Brakes.muBrakePad * carData.Brakes.radBrakeDisc / carData.Chassis.radWheel; %N
    Rear_Brake_Force = 2*(brake_pressure * 100 *(1-carData.Brakes.rBrakeBias) .*10^5 .*carData.Brakes.areaPiston .*carData.Brakes.nPistonFront) * carData.Brakes.muBrakePad * carData.Brakes.radBrakeDisc / carData.Chassis.radWheel; %N
    F_brake = -(Front_Brake_Force + Rear_Brake_Force);
    
    ax_control = (F_tractive + F_brake - Fd) / carData.Chassis.mass;
    
    % Slip Angles
    alpha_fl = ((Vx*tan(beta) + carData.Chassis.frontMomentArm*yaw_rate) / (Vx + yaw_rate*carData.Chassis.trackWidth*0.5)) - (delta - carData.Suspension.aToeStaticFront*pi/180); % [deg], negative is toe inwards, wheel pointing inwards to chassis
    alpha_fr = ((Vx*tan(beta) + carData.Chassis.frontMomentArm*yaw_rate) / (Vx - yaw_rate*carData.Chassis.trackWidth*0.5)) - (delta + carData.Suspension.aToeStaticFront*pi/180);
    alpha_rl = ((Vx*tan(beta) - carData.Chassis.rearMomentArm*yaw_rate) / (Vx + yaw_rate*carData.Chassis.trackWidth*0.5))  - (-carData.Suspension.aToeStaticRear*pi/180); % [deg], negative is toe inwards, wheel pointing inwards to chassis
    alpha_rr = ((Vx*tan(beta) - carData.Chassis.rearMomentArm*yaw_rate) / (Vx - yaw_rate*carData.Chassis.trackWidth*0.5))  - (carData.Suspension.aToeStaticRear*pi/180);
    
    % Slip Ratios
    kappa_fl = (wheel_rot_fl*carData.Chassis.radWheel - Vx)/Vx;
    kappa_fr = (wheel_rot_fr*carData.Chassis.radWheel - Vx)/Vx;
    kappa_rl = (wheel_rot_rl*carData.Chassis.radWheel - Vx)/Vx;
    kappa_rr = (wheel_rot_rr*carData.Chassis.radWheel - Vx)/Vx;
    
    % Lateral Load Transfer
    del_w_f = (carData.Chassis.SprungMass * ay_control * carData.Suspension.heightCG2rollAxis * carData.Suspension.mechanicalBalance/carData.Chassis.trackWidth)...
        + (carData.Chassis.sprungMassFront *ay_control * carData.Suspension.rollCentreFront / carData.Chassis.trackWidth) + (carData.Chassis.unsprungMass * carData.Chassis.heightUnsprungCOG * ay_control / carData.Chassis.trackWidth);
    del_w_r = (carData.Chassis.SprungMass * ay_control * carData.Suspension.heightCG2rollAxis * (1-carData.Suspension.mechanicalBalance)/carData.Chassis.trackWidth)...
        + (carData.Chassis.sprungMassRear * ay_control * carData.Suspension.rollCentreRear / carData.Chassis.trackWidth) + (carData.Chassis.unsprungMass * carData.Chassis.heightUnsprungCOG * ay_control / carData.Chassis.trackWidth);
    
    % Longitudinal Load Transfer
    longLT = carData.Chassis.mass * ax_control * carData.Chassis.heightSprungCOG / (2 * carData.Chassis.wheelBase);
    
    % Wheel Loads 
    w_fl = (carData.Chassis.massFront * g / 2) + (del_w_f) - longLT + (DF_front/2);
    w_fr = (carData.Chassis.massFront * g / 2) - (del_w_f) - longLT + (DF_front/2);
    w_rl = (carData.Chassis.massRear * g / 2) + (del_w_r) + longLT + (DF_rear/2);
    w_rr = (carData.Chassis.massRear * g / 2) - (del_w_r) + longLT + (DF_rear/2);
    
    % Wheel Forces
    [fy_fl, fx_fl] = tyreModel.MF52(kappa_fl,alpha_fl,w_fl,carData.Suspension.aCamberFront, carData);
    [fy_fr, fx_fr] = tyreModel.MF52(kappa_fr,alpha_fr,w_fr,-carData.Suspension.aCamberFront, carData);
    [fy_rl, fx_rl] = tyreModel.MF52(kappa_rl,alpha_rl,w_rl,carData.Suspension.aCamberRear, carData);
    [fy_rr, fx_rr] = tyreModel.MF52(kappa_rr,alpha_rr,w_rr,-carData.Suspension.aCamberRear, carData);
    
    % Car States
    ay_out = (fy_fl + fy_fr + fy_rl + fy_rr)/carData.Chassis.mass;
    ax_out = (fx_fl + fx_fr + fx_rl + fx_rr)/carData.Chassis.mass;
    Mz_out = (carData.Chassis.frontMomentArm*(fy_fl + fy_fr) + 0.5*carData.Chassis.trackWidth*(fx_fl + fx_rl) ...
              - carData.Chassis.rearMomentArm*(fy_rl + fy_rr) - 0.5*carData.Chassis.trackWidth*(fx_fr + fx_rr))/carData.Chassis.yawInertia;

    fxDistribution = (fx_fl + fx_fr) / (fx_fl + fx_fr + fx_rl + fx_rr); % Select externally on whether this applies to brake bias or tractive force bias
    
    
   % Connect variables to outputs
    outputs = struct;
    
    % Residuals
    outputs.ax_res = ax_control - ax_out;
    outputs.ay_res = ay_control - ay_out;

    % Auxiliary Outputs
    outputs.Mz_out      = Mz_out;
    
    outputs.ay_out      = ay_out;
    outputs.ay_control  = ay_control;

    outputs.ax_out      = ax_out;
    outputs.ax_control  = ax_control;

    outputs.fxDistribution = fxDistribution;

    outputs.alpha_fl    = alpha_fl;
    outputs.alpha_fr    = alpha_fr;
    outputs.alpha_rl    = alpha_rl;
    outputs.alpha_rr    = alpha_rr;

    outputs.kappa_fl    = kappa_fl;
    outputs.kappa_fr    = kappa_fr;
    outputs.kappa_rl    = kappa_rl;
    outputs.kappa_rr    = kappa_rr;

end