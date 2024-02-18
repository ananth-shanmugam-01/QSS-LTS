%%%%%%%%%%%%%%%%%%%%%%%
% CasADi Problem Formulation of Vehicle Model
% Reference - Mario Boxheimer 

function FSP = forwardSpeed(i, sector_length, LSP, FSP, curvature, V_current, ay_control, carData)

    %% initialize Problem
    import casadi.*
    
    nlpFSP = casadi.Opti();
    
    deltaMin = -27;
    deltaMax = 27;
    betaMin = -3;
    betaMax = 3;
    
    % decision variables & box constraints
    delta = nlpFSP.variable(); nlpFSP.subject_to(deltaMin*pi/180<=delta<=deltaMax*pi/180);                                           % steering angle (rad)
    beta = nlpFSP.variable(); nlpFSP.subject_to(betaMin*pi/180<=beta<=betaMax*pi/180);                                               % sideslip angle (rad)
    throttle_position = nlpFSP.variable(); nlpFSP.subject_to(0<=throttle_position<=1);                                               % throttle position (-)
    wheel_rot_fl = nlpFSP.variable(); nlpFSP.subject_to(0<=wheel_rot_fl<=carData.Powertrain.vMax/carData.Chassis.radWheel)           % FL wheel angular velocity (rad/s)
    wheel_rot_fr = nlpFSP.variable(); nlpFSP.subject_to(0<=wheel_rot_fr<=carData.Powertrain.vMax/carData.Chassis.radWheel)           % FR wheel angular velocity (rad/s)
    wheel_rot_rl = nlpFSP.variable(); nlpFSP.subject_to(0<=wheel_rot_rl<=carData.Powertrain.vMax/carData.Chassis.radWheel)           % RL wheel angular velocity (rad/s)
    wheel_rot_rr = nlpFSP.variable(); nlpFSP.subject_to(0<=wheel_rot_rr<=carData.Powertrain.vMax/carData.Chassis.radWheel)           % RR wheel angular velocity (rad/s)
    
    %% Equations of Motion
    
    yaw_rate = V_current*curvature;
    
    g = 9.81;
    DF_total = 0.5*1.225*carData.Aero.CLA*V_current^2;
    DF_front = carData.Aero.rAeroBalance*DF_total;
    DF_rear = (1-carData.Aero.rAeroBalance)*DF_front;
    Fd = 0.5*1.225*carData.Aero.CDA*V_current^2;

    % Motor Tractive Force
    torqueInterp = interpolant('LUT','bspline',{[carData.Powertrain.RPM]},carData.Powertrain.torqueMotor); % CasADi feature
    wheel_avg_vel = 0.5*(wheel_rot_rl + wheel_rot_rr);
    motor_rot_vel = wheel_avg_vel * carData.Powertrain.rGear *60/(2*pi);
    F_tractive = throttle_position * carData.Powertrain.effPU * carData.Powertrain.nDrive * torqueInterp(motor_rot_vel) * carData.Powertrain.rGear/carData.Chassis.radWheel;
    
    % % Braking Decelerative Force
    % Front_Brake_Force = 2*(brake_pressure * 100 * carData.Brakes.rBrakeBias .*10^5 .*carData.Brakes.areaPiston .*carData.Brakes.nPistonFront) * carData.Brakes.muBrakePad * carData.Brakes.radBrakeDisc / carData.Chassis.radWheel; %N
    % Rear_Brake_Force = 2*(brake_pressure * 100 *(1-carData.Brakes.rBrakeBias) .*10^5 .*carData.Brakes.areaPiston .*carData.Brakes.nPistonFront) * carData.Brakes.muBrakePad * carData.Brakes.radBrakeDisc / carData.Chassis.radWheel; %N
    % F_brake = -(Front_Brake_Force + Rear_Brake_Force);
    
    ax_control = (F_tractive - Fd) / carData.Chassis.mass;
    
    % Slip Angles
    alpha_fl = ((V_current*tan(beta) + carData.Chassis.frontMomentArm*yaw_rate) / (V_current + yaw_rate*carData.Chassis.trackWidth*0.5)) - (delta - carData.Suspension.aToeStaticFront*pi/180); % [deg], negative is toe inwards, wheel pointing inwards to chassis
    alpha_fr = ((V_current*tan(beta) + carData.Chassis.frontMomentArm*yaw_rate) / (V_current - yaw_rate*carData.Chassis.trackWidth*0.5)) - (delta + carData.Suspension.aToeStaticFront*pi/180);
    alpha_rl = (V_current*tan(beta) - carData.Chassis.rearMomentArm*yaw_rate) / (V_current + yaw_rate*carData.Chassis.trackWidth*0.5) - (-carData.Suspension.aToeStaticRear*pi/180); % [deg], negative is toe inwards, wheel pointing inwards to chassis
    alpha_rr = (V_current*tan(beta) - carData.Chassis.rearMomentArm*yaw_rate) / (V_current - yaw_rate*carData.Chassis.trackWidth*0.5) - (carData.Suspension.aToeStaticRear*pi/180);
    
    % Slip Ratios
    kappa_fl = (wheel_rot_fl*carData.Chassis.radWheel - V_current)/V_current;
    kappa_fr = (wheel_rot_fr*carData.Chassis.radWheel - V_current)/V_current;
    kappa_rl = (wheel_rot_rl*carData.Chassis.radWheel - V_current)/V_current;
    kappa_rr = (wheel_rot_rr*carData.Chassis.radWheel - V_current)/V_current;
    
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

    torqueDistribution_tyre = (fx_fl + fx_fr) / (fx_fl + fx_fr + fx_rl + fx_rr);
    
    % Car States
    ay_out = (fy_fl + fy_fr + fy_rl + fy_rr)/carData.Chassis.mass;
    ax_out = (fx_fl + fx_fr + fx_rl + fx_rr)/carData.Chassis.mass;
    Mz_out = (carData.Chassis.frontMomentArm*(fy_fl + fy_fr) + 0.5*carData.Chassis.trackWidth*(fx_fl + fx_rl) ...
      - carData.Chassis.rearMomentArm*(fy_rl + fy_rr) - 0.5*carData.Chassis.trackWidth*(fx_fr + fx_rr))/carData.Chassis.yawInertia;
    
    % Target Speed
    V_out = sqrt(V_current^2 + 2*ax_out*sector_length);
    
    % Residuals
    ax_res = ax_control - ax_out;
    ay_res = ay_control - ay_out;
    % torqueDist_res = carData.Powertrain.torqueDistribution - torqueDistribution_tyre;

    % Path Constraints
    nlpFSP.subject_to(-12*pi/180<=alpha_fl<=12*pi/180);
    nlpFSP.subject_to(-12*pi/180<=alpha_fr<=12*pi/180);
    nlpFSP.subject_to(-12*pi/180<=alpha_rl<=12*pi/180);
    nlpFSP.subject_to(-12*pi/180<=alpha_rr<=12*pi/180);
    
    nlpFSP.subject_to(0<=kappa_fl<=0.1);
    nlpFSP.subject_to(0<=kappa_fr<=0.1);
    nlpFSP.subject_to(0<=kappa_rl<=0.1);
    nlpFSP.subject_to(0<=kappa_rr<=0.1);
    % objective
    nlpFSP.minimize(-V_out);
    
    % initialization of decision variables
    nlpFSP.set_initial(delta,0);
    nlpFSP.set_initial(beta,0);
    nlpFSP.set_initial(throttle_position,1);
    nlpFSP.set_initial(wheel_rot_fl,LSP.wheel_rot_fl(i));
    nlpFSP.set_initial(wheel_rot_fr,LSP.wheel_rot_fr(i));
    nlpFSP.set_initial(wheel_rot_rl,LSP.wheel_rot_rl(i));
    nlpFSP.set_initial(wheel_rot_rr,LSP.wheel_rot_rr(i));
    
    % Constraints
    nlpFSP.subject_to(-0.05<=ay_res<=0.05);
    nlpFSP.subject_to(-0.05<=ax_res<=0.05);
    nlpFSP.subject_to(-5 <= Mz_out <= 5);
    nlpFSP.subject_to(kappa_fl == 0);
    nlpFSP.subject_to(kappa_fr == 0);

    % solve
    plugin_opts = struct('print_time',0);
    solver_opts = struct('print_level',0); %'constr_viol_tol',0.1,'acceptable_obj_change_tol',0.01,
    nlpFSP.solver('ipopt',plugin_opts,solver_opts);  
   
    if i == numel(FSP.Ax)

        nlpFSP.subject_to(V_out <= LSP.Vx(1));
        sol = nlpFSP.solve();
        % extract results
        FSP.V_current(1)           = sol.value(V_out);                % Velocity (m/s)

    else
        nlpFSP.subject_to(V_out <= LSP.Vx(i+1));
        sol = nlpFSP.solve();
        % extract results
        FSP.V_current(i+1)         = sol.value(V_out);                % Velocity (m/s)

    end

    % extract results
    FSP.Ay(i)                  = sol.value(ay_out);            % Lateral Acceleration (m/s^2)
    FSP.Ax(i)                  = sol.value(ax_out);            % Lateral Acceleration (m/s^2)
    FSP.Ax_control(i)          = sol.value(ax_control);
    FSP.delta(i)               = sol.value(delta);             % steering angle (rad)
    FSP.beta(i)                = sol.value(beta);              % sideslip angle (rad)
    FSP.yaw_rate(i)            = sol.value(yaw_rate);          % yaw rate (rad/s)
    FSP.throttle_position(i)   = sol.value(throttle_position); % throttle position (-)
    FSP.wheel_rot_fl(i)        = sol.value(wheel_rot_fl);      % FL wheel angular velocity (rad/s)
    FSP.wheel_rot_fr(i)        = sol.value(wheel_rot_fr);      % FR wheel angular velocity (rad/s)
    FSP.wheel_rot_rl(i)        = sol.value(wheel_rot_rl);      % RL wheel angular velocity (rad/s)
    FSP.wheel_rot_rr(i)        = sol.value(wheel_rot_rr);      % RR wheel angular velocity (rad/s)    
    FSP.F_tractive(i)          = sol.value(F_tractive);
    FSP.F_drag(i)              = sol.value(Fd);
end