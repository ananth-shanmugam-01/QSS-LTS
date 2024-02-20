%%%%%%%%%%%%%%%%%%%%%%%
% CasADi Problem Formulation of Vehicle Model
% Reference - Mario Boxheimer 

function FSP = forwardSpeed(i, sector_length, LSP, FSP, curvature, V_current, ay_control, carData)

    % initialize Problem
    import casadi.*
    
    nlpFSP = casadi.Opti();

    % decision variables & box constraints
    deltaScaled = nlpFSP.variable();                nlpFSP.subject_to(-1<=deltaScaled<=1);             % steering angle (rad)
    betaScaled = nlpFSP.variable();                 nlpFSP.subject_to(-1<=betaScaled<=1);                 % sideslip angle (rad)
    throttle_positionScaled = nlpFSP.variable();    nlpFSP.subject_to(0<=throttle_positionScaled<=1);     % throttle position (-)
    % brake_pressureScaled = nlpFSP.variable();           nlpFSP.subject_to(0<=brake_pressureScaled<=1);              % brake pressure (bar)
    wheel_rot_flScaled = nlpFSP.variable();         nlpFSP.subject_to(0<=wheel_rot_flScaled<=1)           % FL wheel angular velocity (rad/s)
    wheel_rot_frScaled = nlpFSP.variable();         nlpFSP.subject_to(0<=wheel_rot_frScaled<=1)           % FR wheel angular velocity (rad/s)
    wheel_rot_rlScaled = nlpFSP.variable();         nlpFSP.subject_to(0<=wheel_rot_rlScaled<=1)           % RL wheel angular velocity (rad/s)
    wheel_rot_rrScaled = nlpFSP.variable();         nlpFSP.subject_to(0<=wheel_rot_rrScaled<=1)           % RR wheel angular velocity (rad/s)
    
    % State Weights
    deltaWeight = 30*pi/180;
    betaWeight = 5*pi/180;
    throttle_position_weight = 1;
    brake_pressure_weight = 100;
    wheel_rot_weight = carData.Powertrain.vMax/carData.Chassis.radWheel;
   
    % Scaling for decision variables
    delta = deltaScaled * deltaWeight;
    beta = betaScaled * betaWeight;
    throttle_position = throttle_positionScaled  * throttle_position_weight;
    % brake_pressure = brake_pressureScaled * brake_pressure_weight;
    wheel_rot_fl = wheel_rot_flScaled * wheel_rot_weight;
    wheel_rot_fr = wheel_rot_frScaled * wheel_rot_weight;
    wheel_rot_rl = wheel_rot_rlScaled * wheel_rot_weight;
    wheel_rot_rr = wheel_rot_rrScaled * wheel_rot_weight;
    
    % Initial Value for Wheel Velocity - Scaled
    initWheelVel = (V_current/carData.Chassis.radWheel)/(wheel_rot_weight);

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
       
    % Constraints
    nlpFSP.subject_to(-0.05<=ay_res<=0.05);
    nlpFSP.subject_to(-0.05<=ax_res<=0.05);
    nlpFSP.subject_to(-5 <= Mz_out <= 5);
    nlpFSP.subject_to(kappa_fl == 0);
    nlpFSP.subject_to(kappa_fr == 0);

    % solve
    plugin_opts = struct('print_time',0);
    solver_opts = struct('print_level',0,'max_iter', 1000); %'constr_viol_tol',0.1,'acceptable_obj_change_tol',0.01,'print_level',0, 'max_iter', 400
    nlpFSP.solver('ipopt',plugin_opts,solver_opts);  
  
    nlpFSP.set_initial(wheel_rot_flScaled,initWheelVel);
    nlpFSP.set_initial(wheel_rot_frScaled,initWheelVel);
    nlpFSP.set_initial(wheel_rot_rlScaled,initWheelVel);
    nlpFSP.set_initial(wheel_rot_rrScaled,initWheelVel);

    if i == 1 % Harness continuity from last point on track

        % initialization of decision variables
        nlpFSP.set_initial(deltaScaled,FSP.delta(end)/deltaWeight);
        nlpFSP.set_initial(betaScaled,FSP.beta(end)/betaWeight);
        nlpFSP.set_initial(throttle_positionScaled,FSP.throttle_position(end));
        sol = nlpFSP.solve();

        if sol.value(V_out) > LSP.Vx(i+1)
            FSP.V_current(i+1) = LSP.Vx(i+1);
        else
            FSP.V_current(i+1) = sol.value(V_out); % Velocity (m/s)
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

    elseif i == numel(FSP.Ax) % Last point on the track, enforce continuity to the first point

        % initialization of decision variables
        nlpFSP.set_initial(deltaScaled,FSP.delta(i-1)/deltaWeight);
        nlpFSP.set_initial(betaScaled,FSP.beta(i-1)/betaWeight);
        nlpFSP.set_initial(throttle_positionScaled,FSP.throttle_position(i-1));
        sol = nlpFSP.solve();

        if sol.value(V_out) > LSP.Vx(1)
            FSP.V_current(1) = LSP.Vx(1);
        else
            FSP.V_current(1) = sol.value(V_out); % Velocity (m/s)
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

    else % Anywhere else on the track mesh

        % initialization of decision variables
        nlpFSP.set_initial(deltaScaled,FSP.delta(i-1)/deltaWeight);
        nlpFSP.set_initial(betaScaled,FSP.beta(i-1)/betaWeight);
        nlpFSP.set_initial(throttle_positionScaled,FSP.throttle_position(i-1));
        sol = nlpFSP.solve();

        if sol.value(V_out) > LSP.Vx(i+1)
            FSP.V_current(i+1) = LSP.Vx(i+1);
        else
            FSP.V_current(i+1) = sol.value(V_out); % Velocity (m/s)
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

end