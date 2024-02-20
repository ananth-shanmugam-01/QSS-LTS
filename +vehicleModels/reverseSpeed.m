%%%%%%%%%%%%%%%%%%%%%%%
% CasADi Problem Formulation of Vehicle Model
% Reference - Mario Boxheimer 

function RSP = reverseSpeed(i, sector_length, LSP, RSP, curvature, V_current, ay_control, carData)

    % initialize Problem
    import casadi.*
    
    nlpRSP = casadi.Opti();

    % decision variables & box constraints
    V_outScaled = nlpRSP.variable();                nlpRSP.subject_to(0.1<=V_outScaled<=1)   
    deltaScaled = nlpRSP.variable();                nlpRSP.subject_to(-1<=deltaScaled<=1);                  % steering angle (rad)
    betaScaled = nlpRSP.variable();                 nlpRSP.subject_to(-1<=betaScaled<=1);                   % sideslip angle (rad)
    yaw_rateScaled = nlpRSP.variable();             nlpRSP.subject_to(-1<=yaw_rateScaled<=1);               % yaw rate (rad/s)
    % throttle_positionScaled = nlpRSP.variable();    nlpRSP.subject_to(0<=throttle_positionScaled<=1);     % throttle position (-)
    brake_pressureScaled = nlpRSP.variable();       nlpRSP.subject_to(0<=brake_pressureScaled<=1);            % brake pressure (bar)
    wheel_rot_flScaled = nlpRSP.variable();         nlpRSP.subject_to(0<=wheel_rot_flScaled<=1)             % FL wheel angular velocity (rad/s)
    wheel_rot_frScaled = nlpRSP.variable();         nlpRSP.subject_to(0<=wheel_rot_frScaled<=1)             % FR wheel angular velocity (rad/s)
    wheel_rot_rlScaled = nlpRSP.variable();         nlpRSP.subject_to(0<=wheel_rot_rlScaled<=1)             % RL wheel angular velocity (rad/s)
    wheel_rot_rrScaled = nlpRSP.variable();         nlpRSP.subject_to(0<=wheel_rot_rrScaled<=1)             % RR wheel angular velocity (rad/s)
    
    % State Weights
    V_out_weight = carData.Powertrain.vMax;
    deltaWeight = 30*pi/180;
    betaWeight = 5*pi/180;
    % throttle_position_weight = 1;
    brake_pressure_weight = 100;
    wheel_rot_weight = carData.Powertrain.vMax/carData.Chassis.radWheel;
   
    % Scaling for decision variables
    V_out = V_outScaled * V_out_weight; 
    delta = deltaScaled * deltaWeight;
    beta = betaScaled * betaWeight;
    % throttle_position = throttle_positionScaled  * throttle_position_weight;
    brake_pressure = brake_pressureScaled * brake_pressure_weight;
    wheel_rot_fl = wheel_rot_flScaled * wheel_rot_weight;
    wheel_rot_fr = wheel_rot_frScaled * wheel_rot_weight;
    wheel_rot_rl = wheel_rot_rlScaled * wheel_rot_weight;
    wheel_rot_rr = wheel_rot_rrScaled * wheel_rot_weight;
    
    % Initial Value for Wheel Velocity - Scaled
    initWheelVel = (V_current/carData.Chassis.radWheel)/(wheel_rot_weight);

    %% Equations of Motion
    
    yaw_rate = V_out * curvature;
    
    g = 9.81;
    DF_total = 0.5*1.225*carData.Aero.CLA*V_out^2;
    DF_front = carData.Aero.rAeroBalance*DF_total;
    DF_rear = (1-carData.Aero.rAeroBalance)*DF_front;
    Fd = 0.5*1.225*carData.Aero.CDA*V_out^2;
    
    % % Motor Tractive Force
    % torqueInterp = interpolant('LUT','bspline',{[carData.Powertrain.RPM]},carData.Powertrain.torqueMotor); % CasADi feature
    % wheel_avg_vel = 0.5*(wheel_rot_rl + wheel_rot_rr);
    % motor_rot_vel = wheel_avg_vel * carData.Powertrain.rGear *60/(2*pi);
    % F_tractive = throttle_position * carData.Powertrain.effPU * carData.Powertrain.nDrive * torqueInterp(motor_rot_vel) * carData.Powertrain.rGear/carData.Chassis.radWheel;

    % Braking Decelerative Force
    Front_Brake_Force = 2*(brake_pressure * 100 * carData.Brakes.rBrakeBias .*10^5 .*carData.Brakes.areaPiston .*carData.Brakes.nPistonFront) * carData.Brakes.muBrakePad * carData.Brakes.radBrakeDisc / carData.Chassis.radWheel; %N
    Rear_Brake_Force = 2*(brake_pressure * 100 *(1-carData.Brakes.rBrakeBias) .*10^5 .*carData.Brakes.areaPiston .*carData.Brakes.nPistonRear) * carData.Brakes.muBrakePad * carData.Brakes.radBrakeDisc / carData.Chassis.radWheel; %N
    F_brake = -(Front_Brake_Force + Rear_Brake_Force);

    ax_control = (F_brake - Fd) / carData.Chassis.mass;
    
    % Slip Angles
    alpha_fl = ((V_out*tan(beta) + carData.Chassis.frontMomentArm*yaw_rate) / (V_out + yaw_rate*carData.Chassis.trackWidth*0.5)) - (delta - carData.Suspension.aToeStaticFront*pi/180);
    alpha_fr = ((V_out*tan(beta) + carData.Chassis.frontMomentArm*yaw_rate) / (V_out - yaw_rate*carData.Chassis.trackWidth*0.5)) - (delta + carData.Suspension.aToeStaticFront*pi/180);
    alpha_rl = (V_out*tan(beta) - carData.Chassis.rearMomentArm*yaw_rate) / (V_out + yaw_rate*carData.Chassis.trackWidth*0.5) - (-carData.Suspension.aToeStaticRear*pi/180);
    alpha_rr = (V_out*tan(beta) - carData.Chassis.rearMomentArm*yaw_rate) / (V_out - yaw_rate*carData.Chassis.trackWidth*0.5) - (carData.Suspension.aToeStaticRear*pi/180);
    
    % Slip Ratios
    kappa_fl = (wheel_rot_fl*carData.Chassis.radWheel - V_out)/V_out;
    kappa_fr = (wheel_rot_fr*carData.Chassis.radWheel - V_out)/V_out;
    kappa_rl = (wheel_rot_rl*carData.Chassis.radWheel - V_out)/V_out;
    kappa_rr = (wheel_rot_rr*carData.Chassis.radWheel - V_out)/V_out;
    
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

    brakeBias_tyre = (fx_fl + fx_fr) / (fx_fl + fx_fr + fx_rl + fx_rr);
    
    % Car States
    ay_out = (fy_fl + fy_fr + fy_rl + fy_rr)/carData.Chassis.mass;
    ax_out = (fx_fl + fx_fr + fx_rl + fx_rr)/carData.Chassis.mass;
    Mz_out = (carData.Chassis.frontMomentArm*(fy_fl + fy_fr) + 0.5*carData.Chassis.trackWidth*(fx_fl + fx_rl) ...
              - carData.Chassis.rearMomentArm*(fy_rl + fy_rr) - 0.5*carData.Chassis.trackWidth*(fx_fr + fx_rr))/carData.Chassis.yawInertia;
    
    % Target Speed
    V_out_guess = sqrt(V_current^2 - 2*ax_out*sector_length);

    % Residuals
    ax_res = ax_control - ax_out;
    ay_res = ay_control - ay_out;
    Vx_res = V_out - V_out_guess;
    brakeBias_res = carData.Brakes.rBrakeBias - brakeBias_tyre;
    
    % Path Constraints

    nlpRSP.subject_to(-12*pi/180<=alpha_fl<=12*pi/180);
    nlpRSP.subject_to(-12*pi/180<=alpha_fr<=12*pi/180);
    nlpRSP.subject_to(-12*pi/180<=alpha_rl<=12*pi/180);
    nlpRSP.subject_to(-12*pi/180<=alpha_rr<=12*pi/180);

    nlpRSP.subject_to(-0.1<=kappa_fl<=0);
    nlpRSP.subject_to(-0.1<=kappa_fr<=0);
    nlpRSP.subject_to(-0.1<=kappa_rl<=0);
    nlpRSP.subject_to(-0.1<=kappa_rr<=0);

    % objective
    nlpRSP.minimize(-V_out);
    
    % initialization of decision variables
    nlpRSP.set_initial(V_outScaled,V_current/V_out_weight);
    nlpRSP.set_initial(deltaScaled,0);
    nlpRSP.set_initial(betaScaled,0);
    nlpRSP.set_initial(brake_pressureScaled,0);
    nlpRSP.set_initial(wheel_rot_flScaled,initWheelVel);
    nlpRSP.set_initial(wheel_rot_frScaled,initWheelVel);
    nlpRSP.set_initial(wheel_rot_rlScaled,initWheelVel);
    nlpRSP.set_initial(wheel_rot_rrScaled,initWheelVel);

    % Constraints
    nlpRSP.subject_to(-0.05<=ay_res<=0.05);
    nlpRSP.subject_to(-0.05<=ax_res<=0.05);
    nlpRSP.subject_to(-0.025<=brakeBias_res<=0.025);
    nlpRSP.subject_to(-5 <= Mz_out <= 5);
    nlpRSP.subject_to(-0.001 <= Vx_res <= 0.001);
    % nlpRSP.subject_to(V_out <= LSP.Vx(i-1));
    
    % solve
    plugin_opts = struct('print_time',0);
    solver_opts = struct('print_level',0); % 'constr_viol_tol',0.1,'acceptable_obj_change_tol',0.01,'print_level',0
    nlpRSP.solver('ipopt',plugin_opts,solver_opts);  
    sol = nlpRSP.solve();

    if sol.value(V_out) > LSP.Vx(i-1)
        % extract results
        RSP.V_current(i-1)         = LSP.Vx(i-1);                % Velocity (m/s)
    else
        % extract results
        RSP.V_current(i-1)         = sol.value(V_out);                % Velocity (m/s)
    end

    RSP.Ay(i)                  = sol.value(ay_out);            % Lateral Acceleration (m/s^2)
    RSP.Ax(i)                  = sol.value(ax_out);            % Lateral Acceleration (m/s^2)
    RSP.Ax_control(i)          = sol.value(ax_control);
    RSP.delta(i)               = sol.value(delta);             % steering angle (rad)
    RSP.beta(i)                = sol.value(beta);              % sideslip angle (rad)
    RSP.yaw_rate(i)            = sol.value(yaw_rate);          % yaw rate (rad/s)
    RSP.brake_pressure(i)      = sol.value(brake_pressure);    % throttle position (-)
    RSP.wheel_rot_fl(i)        = sol.value(wheel_rot_fl);      % FL wheel angular velocity (rad/s)
    RSP.wheel_rot_fr(i)        = sol.value(wheel_rot_fr);      % FR wheel angular velocity (rad/s)
    RSP.wheel_rot_rl(i)        = sol.value(wheel_rot_rl);      % RL wheel angular velocity (rad/s)
    RSP.wheel_rot_rr(i)        = sol.value(wheel_rot_rr);      % RR wheel angular velocity (rad/s)
    RSP.F_brake(i)             = sol.value(F_brake);
    RSP.F_drag(i)              = sol.value(Fd);

    end