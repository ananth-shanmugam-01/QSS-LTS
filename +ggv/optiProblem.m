%% Single GGV Solver
% switch cases to enable/disable decision variables
% intent is to have a single point of control to link all model and control
% changes

function GGV = optiProblem(velocity, carData, GGV, index, maneuver, ax_target)

    % initialize Problem
    import casadi.*
    
    opti = casadi.Opti();
    
    %% Common Decision Variables
    
    % decision variables & box constraints
    deltaScaled = opti.variable();                opti.subject_to(-1<=deltaScaled<=1);                % steering angle (rad)
    betaScaled = opti.variable();                 opti.subject_to(-1<=betaScaled<=1);                 % sideslip angle (rad)
    yaw_rateScaled = opti.variable();             opti.subject_to(-1<=yaw_rateScaled<=1);             % yaw rate (rad/s)
    wheel_rot_flScaled = opti.variable();         opti.subject_to(0<=wheel_rot_flScaled<=1)           % FL wheel angular velocity (rad/s)
    wheel_rot_frScaled = opti.variable();         opti.subject_to(0<=wheel_rot_frScaled<=1)           % FR wheel angular velocity (rad/s)
    wheel_rot_rlScaled = opti.variable();         opti.subject_to(0<=wheel_rot_rlScaled<=1)           % RL wheel angular velocity (rad/s)
    wheel_rot_rrScaled = opti.variable();         opti.subject_to(0<=wheel_rot_rrScaled<=1)           % RR wheel angular velocity (rad/s)
    
    
    inputs.Vx = velocity;
    % Scaling for decision variables
    inputs.delta        = deltaScaled * 30*pi/180;
    inputs.beta         = betaScaled * 5*pi/180;
    inputs.yaw_rate     = yaw_rateScaled * 120*pi/180;
    inputs.wheel_rot_fl = wheel_rot_flScaled * carData.Powertrain.vMax/carData.Chassis.radWheel;
    inputs.wheel_rot_fr = wheel_rot_frScaled * carData.Powertrain.vMax/carData.Chassis.radWheel;
    inputs.wheel_rot_rl = wheel_rot_rlScaled * carData.Powertrain.vMax/carData.Chassis.radWheel;
    inputs.wheel_rot_rr = wheel_rot_rrScaled * carData.Powertrain.vMax/carData.Chassis.radWheel;
    
    initWheelVel = (velocity/carData.Chassis.radWheel)/(carData.Powertrain.vMax/carData.Chassis.radWheel);
    
    % Initial Guess for common decision variables
    opti.set_initial(deltaScaled,0);
    opti.set_initial(betaScaled,0);
    opti.set_initial(yaw_rateScaled,0);
    opti.set_initial(wheel_rot_flScaled,initWheelVel);
    opti.set_initial(wheel_rot_frScaled,initWheelVel);
    opti.set_initial(wheel_rot_rlScaled,initWheelVel);
    opti.set_initial(wheel_rot_rrScaled,initWheelVel);
    
    
    %% Sequence dependent decision variables
    
    switch maneuver
    
        case 'acceleration'
            throttle_positionScaled = opti.variable();    opti.subject_to(0<=throttle_positionScaled<=1);     % throttle position (-)
            inputs.throttle_position = throttle_positionScaled;
            opti.set_initial(throttle_positionScaled,0.1);
    
            % Set brake pressure to zero
            inputs.brake_pressure = 0 * 100;  
        
        case 'deceleration'
            brake_pressureScaled = opti.variable();     opti.subject_to(0<=brake_pressureScaled<=1);           % brake pressure (bar)
            inputs.brake_pressure = brake_pressureScaled * 100;
            opti.set_initial( brake_pressureScaled,0);
            
            % Set throttle position to zero
            inputs.throttle_position = 0;
    
        case 'combinedacceleration'
            throttle_positionScaled = opti.variable();    opti.subject_to(0<=throttle_positionScaled<=1);     % throttle position (-)
            inputs.throttle_position = throttle_positionScaled;
            opti.set_initial(throttle_positionScaled,1);
    
            % Set brake pressure to zero
            inputs.brake_pressure = 0 * 100;  
        
        case 'combineddeceleration'
            brake_pressureScaled = opti.variable();     opti.subject_to(0<=brake_pressureScaled<=1);          % brake pressure (bar)
            inputs.brake_pressure = brake_pressureScaled * 100;
            opti.set_initial(brake_pressureScaled,0);
    
            % Set throttle position to zero
            inputs.throttle_position = 0;
    end
    
    
    %% Call Vehicle Model
    
    outputs = vehicleModels.carModel(inputs, carData);
    
    
    %% Sequence Objective and Constraints
    switch maneuver
    
        case 'acceleration'
                opti.minimize(-outputs.ax_out);
                opti.subject_to(outputs.ay_out == 0);
                opti.subject_to(outputs.kappa_fl == 0);
                opti.subject_to(outputs.kappa_fr == 0);
        
        case 'deceleration'
                opti.minimize(outputs.ax_out);
                opti.subject_to(outputs.ay_out == 0);
                brakeBias_res = carData.Brakes.rBrakeBias - outputs.fxDistribution;
                opti.subject_to(-0.025<=brakeBias_res<=0.025);
    
        case 'combinedacceleration'
                opti.minimize(-outputs.ay_out);
                opti.subject_to(outputs.kappa_fl == 0);
                opti.subject_to(outputs.kappa_fr == 0);

                ax_constraint_res = outputs.ax_out - ax_target;   
                if ax_target > 1
                    opti.subject_to(-0.05<=ax_constraint_res<=0.05);
                else
                end
        
        case 'combineddeceleration'
                opti.minimize(-outputs.ay_out);   
    
                ax_constraint_res = outputs.ax_out - ax_target;
                brakeBias_res = carData.Brakes.rBrakeBias - outputs.fxDistribution;
    
                if ax_target < -3
                    opti.subject_to(-0.05<=ax_constraint_res<=0.05);
                else
                end
                opti.subject_to(-0.025<=brakeBias_res<=0.025);

    end
    
    
    %% Common Path Constraints
    
    opti.subject_to(-12*pi/180<=outputs.alpha_fl<=12*pi/180);
    opti.subject_to(-12*pi/180<=outputs.alpha_fr<=12*pi/180);
    opti.subject_to(-12*pi/180<=outputs.alpha_rl<=12*pi/180);
    opti.subject_to(-12*pi/180<=outputs.alpha_rr<=12*pi/180);

    opti.subject_to(-0.1<=outputs.kappa_fl<=0.1);
    opti.subject_to(-0.1<=outputs.kappa_fr<=0.1);
    opti.subject_to(-0.1<=outputs.kappa_rl<=0.1);
    opti.subject_to(-0.1<=outputs.kappa_rr<=0.1);

    % Non-negative wheel forces
    opti.subject_to(0<=outputs.w_fl);
    opti.subject_to(0<=outputs.w_fr);
    opti.subject_to(0<=outputs.w_rl);
    opti.subject_to(0<=outputs.w_rr);
   
    %% Vehicle State Residuals
    opti.subject_to(-0.05<=outputs.ay_res<=0.05);
    opti.subject_to(-0.05<=outputs.ax_res<=0.05);
    opti.subject_to(-5 <= outputs.Mz_out <= 5);
    
    %% Solver Settings
    plugin_opts = struct('print_time',0);
    solver_opts = struct('print_level',0);
    opti.solver('ipopt',plugin_opts,solver_opts);
    sol = opti.solve();
    
    %% Obtain Outputs
    GGV.ay(index)           = sol.value(outputs.ay_out);
    GGV.ax(index)           = sol.value(outputs.ax_out);
    GGV.delta(index)        = sol.value(inputs.delta);
    GGV.beta(index)         = sol.value(inputs.beta);
    GGV.yaw_rate(index)     = sol.value(inputs.yaw_rate);
    GGV.wheel_rot_fl(index) = sol.value(inputs.wheel_rot_fl);
    GGV.wheel_rot_fr(index) = sol.value(inputs.wheel_rot_fr);
    GGV.wheel_rot_rl(index) = sol.value(inputs.wheel_rot_rl);
    GGV.wheel_rot_rr(index) = sol.value(inputs.wheel_rot_rr);

    GGV.throttle_position(index) = sol.value(inputs.throttle_position);
    GGV.brake_pressure(index)    = sol.value(inputs.brake_pressure)*100;

end