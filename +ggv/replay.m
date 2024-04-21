function result = replay(index, initialSolution, targets, result, carData)

import casadi.*

replay = casadi.Opti();

% Input Values
Vx          = targets.Vel(index);
ax_target   = targets.gLong(index);
ay_target   = targets.gLat(index); 
yaw_rate_target =  ay_target/Vx; 

% decision variables & box constraints
deltaScaled = replay.variable();                replay.subject_to(-1<=deltaScaled<=1);                % steering angle (rad)
betaScaled = replay.variable();                 replay.subject_to(-1<=betaScaled<=1);                 % sideslip angle (rad)
wheel_rot_flScaled = replay.variable();         replay.subject_to(0<=wheel_rot_flScaled<=1)           % FL wheel angular velocity (rad/s)
wheel_rot_frScaled = replay.variable();         replay.subject_to(0<=wheel_rot_frScaled<=1)           % FR wheel angular velocity (rad/s)
wheel_rot_rlScaled = replay.variable();         replay.subject_to(0<=wheel_rot_rlScaled<=1)           % RL wheel angular velocity (rad/s)
wheel_rot_rrScaled = replay.variable();         replay.subject_to(0<=wheel_rot_rrScaled<=1)           % RR wheel angular velocity (rad/s)

inputs.Vx           = Vx;
inputs.yaw_rate     = yaw_rate_target;
% Scaling for decision variables
inputs.delta        = deltaScaled * 30*pi/180;
inputs.beta         = betaScaled * 5*pi/180;
inputs.wheel_rot_fl = wheel_rot_flScaled * carData.Powertrain.vMax/carData.Chassis.radWheel;
inputs.wheel_rot_fr = wheel_rot_frScaled * carData.Powertrain.vMax/carData.Chassis.radWheel;
inputs.wheel_rot_rl = wheel_rot_rlScaled * carData.Powertrain.vMax/carData.Chassis.radWheel;
inputs.wheel_rot_rr = wheel_rot_rrScaled * carData.Powertrain.vMax/carData.Chassis.radWheel;

% Case Dependent Decision Variables and Constraints

if ax_target >=0.1 % During Acceleration

    throttle_positionScaled = replay.variable();    replay.subject_to(0<=throttle_positionScaled<=1);     % throttle position (-)
    inputs.throttle_position = throttle_positionScaled;
    inputs.brake_pressure = 0 * 100;  

    replay.set_initial(throttle_positionScaled,0.1);

elseif ax_target <=-0.1 % During Deceleration
  
    brake_pressureScaled = replay.variable();     replay.subject_to(0<=brake_pressureScaled<=1);           % brake pressure (bar)
    inputs.brake_pressure = brake_pressureScaled * 100;
    inputs.throttle_position = 0;

    replay.set_initial(brake_pressureScaled,0);

else % Approximate steady-state cornering

    % Both Throttle and Brake available as decision variables

    throttle_positionScaled = replay.variable();    replay.subject_to(0<=throttle_positionScaled<=1);     % throttle position (-)
    inputs.throttle_position = throttle_positionScaled;
    brake_pressureScaled = replay.variable();     replay.subject_to(0<=brake_pressureScaled<=0.1);           % brake pressure (bar)
    inputs.brake_pressure = brake_pressureScaled * 100;
    
    replay.subject_to(inputs.throttle_position <= 0.05);
    replay.subject_to(inputs.brake_pressure <= 0.05);

end


%% Vehicle Model

outputs = vehicleModels.carModel(inputs, carData);

%% Path Constraints
replay.subject_to(-5 <= outputs.Mz_out <= 5);

replay.subject_to(-12*pi/180<=outputs.alpha_fl<=12*pi/180);
replay.subject_to(-12*pi/180<=outputs.alpha_fr<=12*pi/180);
replay.subject_to(-12*pi/180<=outputs.alpha_rl<=12*pi/180);
replay.subject_to(-12*pi/180<=outputs.alpha_rr<=12*pi/180);

replay.subject_to(-0.1<=outputs.kappa_fl<=0.1);
replay.subject_to(-0.1<=outputs.kappa_fr<=0.1);
replay.subject_to(-0.1<=outputs.kappa_rl<=0.1);
replay.subject_to(-0.1<=outputs.kappa_rr<=0.1);

%% objective - minimise error in achieved and target states

% Additional Constraints
ax_constraint_res = outputs.ax_out - ax_target;
ay_constraint_res = outputs.ay_out - ay_target;
brakeBias_res     = outputs.fxDistribution - carData.Brakes.rBrakeBias;

objective = outputs.ax_res^2 + outputs.ay_res^2 + ax_constraint_res^2 + ay_constraint_res^2;
replay.minimize(objective);

% Replay Constraints


if ax_target <=-0.1 % During Deceleration

    replay.subject_to(-0.025<=brakeBias_res<=0.025);

end

% solve
plugin_opts = struct('print_time',0);
solver_opts = struct('print_level',0); %'print_level',0, 'constr_viol_tol',0.01,'acceptable_obj_change_tol',0.01,
replay.solver('ipopt',plugin_opts,solver_opts);
    
% Spline Interpolated Initial Conditions
replay.set_initial(deltaScaled,deg2rad(initialSolution.delta(index))/(30*pi/180));
replay.set_initial(betaScaled,deg2rad(initialSolution.beta(index))/(5*pi/180));
replay.set_initial(wheel_rot_flScaled,initialSolution.wheel_rot_fl(index)/(carData.Powertrain.vMax/carData.Chassis.radWheel));
replay.set_initial(wheel_rot_frScaled,initialSolution.wheel_rot_fr(index)/(carData.Powertrain.vMax/carData.Chassis.radWheel));
replay.set_initial(wheel_rot_rlScaled,initialSolution.wheel_rot_rl(index)/(carData.Powertrain.vMax/carData.Chassis.radWheel));
replay.set_initial(wheel_rot_rrScaled,initialSolution.wheel_rot_rr(index)/(carData.Powertrain.vMax/carData.Chassis.radWheel));   

sol = replay.solve();
    
% extract results
result.vCar(index)          = Vx;
result.gLat(index)          = sol.value(outputs.ay_out);
result.gLong(index)         = sol.value(outputs.ax_out);
result.rThrottle(index)     = sol.value(inputs.throttle_position);
result.pBrake(index)        = sol.value(inputs.brake_pressure)*100; 
result.aSteer(index)        = sol.value(inputs.delta);
result.aBeta(index)         = sol.value(inputs.beta);
result.yawRate(index)       = sol.value(inputs.yaw_rate);
result.nWheelRotFL(index)   = sol.value(inputs.wheel_rot_fl);
result.nWheelRotFR(index)   = sol.value(inputs.wheel_rot_fr);
result.nWheelRotRL(index)   = sol.value(inputs.wheel_rot_rl);
result.nWheelRotRR(index)   = sol.value(inputs.wheel_rot_rr);

% Auxilliary Outputs

% Wheel Kinematics
result.aSlipAngleFL(index) = sol.value(outputs.alpha_fl);
result.aSlipAngleFR(index) = sol.value(outputs.alpha_fr);
result.aSlipAngleRL(index) = sol.value(outputs.alpha_rl);
result.aSlipAngleRR(index) = sol.value(outputs.alpha_rr);
result.aSlipRatioFL(index) = sol.value(outputs.kappa_fl);
result.aSlipRatioFR(index) = sol.value(outputs.kappa_fr);
result.aSlipRatioRL(index) = sol.value(outputs.kappa_rl);
result.aSlipRatioRR(index) = sol.value(outputs.kappa_rr);

% Wheel Forces
result.FzTyreFL(index) = sol.value(outputs.w_fl);
result.FzTyreFR(index) = sol.value(outputs.w_fr);
result.FzTyreRL(index) = sol.value(outputs.w_rl);
result.FzTyreRR(index) = sol.value(outputs.w_rr);

result.FyTyreFL(index) = sol.value(outputs.fy_fl);
result.FyTyreFR(index) = sol.value(outputs.fy_fr);
result.FyTyreRL(index) = sol.value(outputs.fy_rl);
result.FyTyreRR(index) = sol.value(outputs.fy_rr);

result.FxTyreFL(index) = sol.value(outputs.fx_fl);
result.FxTyreFR(index) = sol.value(outputs.fx_fr);
result.FxTyreRL(index) = sol.value(outputs.fx_rl);
result.FxTyreRR(index) = sol.value(outputs.fx_rr);

% Body Forces
result.FBrakeFront(index) = sol.value(outputs.FxBrake_front);
result.FBrakeRear(index)  = sol.value(outputs.FxBrake_rear);
result.FTractive(index)   = sol.value(outputs.FxTractive);
result.FDownforceTotal(index) = sol.value(outputs.FzAero_total);
result.FDownforceFront(index) = sol.value(outputs.FzAero_front);
result.FDownforceRear(index)  = sol.value(outputs.FzAero_rear);
result.FDrag(index)           = sol.value(outputs.FxAero_drag);

disp(['Replayed States at Node: ', num2str(index)])


end
