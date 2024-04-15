%%%%%%%%%%%%%%%%%%%%%%%
% CasADi Problem Formulation of Vehicle Model
% Reference - Mario Boxheimer 

function GGV = combinedAccelerationBraking(index, GGV, carData, velocity)

% initialize Problem
import casadi.*

CAB = casadi.Opti();

% decision variables & box constraints
deltaScaled = CAB.variable();                CAB.subject_to(-1<=deltaScaled<=1);                  % steering angle (rad)
betaScaled = CAB.variable();                 CAB.subject_to(-1<=betaScaled<=1);                   % sideslip angle (rad)
yaw_rateScaled = CAB.variable();             CAB.subject_to(-1<=yaw_rateScaled<=1);               % yaw rate (rad/s)
% throttle_positionScaled = CAB.variable();    CAB.subject_to(0<=throttle_positionScaled<=1);     % throttle position (-)
brake_pressureScaled = CAB.variable();       CAB.subject_to(0<=brake_pressureScaled<=1);            % brake pressure (bar)
wheel_rot_flScaled = CAB.variable();         CAB.subject_to(0<=wheel_rot_flScaled<=1)             % FL wheel angular velocity (rad/s)
wheel_rot_frScaled = CAB.variable();         CAB.subject_to(0<=wheel_rot_frScaled<=1)             % FR wheel angular velocity (rad/s)
wheel_rot_rlScaled = CAB.variable();         CAB.subject_to(0<=wheel_rot_rlScaled<=1)             % RL wheel angular velocity (rad/s)
wheel_rot_rrScaled = CAB.variable();         CAB.subject_to(0<=wheel_rot_rrScaled<=1)             % RR wheel angular velocity (rad/s)


% Input Values
inputs.Vx = velocity;
% Scaling for decision variables
inputs.delta = deltaScaled * 30*pi/180;
inputs.beta = betaScaled * 5*pi/180;
inputs.yaw_rate = yaw_rateScaled * 120*pi/180;
inputs.throttle_position = 0;
inputs.brake_pressure = brake_pressureScaled * 100;
inputs.wheel_rot_fl = wheel_rot_flScaled * carData.Powertrain.vMax/carData.Chassis.radWheel;
inputs.wheel_rot_fr = wheel_rot_frScaled * carData.Powertrain.vMax/carData.Chassis.radWheel;
inputs.wheel_rot_rl = wheel_rot_rlScaled * carData.Powertrain.vMax/carData.Chassis.radWheel;
inputs.wheel_rot_rr = wheel_rot_rrScaled * carData.Powertrain.vMax/carData.Chassis.radWheel;

% Initial Value for decision variables
initWheelVel = (velocity/carData.Chassis.radWheel)/(carData.Powertrain.vMax/carData.Chassis.radWheel);

% initialization of decision variables
CAB.set_initial(deltaScaled,0);
CAB.set_initial(betaScaled,0);
CAB.set_initial(yaw_rateScaled,0);
CAB.set_initial(brake_pressureScaled,0);
CAB.set_initial(wheel_rot_flScaled,initWheelVel);
CAB.set_initial(wheel_rot_frScaled,initWheelVel);
CAB.set_initial(wheel_rot_rlScaled,initWheelVel);
CAB.set_initial(wheel_rot_rrScaled,initWheelVel);


%% call vehicle model

outputs = vehicleModels.carModel(inputs, carData);

%% Objective
CAB.minimize(-outputs.ay_out);

%% Path Constraints

CAB.subject_to(-12*pi/180<=outputs.alpha_fl<=12*pi/180);
CAB.subject_to(-12*pi/180<=outputs.alpha_fr<=12*pi/180);
CAB.subject_to(-12*pi/180<=outputs.alpha_rl<=12*pi/180);
CAB.subject_to(-12*pi/180<=outputs.alpha_rr<=12*pi/180);

CAB.subject_to(-0.10<=outputs.kappa_fl<=0);
CAB.subject_to(-0.10<=outputs.kappa_fr<=0);
CAB.subject_to(-0.10<=outputs.kappa_rl<=0);
CAB.subject_to(-0.10<=outputs.kappa_rr<=0);

ax_constraint_res = outputs.ax_out - GGV.ax(index);
brakeBias_res = carData.Brakes.rBrakeBias - outputs.fxDistribution;

try
    % steady state constraints
    CAB.subject_to(-0.05<=outputs.ay_res<=0.05);
    CAB.subject_to(-0.05<=outputs.ax_res<=0.05);
    if GGV.ax(index) < -3
        CAB.subject_to(-0.05<=ax_constraint_res<=0.05);
    else
    end
    CAB.subject_to(-0.025<=brakeBias_res<=0.025);
    CAB.subject_to(-5 <= outputs.Mz_out <= 5);
    
    % solve
    plugin_opts = struct('print_time',0);
    solver_opts = struct('print_level',0); % 'constr_viol_tol',0.1,'acceptable_obj_change_tol',0.01,
    CAB.solver('ipopt',plugin_opts,solver_opts);
    sol = CAB.solve();

    % extract results
    GGV.ay(index)           = sol.value(outputs.ay_out);
    GGV.ax(index)           = sol.value(outputs.ax_out);
    GGV.delta(index)        = sol.value(inputs.delta);
    GGV.beta(index)         = sol.value(inputs.beta);
    GGV.yaw_rate(index)     = sol.value(inputs.yaw_rate);
    GGV.wheel_rot_fl(index) = sol.value(inputs.wheel_rot_fl);
    GGV.wheel_rot_fr(index) = sol.value(inputs.wheel_rot_fr);
    GGV.wheel_rot_rl(index) = sol.value(inputs.wheel_rot_rl);
    GGV.wheel_rot_rr(index) = sol.value(inputs.wheel_rot_rr);

catch 
    disp(['Iteration Failed; ', 'Velocity: ', num2str(Vx), '; ', 'Ax Target: ', num2str(GGV.ax(index))])
end


end