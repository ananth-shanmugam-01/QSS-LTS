%%%%%%%%%%%%%%%%%%%%%%%
% CasADi Problem Formulation of Vehicle Model
% Reference - Mario Boxheimer 

function GGV = deceleration(GGV,carData,velocity)

%% initialize Problem
import casadi.*

BA = casadi.Opti();

% decision variables & box constraints
deltaScaled = BA.variable();                BA.subject_to(-1<=deltaScaled<=1);                  % steering angle (rad)
betaScaled = BA.variable();                 BA.subject_to(-1<=betaScaled<=1);                   % sideslip angle (rad)
yaw_rateScaled = BA.variable();             BA.subject_to(-1<=yaw_rateScaled<=1);               % yaw rate (rad/s)
% throttle_positionScaled = BA.variable();    BA.subject_to(0<=throttle_positionScaled<=1);     % throttle position (-)
brake_pressureScaled = BA.variable();       BA.subject_to(0<=brake_pressureScaled<=1);            % brake pressure (bar)
wheel_rot_flScaled = BA.variable();         BA.subject_to(0<=wheel_rot_flScaled<=1)             % FL wheel angular velocity (rad/s)
wheel_rot_frScaled = BA.variable();         BA.subject_to(0<=wheel_rot_frScaled<=1)             % FR wheel angular velocity (rad/s)
wheel_rot_rlScaled = BA.variable();         BA.subject_to(0<=wheel_rot_rlScaled<=1)             % RL wheel angular velocity (rad/s)
wheel_rot_rrScaled = BA.variable();         BA.subject_to(0<=wheel_rot_rrScaled<=1)             % RR wheel angular velocity (rad/s)

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
BA.set_initial(deltaScaled,0);
BA.set_initial(betaScaled,0);
BA.set_initial(yaw_rateScaled,0);
BA.set_initial(brake_pressureScaled,0);
BA.set_initial(wheel_rot_flScaled,initWheelVel);
BA.set_initial(wheel_rot_frScaled,initWheelVel);
BA.set_initial(wheel_rot_rlScaled,initWheelVel);
BA.set_initial(wheel_rot_rrScaled,initWheelVel);

%% Call vehicle moodel

outputs = vehicleModels.carModel(inputs, carData);

%% objective
BA.minimize(outputs.ax_out);

%% Path Constraints

BA.subject_to(-12*pi/180<=outputs.alpha_fl<=12*pi/180);
BA.subject_to(-12*pi/180<=outputs.alpha_fr<=12*pi/180);
BA.subject_to(-12*pi/180<=outputs.alpha_rl<=12*pi/180);
BA.subject_to(-12*pi/180<=outputs.alpha_rr<=12*pi/180);

BA.subject_to(-0.10<=outputs.kappa_fl<=0);
BA.subject_to(-0.10<=outputs.kappa_fr<=0);
BA.subject_to(-0.10<=outputs.kappa_rl<=0);
BA.subject_to(-0.10<=outputs.kappa_rr<=0);

% Constraints
brakeBias_res = carData.Brakes.rBrakeBias - outputs.fxDistribution;

BA.subject_to(-0.05<=outputs.ay_res<=0.05);
BA.subject_to(-0.05<=outputs.ax_res<=0.05);
BA.subject_to(-0.025<=brakeBias_res<=0.025);
BA.subject_to(outputs.ay_out == 0);

%% Solver Settings
plugin_opts = struct('print_time',0);
solver_opts = struct('print_level',0); % 'constr_viol_tol',0.1,'acceptable_obj_change_tol',0.001,
BA.solver('ipopt',plugin_opts,solver_opts);
sol = BA.solve();

%% Extract Results
GGV.ay(end)           = sol.value(outputs.ay_out);
GGV.ax(end)           = sol.value(outputs.ax_out);
GGV.delta(end)        = sol.value(inputs.delta);
GGV.beta(end)         = sol.value(inputs.beta);
GGV.yaw_rate(end)     = sol.value(inputs.yaw_rate);
GGV.wheel_rot_fl(end) = sol.value(inputs.wheel_rot_fl);
GGV.wheel_rot_fr(end) = sol.value(inputs.wheel_rot_fr);
GGV.wheel_rot_rl(end) = sol.value(inputs.wheel_rot_rl);
GGV.wheel_rot_rr(end) = sol.value(inputs.wheel_rot_rr);

end