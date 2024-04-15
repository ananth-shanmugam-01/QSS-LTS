%%%%%%%%%%%%%%%%%%%%%%%
% CasADi Problem Formulation of Vehicle Model
% Reference - Mario Boxheimer 
function GGV = acceleration(GGV, carData,velocity)

% initialize Problem
import casadi.*

FA = casadi.Opti();

% Input Values

% decision variables & box constraints
deltaScaled = FA.variable();                FA.subject_to(-1<=deltaScaled<=1);             % steering angle (rad)
betaScaled = FA.variable();                 FA.subject_to(-1<=betaScaled<=1);                 % sideslip angle (rad)
yaw_rateScaled = FA.variable();             FA.subject_to(-1<=yaw_rateScaled<=1);             % yaw rate (rad/s)
throttle_positionScaled = FA.variable();    FA.subject_to(0<=throttle_positionScaled<=1);     % throttle position (-)
% brake_pressureScaled = FA.variable();           FA.subject_to(0<=brake_pressureScaled<=1);              % brake pressure (bar)
wheel_rot_flScaled = FA.variable();         FA.subject_to(0<=wheel_rot_flScaled<=1)           % FL wheel angular velocity (rad/s)
wheel_rot_frScaled = FA.variable();         FA.subject_to(0<=wheel_rot_frScaled<=1)           % FR wheel angular velocity (rad/s)
wheel_rot_rlScaled = FA.variable();         FA.subject_to(0<=wheel_rot_rlScaled<=1)           % RL wheel angular velocity (rad/s)
wheel_rot_rrScaled = FA.variable();         FA.subject_to(0<=wheel_rot_rrScaled<=1)           % RR wheel angular velocity (rad/s)

inputs.Vx = velocity;
% Scaling for decision variables
inputs.delta        = deltaScaled * 30*pi/180;
inputs.beta         =  betaScaled * 5*pi/180;
inputs.yaw_rate     = yaw_rateScaled * 120*pi/180;
inputs.throttle_position = throttle_positionScaled;
inputs.brake_pressure = 0 * 100;
inputs.wheel_rot_fl = wheel_rot_flScaled * carData.Powertrain.vMax/carData.Chassis.radWheel;
inputs.wheel_rot_fr = wheel_rot_frScaled * carData.Powertrain.vMax/carData.Chassis.radWheel;
inputs.wheel_rot_rl = wheel_rot_rlScaled * carData.Powertrain.vMax/carData.Chassis.radWheel;
inputs.wheel_rot_rr = wheel_rot_rrScaled * carData.Powertrain.vMax/carData.Chassis.radWheel;


% Initial Guess for decision variables
initWheelVel = (velocity/carData.Chassis.radWheel)/(carData.Powertrain.vMax/carData.Chassis.radWheel);

% initialization of decision variables
FA.set_initial(deltaScaled,0);
FA.set_initial(betaScaled,0);
FA.set_initial(yaw_rateScaled,0);
FA.set_initial(throttle_positionScaled,1);
FA.set_initial(wheel_rot_flScaled,initWheelVel);
FA.set_initial(wheel_rot_frScaled,initWheelVel);
FA.set_initial(wheel_rot_rlScaled,initWheelVel);
FA.set_initial(wheel_rot_rrScaled,initWheelVel);


%% Call Vehicle Model

outputs = vehicleModels.carModel(inputs, carData);

%% Path Constraints

FA.subject_to(-12*pi/180<=outputs.alpha_fl<=12*pi/180);
FA.subject_to(-12*pi/180<=outputs.alpha_fr<=12*pi/180);
FA.subject_to(-12*pi/180<=outputs.alpha_rl<=12*pi/180);
FA.subject_to(-12*pi/180<=outputs.alpha_rr<=12*pi/180);

FA.subject_to(-0.1<=outputs.kappa_fl<=0.1);
FA.subject_to(-0.1<=outputs.kappa_fr<=0.1);
FA.subject_to(-0.1<=outputs.kappa_rl<=0.1);
FA.subject_to(-0.1<=outputs.kappa_rr<=0.1);

% objective
FA.minimize(-outputs.ax_out);

% Constraints
FA.subject_to(-0.05<=outputs.ay_res<=0.05);
FA.subject_to(-0.05<=outputs.ax_res<=0.05);
FA.subject_to(outputs.ay_out == 0);
FA.subject_to(outputs.kappa_fl == 0);
FA.subject_to(outputs.kappa_fr == 0);

plugin_opts = struct('print_time',0);
solver_opts = struct('print_level',0); % 'constr_viol_tol',0.1,'acceptable_obj_change_tol',0.001,
FA.solver('ipopt',plugin_opts,solver_opts);
sol = FA.solve();

%     extract results
GGV.ay(1)           = sol.value(outputs.ay_out);
GGV.ax(1)           = sol.value(outputs.ax_out);
GGV.delta(1)        = sol.value(inputs.delta);
GGV.beta(1)         = sol.value(inputs.beta);
GGV.yaw_rate(1)     = sol.value(inputs.yaw_rate);
GGV.wheel_rot_fl(1) = sol.value(inputs.wheel_rot_fl);
GGV.wheel_rot_fr(1) = sol.value(inputs.wheel_rot_fr);
GGV.wheel_rot_rl(1) = sol.value(inputs.wheel_rot_rl);
GGV.wheel_rot_rr(1) = sol.value(inputs.wheel_rot_rr);

end
