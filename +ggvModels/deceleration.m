%%%%%%%%%%%%%%%%%%%%%%%
% CasADi Problem Formulation of Vehicle Model
% Reference - Mario Boxheimer 

function GGV = deceleration(GGV,carData,velocity)

%% initialize Problem
import casadi.*

BA = casadi.Opti();

% Input Values
Vx = velocity;

% decision variables & box constraints
delta = BA.variable(); BA.subject_to(-30*pi/180<=delta<=30*pi/180);                                                      % steering angle (rad)
beta = BA.variable(); BA.subject_to(-5*pi/180<=beta<=5*pi/180);                                                          % sideslip angle (rad)
yaw_rate = BA.variable(); BA.subject_to(-120*pi/180<=yaw_rate<=120*pi/180);                                                % yaw rate (rad/s)
% throttle_position = BA.variable(); BA.subject_to(0<=throttle_position<=1);                                               % throttle position (-)
brake_pressure = BA.variable(); BA.subject_to(0<=brake_pressure<=1);                                                     % brake pressure (bar)
wheel_rot_fl = BA.variable(); BA.subject_to(0<=wheel_rot_fl<=carData.Powertrain.vMax/carData.Chassis.radWheel)           % FL wheel angular velocity (rad/s)
wheel_rot_fr = BA.variable(); BA.subject_to(0<=wheel_rot_fr<=carData.Powertrain.vMax/carData.Chassis.radWheel)           % FR wheel angular velocity (rad/s)
wheel_rot_rl = BA.variable(); BA.subject_to(0<=wheel_rot_rl<=carData.Powertrain.vMax/carData.Chassis.radWheel)           % RL wheel angular velocity (rad/s)
wheel_rot_rr = BA.variable(); BA.subject_to(0<=wheel_rot_rr<=carData.Powertrain.vMax/carData.Chassis.radWheel)           % RR wheel angular velocity (rad/s)

%% Equations of Motion

g = 9.81;
DF_total = 0.5*1.225*carData.Aero.CLA*Vx^2;
DF_front = carData.Aero.rAeroBalance*DF_total;
DF_rear = (1-carData.Aero.rAeroBalance)*DF_front;
Fd = 0.5*1.225*carData.Aero.CDA*Vx^2;

ay_control = Vx*yaw_rate;

% % Motor Tractive Force
% torqueInterp = interpolant('LUT','bspline',{[carData.Powertrain.RPM]},carData.Powertrain.torqueMotor); % CasADi feature
% wheel_avg_vel = 0.5*(wheel_rot_rl + wheel_rot_rr);
% motor_rot_vel = wheel_avg_vel * carData.Powertrain.rGear *60/(2*pi);
% F_tractive = throttle_position * carData.Powertrain.effPU * carData.Powertrain.nDrive * torqueInterp(motor_rot_vel) * carData.Powertrain.rGear/carData.Chassis.radWheel;

% Braking Decelerative Force
Front_Brake_Force = 2*(brake_pressure * 100 * carData.Brakes.rBrakeBias .*10^5 .*carData.Brakes.areaPiston .*carData.Brakes.nPistonFront) * carData.Brakes.muBrakePad * carData.Brakes.radBrakeDisc / carData.Chassis.radWheel; %N
Rear_Brake_Force = 2*(brake_pressure * 100 *(1-carData.Brakes.rBrakeBias) .*10^5 .*carData.Brakes.areaPiston .*carData.Brakes.nPistonFront) * carData.Brakes.muBrakePad * carData.Brakes.radBrakeDisc / carData.Chassis.radWheel; %N
F_brake = -(Front_Brake_Force + Rear_Brake_Force);

ax_control = (F_brake - Fd) / carData.Chassis.mass;

% Slip Angles
alpha_fl = ((Vx*tan(beta) + carData.Chassis.frontMomentArm*yaw_rate) / (Vx + yaw_rate*carData.Chassis.trackWidth*0.5)) - delta;
alpha_fr = ((Vx*tan(beta) + carData.Chassis.frontMomentArm*yaw_rate) / (Vx - yaw_rate*carData.Chassis.trackWidth*0.5)) - delta;
alpha_rl = (Vx*tan(beta) - carData.Chassis.rearMomentArm*yaw_rate) / (Vx + yaw_rate*carData.Chassis.trackWidth*0.5);
alpha_rr = (Vx*tan(beta) - carData.Chassis.rearMomentArm*yaw_rate) / (Vx - yaw_rate*carData.Chassis.trackWidth*0.5);

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
[fy_fl, fx_fl] = MF52_Combined(kappa_fl,alpha_fl,w_fl,0);
[fy_fr, fx_fr] = MF52_Combined(kappa_fr,alpha_fr,w_fr,0);
[fy_rl, fx_rl] = MF52_Combined(kappa_rl,alpha_rl,w_rl,0);
[fy_rr, fx_rr] = MF52_Combined(kappa_rr,alpha_rr,w_rr,0);

% Car States
ay_out = (fy_fl + fy_fr + fy_rl + fy_rr)/carData.Chassis.mass;
ax_out = (fx_fl + fx_fr + fx_rl + fx_rr)/carData.Chassis.mass;
Mz_out = (carData.Chassis.frontMomentArm*(fy_fl + fy_fr) + 0.5*carData.Chassis.trackWidth*(fx_fl + fx_rl) ...
          - carData.Chassis.rearMomentArm*(fy_rl + fy_rr) - 0.5*carData.Chassis.trackWidth*(fx_fr + fx_rr))/carData.Chassis.yawInertia;

% Path Constraints

BA.subject_to(-12*pi/180<=alpha_fl<=12*pi/180);
BA.subject_to(-12*pi/180<=alpha_fr<=12*pi/180);
BA.subject_to(-12*pi/180<=alpha_rl<=12*pi/180);
BA.subject_to(-12*pi/180<=alpha_rr<=12*pi/180);

BA.subject_to(-0.1<=kappa_fl<=0.1);
BA.subject_to(-0.1<=kappa_fr<=0.1);
BA.subject_to(-0.1<=kappa_rl<=0.1);
BA.subject_to(-0.1<=kappa_rr<=0.1);

% objective
BA.minimize(ax_out);

% initialization of decision variables
BA.set_initial(delta,0);
BA.set_initial(beta,0);
BA.set_initial(yaw_rate,0);
BA.set_initial(brake_pressure,0);
BA.set_initial(wheel_rot_fl,velocity/carData.Chassis.radWheel);
BA.set_initial(wheel_rot_fr,velocity/carData.Chassis.radWheel);
BA.set_initial(wheel_rot_rl,velocity/carData.Chassis.radWheel);
BA.set_initial(wheel_rot_rr,velocity/carData.Chassis.radWheel);

% Constraints
BA.subject_to(ay_out == ay_control);
BA.subject_to(ax_out == ax_control);
BA.subject_to(ay_out == 0);

% solve
plugin_opts = struct('print_time',0);
solver_opts = struct('constr_viol_tol',0.1,'acceptable_obj_change_tol',0.001,'print_level',0);
BA.solver('ipopt',plugin_opts,solver_opts);
sol = BA.solve();

% extract results
GGV.ay(end) = sol.value(ay_out);
GGV.ax(end) = sol.value(ax_out);

end