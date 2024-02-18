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
deltaScaled = BA.variable();                BA.subject_to(-1<=deltaScaled<=1);                  % steering angle (rad)
betaScaled = BA.variable();                 BA.subject_to(-1<=betaScaled<=1);                   % sideslip angle (rad)
yaw_rateScaled = BA.variable();             BA.subject_to(-1<=yaw_rateScaled<=1);               % yaw rate (rad/s)
% throttle_positionScaled = BA.variable();    BA.subject_to(0<=throttle_positionScaled<=1);     % throttle position (-)
brake_pressureScaled = BA.variable();       BA.subject_to(0<=brake_pressureScaled<=1);            % brake pressure (bar)
wheel_rot_flScaled = BA.variable();         BA.subject_to(0<=wheel_rot_flScaled<=1)             % FL wheel angular velocity (rad/s)
wheel_rot_frScaled = BA.variable();         BA.subject_to(0<=wheel_rot_frScaled<=1)             % FR wheel angular velocity (rad/s)
wheel_rot_rlScaled = BA.variable();         BA.subject_to(0<=wheel_rot_rlScaled<=1)             % RL wheel angular velocity (rad/s)
wheel_rot_rrScaled = BA.variable();         BA.subject_to(0<=wheel_rot_rrScaled<=1)             % RR wheel angular velocity (rad/s)

% Scaling for decision variables
delta = deltaScaled * 30*pi/180;
beta = betaScaled * 5*pi/180;
yaw_rate = yaw_rateScaled * 120*pi/180;
% throttle_position = throttle_positionScaled;
brake_pressure = brake_pressureScaled * 100;
wheel_rot_fl = wheel_rot_flScaled * carData.Powertrain.vMax/carData.Chassis.radWheel;
wheel_rot_fr = wheel_rot_frScaled * carData.Powertrain.vMax/carData.Chassis.radWheel;
wheel_rot_rl = wheel_rot_rlScaled * carData.Powertrain.vMax/carData.Chassis.radWheel;
wheel_rot_rr = wheel_rot_rrScaled * carData.Powertrain.vMax/carData.Chassis.radWheel;

% Initial Value for decision variables
initWheelVel = (velocity/carData.Chassis.radWheel)/(carData.Powertrain.vMax/carData.Chassis.radWheel);

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
Front_Brake_Force = 2*(brake_pressure * carData.Brakes.rBrakeBias .*10^5 .*carData.Brakes.areaPiston .*carData.Brakes.nPistonFront) * carData.Brakes.muBrakePad * carData.Brakes.radBrakeDisc / carData.Chassis.radWheel; %N
Rear_Brake_Force = 2*(brake_pressure * (1-carData.Brakes.rBrakeBias) .*10^5 .*carData.Brakes.areaPiston .*carData.Brakes.nPistonRear) * carData.Brakes.muBrakePad * carData.Brakes.radBrakeDisc / carData.Chassis.radWheel; %N
F_brake = -(Front_Brake_Force + Rear_Brake_Force);

ax_control = (F_brake - Fd) / carData.Chassis.mass;

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

brakeBias_tyre = (fx_fl + fx_fr) / (fx_fl + fx_fr + fx_rl + fx_rr);

% Car States
ay_out = (fy_fl + fy_fr + fy_rl + fy_rr)/carData.Chassis.mass;
ax_out = (fx_fl + fx_fr + fx_rl + fx_rr)/carData.Chassis.mass;
Mz_out = (carData.Chassis.frontMomentArm*(fy_fl + fy_fr) + 0.5*carData.Chassis.trackWidth*(fx_fl + fx_rl) ...
          - carData.Chassis.rearMomentArm*(fy_rl + fy_rr) - 0.5*carData.Chassis.trackWidth*(fx_fr + fx_rr))/carData.Chassis.yawInertia;

% Residuals
ax_res = ax_control - ax_out;
ay_res = ay_control - ay_out;
brakeBias_res = carData.Brakes.rBrakeBias - brakeBias_tyre;

% Path Constraints

BA.subject_to(-12*pi/180<=alpha_fl<=12*pi/180);
BA.subject_to(-12*pi/180<=alpha_fr<=12*pi/180);
BA.subject_to(-12*pi/180<=alpha_rl<=12*pi/180);
BA.subject_to(-12*pi/180<=alpha_rr<=12*pi/180);

BA.subject_to(-0.10<=kappa_fl<=0);
BA.subject_to(-0.10<=kappa_fr<=0);
BA.subject_to(-0.10<=kappa_rl<=0);
BA.subject_to(-0.10<=kappa_rr<=0);

% objective
BA.minimize(ax_out);

% initialization of decision variables
BA.set_initial(deltaScaled,0);
BA.set_initial(betaScaled,0);
BA.set_initial(yaw_rateScaled,0);
BA.set_initial(brake_pressureScaled,0);
BA.set_initial(wheel_rot_flScaled,initWheelVel);
BA.set_initial(wheel_rot_frScaled,initWheelVel);
BA.set_initial(wheel_rot_rlScaled,initWheelVel);
BA.set_initial(wheel_rot_rrScaled,initWheelVel);

% Constraints
BA.subject_to(-0.05<=ay_res<=0.05);
BA.subject_to(-0.05<=ax_res<=0.05);
BA.subject_to(-0.025<=brakeBias_res<=0.025);
BA.subject_to(ay_out == 0);

% solve
plugin_opts = struct('print_time',0);
solver_opts = struct('print_level',0); % 'constr_viol_tol',0.1,'acceptable_obj_change_tol',0.001,
BA.solver('ipopt',plugin_opts,solver_opts);
sol = BA.solve();

%     extract results
GGV.ay(end)           = sol.value(ay_out);
GGV.ax(end)           = sol.value(ax_out);
GGV.delta(end)        = sol.value(delta);
GGV.beta(end)         = sol.value(beta);
GGV.yaw_rate(end)     = sol.value(yaw_rate);
GGV.wheel_rot_fl(end) = sol.value(wheel_rot_fl);
GGV.wheel_rot_fr(end) = sol.value(wheel_rot_fr);
GGV.wheel_rot_rl(end) = sol.value(wheel_rot_rl);
GGV.wheel_rot_rr(end) = sol.value(wheel_rot_rr);

end