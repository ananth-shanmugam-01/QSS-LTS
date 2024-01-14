%%%%%%%%%%%%%%%%%%%%%%%
% CasADi Problem Formulation of Vehicle Model
% Reference - Mario Boxheimer 


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

yaw_rate = V_current*Kt;

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

% Braking Decelerative Force
% Front_Brake_Force = 2*(brake_pressure * 100 * carData.Brakes.rBrakeBias .*10^5 .*carData.Brakes.areaPiston .*carData.Brakes.nPistonFront) * carData.Brakes.muBrakePad * carData.Brakes.radBrakeDisc / carData.Chassis.radWheel; %N
% Rear_Brake_Force = 2*(brake_pressure * 100 *(1-carData.Brakes.rBrakeBias) .*10^5 .*carData.Brakes.areaPiston .*carData.Brakes.nPistonFront) * carData.Brakes.muBrakePad * carData.Brakes.radBrakeDisc / carData.Chassis.radWheel; %N
% F_brake = -(Front_Brake_Force + Rear_Brake_Force);

ax_control = (F_tractive - Fd) / carData.Chassis.mass;

% Slip Angles
alpha_fl = ((V_current*tan(beta) + carData.Chassis.frontMomentArm*yaw_rate) / (V_current + yaw_rate*carData.Chassis.trackWidth*0.5)) - delta;
alpha_fr = ((V_current*tan(beta) + carData.Chassis.frontMomentArm*yaw_rate) / (V_current - yaw_rate*carData.Chassis.trackWidth*0.5)) - delta;
alpha_rl = (V_current*tan(beta) - carData.Chassis.rearMomentArm*yaw_rate) / (V_current + yaw_rate*carData.Chassis.trackWidth*0.5);
alpha_rr = (V_current*tan(beta) - carData.Chassis.rearMomentArm*yaw_rate) / (V_current - yaw_rate*carData.Chassis.trackWidth*0.5);

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
[fy_fl, fx_fl] = MF52_Combined(kappa_fl,alpha_fl,w_fl,0);
[fy_fr, fx_fr] = MF52_Combined(kappa_fr,alpha_fr,w_fr,0);
[fy_rl, fx_rl] = MF52_Combined(kappa_rl,alpha_rl,w_rl,0);
[fy_rr, fx_rr] = MF52_Combined(kappa_rr,alpha_rr,w_rr,0);

% Car States
ay_out = (fy_fl + fy_fr + fy_rl + fy_rr)/carData.Chassis.mass;
ax_out = (fx_rl + fx_rr)/carData.Chassis.mass;
Mz_out = (carData.Chassis.frontMomentArm*(fy_fl + fy_fr) + 0.5*carData.Chassis.trackWidth*(fx_fl + fx_rl) ...
          - carData.Chassis.rearMomentArm*(fy_rl + fy_rr) - 0.5*carData.Chassis.trackWidth*(fx_fr + fx_rr))/carData.Chassis.yawInertia;

% Target Speed
V_out = sqrt(V_current^2 + 2*ax_out*sector_length);