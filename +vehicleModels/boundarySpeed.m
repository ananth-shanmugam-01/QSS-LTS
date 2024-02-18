%%%%%%%%%%%%%%%%%%%%%%%
% CasADi Problem Formulation of Vehicle Model
% Reference - Mario Boxheimer 

function LSP = boundarySpeed(i, LSP, curvature, carData)

% initialize Problem
import casadi.*

nlpBSP = casadi.Opti();

% Input Values
Kt = curvature;

deltaMin = -27;
deltaMax = 27;
betaMin = -3;
betaMax = 3;

% decision variables & box constraints
Vx = nlpBSP.variable(); nlpBSP.subject_to(0<=Vx<=carData.Powertrain.vMax);
delta = nlpBSP.variable(); nlpBSP.subject_to(deltaMin*pi/180<=delta<=deltaMax*pi/180);                                           % steering angle (rad)
beta = nlpBSP.variable(); nlpBSP.subject_to(betaMin*pi/180<=beta<=betaMax*pi/180);                                               % sideslip angle (rad)
% yaw_rate = nlpBSP.variable(); nlpBSP.subject_to(-120*pi/180<=yaw_rate<=120*pi/180);                                                % yaw rate (rad/s)
% throttle_position = nlp.variable(); nlp.subject_to(0<=throttle_position<=1);                                               % throttle position (-)
% brake_pressure = nlp.variable(); nlp.subject_to(0<=brake_pressure<=1);                                                     % brake pressure (bar)
wheel_rot_fl = nlpBSP.variable(); nlpBSP.subject_to(0<=wheel_rot_fl<=carData.Powertrain.vMax/carData.Chassis.radWheel)           % FL wheel angular velocity (rad/s)
wheel_rot_fr = nlpBSP.variable(); nlpBSP.subject_to(0<=wheel_rot_fr<=carData.Powertrain.vMax/carData.Chassis.radWheel)           % FR wheel angular velocity (rad/s)
wheel_rot_rl = nlpBSP.variable(); nlpBSP.subject_to(0<=wheel_rot_rl<=carData.Powertrain.vMax/carData.Chassis.radWheel)           % RL wheel angular velocity (rad/s)
wheel_rot_rr = nlpBSP.variable(); nlpBSP.subject_to(0<=wheel_rot_rr<=carData.Powertrain.vMax/carData.Chassis.radWheel)           % RR wheel angular velocity (rad/s)

%% Equations of Motion

g = 9.81;
DF_total = 0.5*1.225*carData.Aero.CLA*Vx^2;
DF_front = carData.Aero.rAeroBalance*DF_total;
DF_rear = (1-carData.Aero.rAeroBalance)*DF_front;
Fd = 0.5*1.225*carData.Aero.CDA*Vx^2;

ay_control = Kt * Vx^2;
yaw_rate = Kt * Vx;

% Motor Tractive Force
% torqueInterp = interpolant('LUT','bspline',{[carData.Powertrain.RPM]},carData.Powertrain.torqueMotor); % CasADi feature
% wheel_avg_vel = 0.5*(wheel_rot_rl + wheel_rot_rr);
% motor_rot_vel = wheel_avg_vel * carData.Powertrain.rGear *60/(2*pi);
% F_tractive = throttle_position * carData.Powertrain.effPU * carData.Powertrain.nDrive * torqueInterp(motor_rot_vel) * carData.Powertrain.rGear/carData.Chassis.radWheel;
% 
% Braking Decelerative Force
% Front_Brake_Force = 2*(brake_pressure * 100 * carData.Brakes.rBrakeBias .*10^5 .*carData.Brakes.areaPiston .*carData.Brakes.nPistonFront) * carData.Brakes.muBrakePad * carData.Brakes.radBrakeDisc / carData.Chassis.radWheel; %N
% Rear_Brake_Force = 2*(brake_pressure * 100 *(1-carData.Brakes.rBrakeBias) .*10^5 .*carData.Brakes.areaPiston .*carData.Brakes.nPistonFront) * carData.Brakes.muBrakePad * carData.Brakes.radBrakeDisc / carData.Chassis.radWheel; %N
% F_brake = -(Front_Brake_Force + Rear_Brake_Force);
% 
% ax_control = (F_tractive + F_brake) / carData.Chassis.mass;

ax_control = 0;

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
ax_out = (fx_fl + fx_fr + fx_rl + fx_rr - Fd)/carData.Chassis.mass;
Mz_out = (carData.Chassis.frontMomentArm*(fy_fl + fy_fr) + 0.5*carData.Chassis.trackWidth*(fx_fl + fx_rl) ...
          - carData.Chassis.rearMomentArm*(fy_rl + fy_rr) - 0.5*carData.Chassis.trackWidth*(fx_fr + fx_rr))/carData.Chassis.yawInertia;

 % Residuals
ax_res = ax_control - ax_out;
ay_res = ay_control - ay_out;
brakeBias_res = carData.Brakes.rBrakeBias - brakeBias_tyre;

% Path Constraints

nlpBSP.subject_to(-12*pi/180<=alpha_fl<=12*pi/180);
nlpBSP.subject_to(-12*pi/180<=alpha_fr<=12*pi/180);
nlpBSP.subject_to(-12*pi/180<=alpha_rl<=12*pi/180);
nlpBSP.subject_to(-12*pi/180<=alpha_rr<=12*pi/180);

nlpBSP.subject_to(-0.1<=kappa_fl<=0.1);
nlpBSP.subject_to(-0.1<=kappa_fr<=0.1);
nlpBSP.subject_to(-0.1<=kappa_rl<=0.1);
nlpBSP.subject_to(-0.1<=kappa_rr<=0.1);

% objective
nlpBSP.minimize(-Vx);

% initialization of decision variables
nlpBSP.set_initial(Vx,LSP.Vx(i-1));
nlpBSP.set_initial(delta,LSP.delta(i-1));
nlpBSP.set_initial(beta,LSP.beta(i-1));
% nlpBSP.set_initial(yaw_rate,LSP.yaw_rate(i-1));
nlpBSP.set_initial(wheel_rot_fl,LSP.wheel_rot_fl(i-1));
nlpBSP.set_initial(wheel_rot_fr,LSP.wheel_rot_fr(i-1));
nlpBSP.set_initial(wheel_rot_rl,LSP.wheel_rot_rl(i-1));
nlpBSP.set_initial(wheel_rot_rr,LSP.wheel_rot_rr(i-1));

% Constraints
nlpBSP.subject_to(-0.05<=ay_res<=0.05);
nlpBSP.subject_to(-0.05<=ax_res<=0.05);
% nlpBSP.subject_to(yaw_rate == curvature*Vx);
nlpBSP.subject_to(-5 <= Mz_out <= 5);

% solve
plugin_opts = struct('print_time',0);
solver_opts = struct('print_level',0); %'constr_viol_tol',0.1,'acceptable_obj_change_tol',0.001, 
nlpBSP.solver('ipopt',plugin_opts,solver_opts);  
sol = nlpBSP.solve();

% extract results
LSP.Vx(i)                  = sol.value(Vx);                % Velocity (m/s)
LSP.Ay(i)                  = sol.value(ay_out);            % Lateral Acceleration (m/s^2)
LSP.Ax(i)                  = sol.value(ax_out);            % Lateral Acceleration (m/s^2)
LSP.Ax_control(i)          = sol.value(ax_control);
LSP.delta(i)               = sol.value(delta);             % steering angle (rad)
LSP.beta(i)                = sol.value(beta);              % sideslip angle (rad)
LSP.yaw_rate(i)            = sol.value(yaw_rate);          % yaw rate (rad/s)
LSP.wheel_rot_fl(i)        = sol.value(wheel_rot_fl);      % FL wheel angular velocity (rad/s)
LSP.wheel_rot_fr(i)        = sol.value(wheel_rot_fr);      % FR wheel angular velocity (rad/s)
LSP.wheel_rot_rl(i)        = sol.value(wheel_rot_rl);      % RL wheel angular velocity (rad/s)
LSP.wheel_rot_rr(i)        = sol.value(wheel_rot_rr);      % RR wheel angular velocity (rad/s)
LSP.F_drag(i)              = sol.value(Fd);


end