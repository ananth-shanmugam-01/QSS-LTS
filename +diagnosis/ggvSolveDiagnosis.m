%% Failed Solve TroubleShooting
i = 322;
curvature = trackCurvature(i);
carData = preProccess.initVehicleModel();


% initialize Problem
import casadi.*

nlpBSP = casadi.Opti();

% decision variables & box constraints
VxScaled = nlpBSP.variable(); nlpBSP.subject_to(0.1<=VxScaled<=1);
deltaScaled = nlpBSP.variable();                nlpBSP.subject_to(-1<=deltaScaled<=1);             % steering angle (rad)
betaScaled = nlpBSP.variable();                 nlpBSP.subject_to(-1<=betaScaled<=1);                 % sideslip angle (rad)
% throttle_positionScaled = nlpBSP.variable();    nlpBSP.subject_to(0<=throttle_positionScaled<=1);     % throttle position (-)
% brake_pressureScaled = nlpBSP.variable();           nlpBSP.subject_to(0<=brake_pressureScaled<=1);              % brake pressure (bar)
wheel_rot_flScaled = nlpBSP.variable();         nlpBSP.subject_to(0<=wheel_rot_flScaled<=1)           % FL wheel angular velocity (rad/s)
wheel_rot_frScaled = nlpBSP.variable();         nlpBSP.subject_to(0<=wheel_rot_frScaled<=1)           % FR wheel angular velocity (rad/s)
wheel_rot_rlScaled = nlpBSP.variable();         nlpBSP.subject_to(0<=wheel_rot_rlScaled<=1)           % RL wheel angular velocity (rad/s)
wheel_rot_rrScaled = nlpBSP.variable();         nlpBSP.subject_to(0<=wheel_rot_rrScaled<=1)           % RR wheel angular velocity (rad/s)

% State Weights
VxWeight = carData.Powertrain.vMax;
deltaWeight = 30*pi/180;
betaWeight = 5*pi/180;
throttle_position_weight = 1;
brake_pressure_weight = 100;
wheel_rot_weight = carData.Powertrain.vMax/carData.Chassis.radWheel;

% Scaling for decision variables
Vx = VxScaled * VxWeight;
delta = deltaScaled * deltaWeight;
beta = betaScaled * betaWeight;
% throttle_position = throttle_positionScaled  * throttle_position_weight;
% brake_pressure = brake_pressureScaled * brake_pressure_weight;
wheel_rot_fl = wheel_rot_flScaled * wheel_rot_weight;
wheel_rot_fr = wheel_rot_frScaled * wheel_rot_weight;
wheel_rot_rl = wheel_rot_rlScaled * wheel_rot_weight;
wheel_rot_rr = wheel_rot_rrScaled * wheel_rot_weight;


% Equations of Motion

g = 9.81;
DF_total = 0.5*1.225*carData.Aero.CLA*Vx^2;
DF_front = carData.Aero.rAeroBalance*DF_total;
DF_rear = (1-carData.Aero.rAeroBalance)*DF_front;
Fd = 0.5*1.225*carData.Aero.CDA*Vx^2;

ay_control = curvature * Vx^2;
yaw_rate = curvature * Vx;

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

nlpBSP.subject_to(-0.01<=kappa_fl<=0.01);
nlpBSP.subject_to(-0.01<=kappa_fr<=0.01);
nlpBSP.subject_to(-0.01<=kappa_rl<=0.01);
nlpBSP.subject_to(-0.01<=kappa_rr<=0.01);

% objective
nlpBSP.minimize(-Vx);

% initialization of decision variables
nlpBSP.set_initial(VxScaled,LSP.Vx(i-1)/VxWeight);
nlpBSP.set_initial(deltaScaled,LSP.delta(i-1)/deltaWeight);
nlpBSP.set_initial(betaScaled,LSP.beta(i-1)/betaWeight);
nlpBSP.set_initial(wheel_rot_flScaled,LSP.wheel_rot_fl(i-1)/wheel_rot_weight);
nlpBSP.set_initial(wheel_rot_frScaled,LSP.wheel_rot_fr(i-1)/wheel_rot_weight);
nlpBSP.set_initial(wheel_rot_rlScaled,LSP.wheel_rot_rl(i-1)/wheel_rot_weight);
nlpBSP.set_initial(wheel_rot_rrScaled,LSP.wheel_rot_rr(i-1)/wheel_rot_weight);

% Constraints
nlpBSP.subject_to(-0.05<=ay_res<=0.05);
% nlpBSP.subject_to(-0.05<=ax_res<=0.05);
nlpBSP.subject_to(-5 <= Mz_out <= 5);

% solve
plugin_opts = struct('print_time',0);
solver_opts = struct('print_level',0); %'constr_viol_tol',0.1,'acceptable_obj_change_tol',0.001, 
nlpBSP.solver('ipopt',plugin_opts,solver_opts);  
sol = nlpBSP.solve();
% Convergence Constraints
rad2deg(nlpBSP.debug.value(deltaScaled)* 30*pi/180)
rad2deg(nlpBSP.debug.value(betaScaled) *5*pi/180)
nlpBSP.debug.value(wheel_rot_flScaled)
nlpBSP.debug.value(wheel_rot_frScaled)
nlpBSP.debug.value(wheel_rot_rlScaled)
nlpBSP.debug.value(wheel_rot_rrScaled)

% Convergence Constraints
nlpBSP.debug.value(ay_res)
nlpBSP.debug.value(ax_res)
nlpBSP.debug.value(Mz_out)
nlpBSP.debug.value(brakeBias_res)

% States
nlpBSP.debug.value(ax_control)
nlpBSP.debug.value(ax_out)
nlpBSP.debug.value(ay_out)
nlpBSP.debug.value(ay_control)
nlpBSP.debug.value(brakeBias_res)

% Internal States
rad2deg(nlpBSP.debug.value(alpha_fl))
rad2deg(nlpBSP.debug.value(alpha_fr))
rad2deg(nlpBSP.debug.value(alpha_rl))
rad2deg(nlpBSP.debug.value(alpha_rr))

nlpBSP.debug.value(kappa_fl)
nlpBSP.debug.value(kappa_fr)
nlpBSP.debug.value(kappa_rl)
nlpBSP.debug.value(kappa_rr)



