%% Quasi-Steady State Lap Time Simulation

% 200823
% Fundamentally poor

clear;
clc;
close all;
addpath("Profile Calculations\")
addpath("Data Files\")

%% Upper Heyford
tic;
global sector_dist
sector_dist = 0.5;
[ay_meas, dist, vel_ms, time, track_dist, curvature_spline, curvature] = logged_data('track_data.txt',sector_dist);

%% Converting Vehicle Properties to Struct 
% Avoiding the use of global parameters

carData = struct;
constants = struct; 
g = 9.81;

% Chassis Properties
carData.Chassis.mass = 274;
carData.Chassis.unsprungMass = 14.7;
carData.Chassis.SprungMass = carData.Chassis.mass - 4*(carData.Chassis.unsprungMass);
carData.Chassis.heightUnsprungCOG = 0.196;
carData.Chassis.heightSprungCOG = 0.280;
carData.Chassis.weightDist = 0.45;
carData.Chassis.radWheel = 0.196; % Loaded radius
carData.Chassis.wheelBase = 1.535;
carData.Chassis.trackWidth = 1.23;
carData.Chassis.massFront = carData.Chassis.mass*carData.Chassis.weightDist;
carData.Chassis.massRear = carData.Chassis.mass*(1-carData.Chassis.weightDist);
carData.Chassis.sprungMassFront = carData.Chassis.SprungMass*carData.Chassis.weightDist;
carData.Chassis.sprungMassRear = carData.Chassis.SprungMass*(1-carData.Chassis.weightDist);
carData.Chassis.frontMomentArm = carData.Chassis.wheelBase*(1 - carData.Chassis.weightDist);
carData.Chassis.rearMomentArm = carData.Chassis.wheelBase*(carData.Chassis.weightDist);
carData.Chassis.yawInertia = 120; % kgm^2

% Aerodynamic Properties
carData.Aero.CLA = -0.06;
carData.Aero.CDA = 0.6;
carData.Aero.rAeroBalance = 0.55;

% Suspension Properties
carData.Suspension.heightCG2rollAxis = 0.230;
carData.Suspension.rollCentreFront = 0.052;
carData.Suspension.rollCentreRear = 0.051;
carData.Suspension.mechanicalBalance = 0.4;

% Brake Properties
carData.Brakes.muBrakePad = 0.4;
carData.Brakes.nPistonFront = 4;
carData.Brakes.nPistonRear = 2;
carData.Brakes.diamPiston = 0.0254;
carData.Brakes.areaPiston = (0.25*pi*carData.Brakes.diamPiston^2);
carData.Brakes.radBrakeDisc = 0.1836/2;
carData.Brakes.rBrakeBias = 0.5;

% Powertrain Model
[carData.Powertrain.RPM, carData.Powertrain.torqueMotor] = powerCurveInterpolation('Data Files\AMK_21Nm.txt');
carData.Powertrain.rGear = 15.55;
carData.Powertrain.effPU = 0.8;
carData.Powertrain.nDrive = 2; % number of drive wheels, switch case later?

% Vmax Calculations, FS style
carData.Powertrain.vMax = (carData.Powertrain.RPM(end)/carData.Powertrain.rGear)*0.10472*carData.Chassis.radWheel; % Converted to m/s

save('carData.mat','carData')

%% Generate Boundary Speed Profile
clc;

vel(1) = 29;
del(1) = 0;
beta(1) = 0;
ay(1) = 0;

cornering_flag = zeros(length(track_dist),1);

for i = 2:length(curvature_spline)

test_curvature = curvature_spline(i);

if test_curvature > 0 % Right Hand Corner
    min_steering = 0;
    max_steering = 27;
    beta_min = -1;
    beta_max = 0;
else
    min_steering = -27;
    max_steering = 0;
    beta_min = 0;
    beta_max = 1;
end 

x0 = [vel(i-1), del(i-1), beta(i-1)];

Aeq = [];
beq = [];
lb = [0, min_steering, beta_min];
ub = [carData.Powertrain.vMax, max_steering, beta_max];

options = optimoptions("fmincon",'MaxFunEvals',5000,'MaxIter',5000,'Display','off','Algorithm','sqp','ConstraintTolerance',0.05); %,'EnableFeasibilityMode',true,'ConstraintTolerance',0.05,'FiniteDifferenceType','central');
[control_inputs, ~, exitflag] = fmincon(@(x) max_cornering(x,test_curvature,carData),x0,[],[],Aeq,beq,lb,ub,@(x)nonlcon_cornering(x,test_curvature,carData),options);

cornering_flag(i) = exitflag;

vel(i) = control_inputs(1);
del(i) = control_inputs(2);
beta(i) = control_inputs(3);
ay(i) = test_curvature*vel(i)^2;

disp([ num2str(i), '/',num2str(length(track_dist))])
end

%% Identify Apices
vel(vel>29) = carData.Powertrain.vMax; 
[val, locs] = findpeaks(-vel,"MinPeakDistance",6);

%% Forward Acceleration

clear forward_vel
clc;

%     Start at first apex
%     Input Array
%     del = control_variables(1);
%     beta = control_variables(2);
%     throttle_position = control_variables(3);
%     wheel_rot_fl = control_variables(4);
%     wheel_rot_fr = control_variables(5);
%     wheel_rot_rl = control_variables(6);
%     wheel_rot_rr = control_variables(7);

% Start Acceleration from the slowest apex, this way the speed traces will
% align

forward_vel(locs(val == max(val))) = vel(locs(val == max(val)));

accel_flag = zeros(length(track_dist),1);

for i = locs(val == max(val)):length(track_dist)-1

    test_curvature = curvature_spline(i);

    % Control Inputs
    min_steering = -27;
    max_steering = 27;
    
    if test_curvature > 0 % Right Hand Corner
    beta_min = -1;
    beta_max = 0;
    else
    beta_min = 0;
    beta_max = 1;
    end 

    x0 = [del(i), beta(i),0,vel(i)/carData.Chassis.radWheel,vel(i)/carData.Chassis.radWheel,vel(i)/carData.Chassis.radWheel,vel(i)/carData.Chassis.radWheel];

    Aeq = [];
    beq = [];
    lb = [min_steering, beta_min,0.01,0,0,0,0];
    ub = [max_steering, beta_max,1,carData.Powertrain.vMax/carData.Chassis.radWheel,carData.Powertrain.vMax/carData.Chassis.radWheel,carData.Powertrain.vMax/carData.Chassis.radWheel,carData.Powertrain.vMax/carData.Chassis.radWheel];
    [control_inputs, fnval, exitflag] = fmincon(@(x) max_acceleration(x,test_curvature,forward_vel(i),carData),x0,[],[],Aeq,beq,lb,ub,@(x) nonlcon_acceleration(x,test_curvature,forward_vel(i),vel(i+1),carData),options); 

    accel_flag(i) = exitflag;
    
    if -fnval > vel(i+1)
        fnval = -vel(i+1);
    end

    forward_vel(i+1) = -fnval;
    del_accel(i) = control_inputs(1);
    beta_accel(i) = control_inputs(2);
    throttle_position(i) = control_inputs(3);
    ax_acc(i) = (forward_vel(i+1)^2 - forward_vel(i)^2)/(2*sector_dist);
    ay_acc(i) = test_curvature*forward_vel(i)^2;
    disp([ num2str(i), '/',num2str(length(track_dist))])
    
end

forward_vel(1) = forward_vel(end); % Connect end of lap and start of lap

for i = 1: locs(val == max(val))

    test_curvature = curvature_spline(i);

    min_steering = -27;
    max_steering = 27;
    
    if test_curvature > 0 % Right Hand Corner
    beta_min = -1;
    beta_max = 0;
    else
    beta_min = 0;
    beta_max = 1;
    end 

    x0 = [del(i), beta(i),0,vel(i)/carData.Chassis.radWheel,vel(i)/carData.Chassis.radWheel,vel(i)/carData.Chassis.radWheel,vel(i)/carData.Chassis.radWheel];

    Aeq = [];
    beq = [];
    lb = [min_steering, beta_min,0.01,0,0,0,0];
    ub = [max_steering, beta_max,1,carData.Powertrain.vMax/carData.Chassis.radWheel,carData.Powertrain.vMax/carData.Chassis.radWheel,carData.Powertrain.vMax/carData.Chassis.radWheel,carData.Powertrain.vMax/carData.Chassis.radWheel];
    [control_inputs, fnval, exitflag] = fmincon(@(x) max_acceleration(x,test_curvature,forward_vel(i),carData),x0,[],[],Aeq,beq,lb,ub,@(x) nonlcon_acceleration(x,test_curvature,forward_vel(i),vel(i+1),carData),options); 
    accel_flag(i) = exitflag;
    
    if -fnval > vel(i+1)
    fnval = -vel(i+1);
    end

    forward_vel(i+1) = -fnval;
    del_accel(i) = control_inputs(1);
    beta_accel(i) = control_inputs(2);
    throttle_position(i) = control_inputs(3);
    ax_acc(i) = (forward_vel(i+1)^2 - forward_vel(i)^2)/(2*sector_dist);
    ay_acc(i) = test_curvature*forward_vel(i)^2;
    disp([ num2str(i), '/',num2str(length(track_dist))])

end 

del_accel(length(track_dist)) = del_accel(1);
beta_accel(length(track_dist)) = beta_accel(1);
throttle_position(length(track_dist)) = throttle_position(1);
ax_acc(length(track_dist)) = ax_acc(length(track_dist)-1);
ax_acc(1) = ax_acc(length(track_dist));
ay_acc(length(track_dist)) = ay_acc(1);

%% Braking Speed Profile
% Corrected Calculations
% Start from one point left of slowest apex and move backwards
% Solver inputs:
% previous as guess control inputs
% Current curvature
% 'Previous' velocity i.e. true next velocity

clc
clear br_vel;
tic


% Determine Velocity at first node
br_vel = zeros(length(track_dist),1);
braking_flag = zeros(length(track_dist),1);

del_brake(locs(end)) = del(locs(end));
beta_brake(locs(end)) = beta(locs(end));
brakePressure(locs(end)) = 0;
br_vel(locs(end)) = vel(locs(end));

brakePressureOut = zeros(length(track_dist),1);

for i = locs(end)-1:-1:1

    test_curvature = curvature_spline(i);

    % Control Inputs
    min_steering = -27;
    max_steering = 27;
    
    if test_curvature > 0 % Right Hand Corner
    beta_min = -1;
    beta_max = 0;
    else
    beta_min = 0;
    beta_max = 1;
    end 

    % Include new term of velocity
    x0 = [del_brake(i+1), beta_brake(i+1),brakePressure(i+1),br_vel(i+1)/carData.Chassis.radWheel,br_vel(i+1)/carData.Chassis.radWheel,br_vel(i+1)/carData.Chassis.radWheel,br_vel(i+1)/carData.Chassis.radWheel,br_vel(i+1)];
    Aeq = [];
    beq = [];
    lb = [min_steering, beta_min,0,0,0,0,0,0];
    ub = [max_steering, beta_max,0.6,carData.Powertrain.vMax/carData.Chassis.radWheel,carData.Powertrain.vMax/carData.Chassis.radWheel,carData.Powertrain.vMax/carData.Chassis.radWheel,carData.Powertrain.vMax/carData.Chassis.radWheel, carData.Powertrain.vMax]; % Scaled Brake Pressure Sensor
    [control_inputs, fnval, exitflag] = fmincon(@(x) max_deceleration(x,test_curvature,br_vel(i+1),carData),x0,[],[],Aeq,beq,lb,ub,@(x)nonlcon_deceleration(x,test_curvature,br_vel(i+1),vel(i),carData),options);
    braking_flag(i) = exitflag;
    br_vel(i) = -fnval;
    if br_vel(i)>vel(i)
        br_vel(i)=vel(i);
    end
    brakePressureOut(i) = control_inputs(3).*100;
    disp(['Braking Profile - ' ,num2str(i), '/',num2str(length(track_dist))])
end

for i = locs(end):length(track_dist)
    br_vel(i) = vel(i); % Prevent these inputs for affecting array
    braking_flag(i) = 0;
end 

%% Final Profile Calculations
clc;
toc

final_vel = min([vel; forward_vel; br_vel']);
lap_time = cumsum(sector_dist./final_vel);
disp(['Simulated Lap Time - ', num2str(lap_time(end))])
final_ax = zeros(length(track_dist),1);

for i = 1:length(track_dist)-1
    final_ax(i) = (final_vel(i+1)^2 - final_vel(i)^2)/(2*sector_dist);
end

final_ax(length(final_vel)) = (final_vel(1)^2 - final_vel(length(final_vel))^2)/(2*sector_dist);
final_ay = curvature_spline.*final_vel.^2;
%% Control Inputs
brakePressureTest = brakePressureOut.*(br_vel' == final_vel)';
throttlePositionTest = throttle_position'.*(forward_vel == final_vel)';

%% Save Results for QTS

acc = table(track_dist',lap_time',final_ax,final_ay',final_vel',curvature_spline');
acc.Properties.VariableNames = {'dist','time','final_ax','final_ay','final_vel','curvature'};
save('acc.mat','acc')

%% QSS Methodology
close all
figure
hold on
plot(track_dist,vel,'b--','LineWidth',2,'DisplayName','LSP')
plot(track_dist,forward_vel,'LineWidth',2,'DisplayName','Forward')
plot(track_dist,br_vel,'LineWidth',2,'DisplayName','Brake')
plot(track_dist,final_vel,'k','LineWidth',2,'DisplayName','Final Velocity Profile')
hold off
xlim([0 track_dist(end)])
legend
ylabel('Velocity [m/s]')
xlabel('Distance [m]')
title('QSS Simulation')
grid on
box on
subtitle([' Simulated Lap Time - ', num2str(lap_time(end)), '(s); ',' Measured Lap Time - ', num2str(time(end)), '(s)'])
fontsize(gca,13,'points')


%% Measured Data pre-processing
load('Lap4.mat')
filt_ay = lowpass(Lap4.Acc_Y_Axis_g_,0.3);
filt_ax = lowpass(Lap4.Acc_X_Axis_g_,0.5);

vel_interp = interp1(Lap4.dist,Lap4.gps_speed_km_h_./3.6,track_dist,"spline")'; % Interpolated Measured Velocity
ax_interp = interp1(Lap4.dist,filt_ax,track_dist,"spline")'; % Interpolated Measured Ax
ay_interp = interp1(Lap4.dist,filt_ay,track_dist,"spline")'; % Interpolated Measured Ay
meas_laptime = Lap4.time(end);

% QSS Modelling Quality
qss_vel_r_sqr = r_square_calc(vel_interp,final_vel');
qss_ax_r_sqr = r_square_calc(ax_interp,(final_ax./g));
qss_ay_r_sqr = r_square_calc(ay_interp,(final_ay./g)');
qss_del_laptime = 1 - abs(lap_time(end) - meas_laptime)/(meas_laptime);
qss_quality = qss_vel_r_sqr*qss_ax_r_sqr*qss_ay_r_sqr*qss_del_laptime;


%% Measured vs QSS Velocity Comparison
% Velocity Profile
figure
hold on
plot(track_dist,final_vel.*3.6,'k','LineWidth',2,'DisplayName','QSS')
plot(dist,vel_ms.*3.6,'r-','LineWidth',2,'DisplayName','Measured')
hold off
xlim([0 track_dist(end)])
legend
ylabel('Velocity [km/h]')
xlabel('Distance [m]')
title(['QSS vs Measured; R^{2} - ', num2str(qss_vel_r_sqr)])
grid on
box on
subtitle([' Simulated Lap Time - ', num2str(lap_time(end)), '(s); ',' Measured Lap Time - ', num2str(time(end)), '(s)'])


%% Measured vs QSS Accelerations Comparison

% G-G Diagram
figure
hold on
plot(final_ay./g,final_ax./g,'ko','DisplayName','QSS Simulation')
plot(filt_ay,filt_ax,'ro','DisplayName','Measured')
hold off
legend
xlabel('Lateral Acceleration (g)')
ylabel('Longitudinal Acceleration (g)')
title('GG diagram')
axis equal
subtitle([' Simulated Lap Time - ', num2str(lap_time(end)), '(s); ',' Measured Lap Time - ', num2str(time(end)), '(s)'])
box on
%%
figure
sgtitle('QSS vs Measured Acceleration Comparison')
subplot(2,1,1)
hold on
plot(track_dist,ax_interp,'DisplayName','Measured','LineWidth',2)
plot(track_dist,final_ax./g,'DisplayName','QSS ','LineWidth',2)
legend
hold off
ylabel('Ax (g)')
xlabel('Distance (m)')
xlim tight
ylim padded
box on
title(['Longitudinal Acceleration R^{2} - ', num2str(qss_ax_r_sqr)])

subplot(2,1,2)
hold on
plot(track_dist,ay_interp,'DisplayName','Measured','LineWidth',2)
plot(track_dist,final_ay./g,'DisplayName','QSS','LineWidth',2)
legend
hold off
ylabel('Ay (g)')
xlabel('Distance (m)')
xlim tight
ylim padded
box on
box on
title(['Lateral Acceleration R^{2} - ', num2str(qss_ay_r_sqr)])

%% R-squared calculations

function r_sqr = r_square_calc(measured,predicted)

SSR = sum((measured - predicted).^2);
TSS = sum((predicted - mean(measured)).^2);
r_sqr = 1 - (SSR/TSS);

end
