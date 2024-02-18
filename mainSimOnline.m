%% Iterative Calculation Lap Sim
clear; clc; 

addpath(genpath('C:\Users\admin\Desktop\Git Repository\QSS-LTS-F3'))
addpath('C:\Users\admin\Documents\CasAdi')

%% Create Track
% sectorDistance = 3;
% [trackDistance, trackCurvature] = preProccess.loadTrackModel('FSUK_2016.mat', sectorDistance);

sectorDistance = 1;
data = readtable("track_data.txt");
ay_meas = data.Acc_y_g.*9.81; % Smooth Noisy data due to low sampling rate
dist = data.dist;
vel_ms = data.gps_speed_kmh./3.6;
time = data.time;
meas_lap_time = time(end);
yaw_rate = data.yaw_rate_deg_sec;
curvature = lowpass((ay_meas)./(vel_ms.^2),0.075);

trackDistance = 0:sectorDistance:max(dist);
fitting_factor = 0.5;

trackCurvature = csaps(dist, curvature,fitting_factor,trackDistance);

trackData = struct;
trackData.trackDistance = trackDistance;
trackData.trackCurvature = trackCurvature;

%% Initialise Vehicle Model
carData = preProccess.initVehicleModel();

%% Initialise Result Structs
[LSP, FSP, RSP] = preProccess.initialiseResultStructs(numel(trackCurvature));

%% Generate Boundary Speed Profile
startTimer = tic;

% Initial Guess for point 1
LSP.Vx(1)                  = carData.Powertrain.vMax;                               % Velocity (m/s)
LSP.delta(1)               = 0;                                                     % steering angle (rad)
LSP.beta(1)                = 0;                                                     % sideslip angle (rad)
LSP.yaw_rate(1)            = 0;                                                     % yaw rate (rad/s)
LSP.throttle_position(1)   = 0;                                                     % throttle position (-)
LSP.brake_pressure(1)      = 0;                                                     % brake pressure (bar)
LSP.wheel_rot_fl(1)        = carData.Powertrain.vMax/carData.Chassis.radWheel;      % FL wheel angular velocity (rad/s)
LSP.wheel_rot_fr(1)        = carData.Powertrain.vMax/carData.Chassis.radWheel;      % FR wheel angular velocity (rad/s)
LSP.wheel_rot_rl(1)        = carData.Powertrain.vMax/carData.Chassis.radWheel;      % RL wheel angular velocity (rad/s)
LSP.wheel_rot_rr(1)        = carData.Powertrain.vMax/carData.Chassis.radWheel;      % RR wheel angular velocity (rad/s)

for i = 2:length(trackCurvature)

    curvature = trackCurvature(i);

    LSP = vehicleModels.boundarySpeed(i, LSP, curvature, carData);

    disp(['Boundary Speed Profile -- ',   num2str(i), '/',num2str(numel(trackCurvature))])

end

% Identify Apexes
% Start point from slowest corner
[ApexStartSpeed, ApexStartPosition] = min(LSP.Vx);


%% Forward Speed Profile
clear i; clc;

% Initial Guess for slowest apex
FSP.V_current(ApexStartPosition)           = LSP.Vx(ApexStartPosition);                % Velocity (m/s)
FSP.delta(ApexStartPosition)               = LSP.delta(ApexStartPosition);             % steering angle (rad)
FSP.beta(ApexStartPosition)                = LSP.beta(ApexStartPosition);              % sideslip angle (rad)
FSP.yaw_rate(ApexStartPosition)            = LSP.yaw_rate(ApexStartPosition);          % yaw rate (rad/s)
FSP.throttle_position(ApexStartPosition)   = LSP.throttle_position(ApexStartPosition); % throttle position (-)
FSP.brake_pressure(ApexStartPosition)      = 0;                                        % brake pressure (bar)
FSP.wheel_rot_fl(ApexStartPosition)        = LSP.wheel_rot_fl(ApexStartPosition);      % FL wheel angular velocity (rad/s)
FSP.wheel_rot_fr(ApexStartPosition)        = LSP.wheel_rot_fr(ApexStartPosition);      % FR wheel angular velocity (rad/s)
FSP.wheel_rot_rl(ApexStartPosition)        = LSP.wheel_rot_rl(ApexStartPosition);      % RL wheel angular velocity (rad/s)
FSP.wheel_rot_rr(ApexStartPosition)        = LSP.wheel_rot_rr(ApexStartPosition);      % RR wheel angular velocity (rad/s)

% Start from slowest Apex to End of Track

for i = ApexStartPosition:numel(trackDistance)

    try
    
        % Input Values
        curvature = trackCurvature(i);
        V_current = FSP.V_current(i);
        ay_control = curvature * V_current^2;
        
        % find Maximum Forward Acceleration
        FSP = vehicleModels.forwardSpeed(i, sectorDistance, LSP, FSP, curvature, V_current, ay_control, carData);

    catch 

        FSP.V_current(i+1)         = LSP.Vx(i+1);                % Velocity (m/s)
        disp('---Infeasible State / Hit Boundary Speed------')

    end

    disp(['Forward Speed Profile -- ',  num2str(i), '/',num2str(numel(trackCurvature))])
    
end

% From start of track to slowest apex

for i = 1:ApexStartPosition-1
    try    
        % Input Values
        curvature = trackCurvature(i);
        V_current = FSP.V_current(i);
        ay_control = curvature * V_current^2;
        
        % find Maximum Forward Acceleration
        FSP = vehicleModels.forwardSpeed(i, sectorDistance, LSP, FSP, curvature, V_current, ay_control, carData);

    catch 
        FSP.V_current(i+1)         = LSP.Vx(i+1);                % Velocity (m/s)
        disp('---Infeasible State / Hit Boundary Speed------')
    end

    disp(['Forward Speed Profile -- ',  num2str(i), '/',num2str(numel(trackCurvature))])

end

%% Reverse Speed Profile

RSP.V_current(end)           = LSP.Vx(end);

for i = numel(trackDistance):-1:2
     try    
        % Input Values
        curvature = trackCurvature(i);
        V_current = RSP.V_current(i);
        ay_control = curvature * V_current^2;
        
        RSP = vehicleModels.reverseSpeed(i, sectorDistance, LSP, RSP, curvature, V_current, ay_control, carData);

    catch 
        RSP.V_current(i-1)         = LSP.Vx(i-1);                % Velocity (m/s)
        disp('---Infeasible State / Hit Boundary Speed------')
    end

    disp(['Reverse Speed Profile -- ', num2str(i), '/',num2str(numel(trackCurvature))])

end

%%
stopTimer = toc(startTimer);
disp(['Online Calculation Complete. Time taken: ', num2str(stopTimer), '(s)'])

lapData = postProcess.fnPostSimProcess(trackData,LSP,FSP,RSP);

%% Plots

figure(1); plotbrowser("on");

tiledlayout(3,1)
nexttile
hold on
plot(trackData.trackDistance,LSP.Vx)
plot(trackData.trackDistance,FSP.V_current)
plot(trackData.trackDistance,RSP.V_current)
plot(trackData.trackDistance, lapData.vCar,'k','LineWidth',1.5)
hold off
subtitle(['Calculated Lap Time --', num2str(lapData.time(end)),'(s)'])

nexttile
hold on
plot(trackDistance,lapData.gLat,'DisplayName','gLat')
plot(trackDistance,lapData.gLong,'DisplayName','gLong')
ylabel('Acceleration')
hold off

nexttile
plot(trackDistance,lapData.vYaw)
ylabel('yawRate')


figure(2); plotbrowser("on"); tiledlayout(4,1)
nexttile
hold on
plot(trackData.trackDistance,LSP.Vx)
plot(trackData.trackDistance,FSP.V_current)
plot(trackData.trackDistance,RSP.V_current)
plot(trackData.trackDistance, lapData.vCar,'k','LineWidth',1.5)
hold off
subtitle(['Simulated Lap Time: ', num2str(lapData.time(end)),'(s)'])

nexttile
plot(trackDistance,lapData.aSteer)
ylabel('aSteer')

nexttile
plot(trackDistance,lapData.rThrottle)
ylabel('rThrottle')

nexttile
plot(trackDistance,lapData.pBrake)
ylabel('pBrake')
