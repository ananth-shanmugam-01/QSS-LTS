%% Iterative Calculation Lap Sim
clear; clc; 

addpath(genpath('C:\Users\admin\Desktop\Git Repository\QSS-LTS-F3'))
addpath('C:\Users\admin\Documents\CasAdi')

%% Create Track
sectorDistance = 1;
[trackDistance, trackCurvature] = preProccess.loadTrackModel('FSUK_2016.mat', sectorDistance);

% sectorDistance = 1;
% data = readtable("track_data.txt");
% ay_meas = data.Acc_y_g.*9.81; % Smooth Noisy data due to low sampling rate
% dist = data.dist;
% vel_ms = data.gps_speed_kmh./3.6;
% time = data.time;
% meas_lap_time = time(end);
% yaw_rate = data.yaw_rate_deg_sec;
% curvature = lowpass((ay_meas)./(vel_ms.^2),0.075);
% 
% trackDistance = 0:sectorDistance:max(dist);
% fitting_factor = 0.5;
% 
% trackCurvature = csaps(dist, curvature,fitting_factor,trackDistance);

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
FSP.V_current(ApexStartPosition-1:ApexStartPosition)           = LSP.Vx(ApexStartPosition-1:ApexStartPosition);                % Velocity (m/s)
FSP.delta(ApexStartPosition-1:ApexStartPosition)               = LSP.delta(ApexStartPosition-1:ApexStartPosition);             % steering angle (rad)
FSP.beta(ApexStartPosition-1:ApexStartPosition)                = LSP.beta(ApexStartPosition-1:ApexStartPosition);              % sideslip angle (rad)
FSP.yaw_rate(ApexStartPosition-1:ApexStartPosition)            = LSP.yaw_rate(ApexStartPosition-1:ApexStartPosition);          % yaw rate (rad/s)
FSP.throttle_position(ApexStartPosition-1:ApexStartPosition)   = LSP.throttle_position(ApexStartPosition-1:ApexStartPosition); % throttle position (-)
FSP.brake_pressure(ApexStartPosition-1:ApexStartPosition)      = 0;                                        % brake pressure (bar)
FSP.wheel_rot_fl(ApexStartPosition-1:ApexStartPosition)        = LSP.wheel_rot_fl(ApexStartPosition-1:ApexStartPosition);      % FL wheel angular velocity (rad/s)
FSP.wheel_rot_fr(ApexStartPosition-1:ApexStartPosition)        = LSP.wheel_rot_fr(ApexStartPosition-1:ApexStartPosition);      % FR wheel angular velocity (rad/s)
FSP.wheel_rot_rl(ApexStartPosition-1:ApexStartPosition)        = LSP.wheel_rot_rl(ApexStartPosition-1:ApexStartPosition);      % RL wheel angular velocity (rad/s)
FSP.wheel_rot_rr(ApexStartPosition-1:ApexStartPosition)        = LSP.wheel_rot_rr(ApexStartPosition-1:ApexStartPosition);      % RR wheel angular velocity (rad/s)

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
        
        % Dealing with Failed Solutions
        if FSP.V_current(i) > LSP.Vx(i+1)
            FSP.V_current(i+1)         = LSP.Vx(i+1);            
            FSP.delta(i)               = LSP.delta(i);            
            FSP.beta(i)                = LSP.beta(i);             
            FSP.throttle_position(i)   = 0; % throttle position (-)
            FSP.wheel_rot_fl(i)        = LSP.Vx(i)/carData.Chassis.radWheel;     
            FSP.wheel_rot_fr(i)        = LSP.Vx(i)/carData.Chassis.radWheel;    
            FSP.wheel_rot_rl(i)        = LSP.Vx(i)/carData.Chassis.radWheel;    
            FSP.wheel_rot_rr(i)        = LSP.Vx(i)/carData.Chassis.radWheel;  

        else
            FSP.V_current(i+1)           = FSP.V_current(i);        
            FSP.delta(i+1)               = FSP.delta(i-1);            
            FSP.beta(i+1)                = FSP.beta(i-1);             
            FSP.throttle_position(i+1)   = FSP.throttle_position(i-1); % throttle position (-)
            FSP.wheel_rot_fl(i+1)        = FSP.V_current(i)/carData.Chassis.radWheel;     
            FSP.wheel_rot_fr(i+1)        = FSP.V_current(i)/carData.Chassis.radWheel;    
            FSP.wheel_rot_rl(i+1)        = FSP.V_current(i)/carData.Chassis.radWheel;    
            FSP.wheel_rot_rr(i+1)        = FSP.V_current(i)/carData.Chassis.radWheel;  

        end
        
        disp('---Infeasible State / Use Previous Point ------')

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
        
        % Dealing with Failed Solutions
        if FSP.V_current(i) > LSP.Vx(i+1)
            FSP.V_current(i+1)         = LSP.Vx(i+1);            
            FSP.delta(i)               = LSP.delta(i);            
            FSP.beta(i)                = LSP.beta(i);             
            FSP.throttle_position(i)   = 0; % throttle position (-)
            FSP.wheel_rot_fl(i)        = LSP.Vx(i)/carData.Chassis.radWheel;     
            FSP.wheel_rot_fr(i)        = LSP.Vx(i)/carData.Chassis.radWheel;    
            FSP.wheel_rot_rl(i)        = LSP.Vx(i)/carData.Chassis.radWheel;    
            FSP.wheel_rot_rr(i)        = LSP.Vx(i)/carData.Chassis.radWheel;  

        else
            FSP.V_current(i+1)           = FSP.V_current(i);        
            FSP.delta(i+1)               = FSP.delta(i-1);            
            FSP.beta(i+1)                = FSP.beta(i-1);             
            FSP.throttle_position(i+1)   = FSP.throttle_position(i-1); % throttle position (-)
            FSP.wheel_rot_fl(i+1)        = FSP.V_current(i)/carData.Chassis.radWheel;     
            FSP.wheel_rot_fr(i+1)        = FSP.V_current(i)/carData.Chassis.radWheel;    
            FSP.wheel_rot_rl(i+1)        = FSP.V_current(i)/carData.Chassis.radWheel;    
            FSP.wheel_rot_rr(i+1)        = FSP.V_current(i)/carData.Chassis.radWheel;  

        end
        
        disp('---Infeasible State / Use Previous Point ------')

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

        % Dealing with Failed Solutions

        if RSP.V_current(i) > LSP.Vx(i-1) % If current velocity is higher than next boundary speed
            RSP.V_current(i-1)         = LSP.Vx(i-1);            
            RSP.delta(i)               = LSP.delta(i);            
            RSP.beta(i)                = LSP.beta(i);             
            RSP.brake_pressure(i)      = 0; % throttle position (-)
            RSP.wheel_rot_fl(i)        = LSP.Vx(i-1)/carData.Chassis.radWheel;     
            RSP.wheel_rot_fr(i)        = LSP.Vx(i-1)/carData.Chassis.radWheel;    
            RSP.wheel_rot_rl(i)        = LSP.Vx(i-1)/carData.Chassis.radWheel;    
            RSP.wheel_rot_rr(i)        = LSP.Vx(i-1)/carData.Chassis.radWheel;  

        else
            RSP.V_current(i-1)         = RSP.V_current(i);        
            RSP.delta(i)               = RSP.delta(i+1);            
            RSP.beta(i)                = RSP.beta(i+1);             
            RSP.brake_pressure(i)      = RSP.brake_pressure(i+1); % throttle position (-)
            RSP.wheel_rot_fl(i)        = RSP.V_current(i)/carData.Chassis.radWheel;     
            RSP.wheel_rot_fr(i)        = RSP.V_current(i)/carData.Chassis.radWheel;    
            RSP.wheel_rot_rl(i)        = RSP.V_current(i)/carData.Chassis.radWheel;    
            RSP.wheel_rot_rr(i)        = RSP.V_current(i)/carData.Chassis.radWheel;  

        end

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
