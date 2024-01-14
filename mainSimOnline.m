%% Iterative Calculation Lap Sim
clear; clc; 

addpath(genpath('C:\Users\admin\Desktop\Git Repository\QSS-LTS-F3'))
addpath('C:\Users\admin\Documents\CasAdi')

%% Create Track
% sectorDistance = 3;
% [trackDistance, trackCurvature] = preProccess.trackModel('dataFiles\230722_Endurance_lap2.mat', sectorDistance);

sector_length = 1;
data = readtable("track_data.txt");
ay_meas = data.Acc_y_g.*9.81; % Smooth Noisy data due to low sampling rate
dist = data.dist;
vel_ms = data.gps_speed_kmh./3.6;
time = data.time;
meas_lap_time = time(end);
yaw_rate = data.yaw_rate_deg_sec;
curvature = lowpass((ay_meas)./(vel_ms.^2),0.075);

trackDistance = 0:sector_length:max(dist);
fitting_factor = 0.5;

trackCurvature = csaps(dist, curvature,fitting_factor,trackDistance);

trackData = struct;
trackData.trackDistance = trackDistance;
trackData.trackCurvature = trackCurvature;

%% Initialise Vehicle Model
carData = preProccess.initVehicleModel();

%% Generate Boundary Speed Profile
clc;

LSP = struct;

% Optimiser Inputs - Decision Variables + control inputs
LSP.Vx                  = zeros(numel(trackCurvature),1);       % Velocity (m/s)
LSP.delta               = zeros(numel(trackCurvature),1);       % steering angle (rad)
LSP.beta                = zeros(numel(trackCurvature),1);       % sideslip angle (rad)
LSP.yaw_rate            = zeros(numel(trackCurvature),1);       % yaw rate (rad/s)
LSP.throttle_position   = zeros(numel(trackCurvature),1);       % throttle position (-)
LSP.brake_pressure      = zeros(numel(trackCurvature),1);       % brake pressure (bar)
LSP.wheel_rot_fl        = zeros(numel(trackCurvature),1);       % FL wheel angular velocity (rad/s)
LSP.wheel_rot_fr        = zeros(numel(trackCurvature),1);       % FR wheel angular velocity (rad/s)
LSP.wheel_rot_rl        = zeros(numel(trackCurvature),1);       % RL wheel angular velocity (rad/s)
LSP.wheel_rot_rr        = zeros(numel(trackCurvature),1);       % RR wheel angular velocity (rad/s)

% Output States - g(x)
LSP.Ay                  = zeros(numel(trackCurvature),1);
LSP.Ax                  = zeros(numel(trackCurvature),1);
LSP.Ax_control          = zeros(numel(trackCurvature),1);
LSP.F_tractive          = zeros(numel(trackCurvature),1);
LSP.F_braking           = zeros(numel(trackCurvature),1);
LSP.F_drag              = zeros(numel(trackCurvature),1);


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

    % find Maximum Forward Acceleration
    % nlpBSP = []; 
    
    % Call Vehicle Model
    vehicleModels.boundarySpeed;
    
    % objective
    nlpBSP.minimize(-Vx);
    
    % initialization of decision variables
    nlpBSP.set_initial(Vx,LSP.Vx(i-1));
    nlpBSP.set_initial(delta,LSP.delta(i-1));
    nlpBSP.set_initial(beta,LSP.beta(i-1));
    nlpBSP.set_initial(yaw_rate,LSP.yaw_rate(i-1));
    nlpBSP.set_initial(wheel_rot_fl,LSP.wheel_rot_fl(i-1));
    nlpBSP.set_initial(wheel_rot_fr,LSP.wheel_rot_fr(i-1));
    nlpBSP.set_initial(wheel_rot_rl,LSP.wheel_rot_rl(i-1));
    nlpBSP.set_initial(wheel_rot_rr,LSP.wheel_rot_rr(i-1));

    % Constraints
    nlpBSP.subject_to(ay_out == ay_control);
    nlpBSP.subject_to(ax_out == ax_control);
    nlpBSP.subject_to(yaw_rate == curvature*Vx);
    nlpBSP.subject_to(-10 <= Mz_out <= 10);

    nlpBSP.subject_to(-12*pi/180<=alpha_fl<=12*pi/180);
    nlpBSP.subject_to(-12*pi/180<=alpha_fr<=12*pi/180);
    nlpBSP.subject_to(-12*pi/180<=alpha_rl<=12*pi/180);
    nlpBSP.subject_to(-12*pi/180<=alpha_rr<=12*pi/180);

    nlpBSP.subject_to(-0.1<=kappa_fl<=0.1);
    nlpBSP.subject_to(-0.1<=kappa_fr<=0.1);
    nlpBSP.subject_to(-0.1<=kappa_rl<=0.1);
    nlpBSP.subject_to(-0.1<=kappa_rr<=0.1);

    % solve
    plugin_opts = struct('print_time',0);
    solver_opts = struct('constr_viol_tol',0.1,'acceptable_obj_change_tol',0.001,'print_level',0);
    nlpBSP.solver('ipopt',plugin_opts,solver_opts);  
    sol = nlpBSP.solve();
    
    % extract results
    LSP.Vx(i)                  = sol.value(Vx);                % Velocity (m/s)
    LSP.Ay(i)                  = sol.value(ay_out);            % Lateral Acceleration (m/s^2)
    LSP.Ax(i)                  = sol.value(ax_out);            % Lateral Acceleration (m/s^2)
    LSP.Ax_control(i)             = sol.value(ax_control);
    LSP.delta(i)               = sol.value(delta);             % steering angle (rad)
    LSP.beta(i)                = sol.value(beta);              % sideslip angle (rad)
    LSP.yaw_rate(i)            = sol.value(yaw_rate);          % yaw rate (rad/s)
    LSP.wheel_rot_fl(i)        = sol.value(wheel_rot_fl);      % FL wheel angular velocity (rad/s)
    LSP.wheel_rot_fr(i)        = sol.value(wheel_rot_fr);      % FR wheel angular velocity (rad/s)
    LSP.wheel_rot_rl(i)        = sol.value(wheel_rot_rl);      % RL wheel angular velocity (rad/s)
    LSP.wheel_rot_rr(i)        = sol.value(wheel_rot_rr);      % RR wheel angular velocity (rad/s)
    LSP.F_drag(i)       = sol.value(Fd);

    disp(['Boundary Speed Profile -- ',   num2str(i), '/',num2str(numel(trackCurvature))])

end

%% Identify Apexes

% Start point from slowest corner
[ApexStartSpeed, ApexStartPosition] = min(LSP.Vx);


%% Forward Speed Profile
clear i; clc;

% Optimiser Inputs - Decision Variables + control inputs
FSP.V_current           = zeros(numel(trackCurvature),1);       % Velocity (m/s)
FSP.delta               = zeros(numel(trackCurvature),1);       % steering angle (rad)
FSP.beta                = zeros(numel(trackCurvature),1);       % sideslip angle (rad)
FSP.yaw_rate            = zeros(numel(trackCurvature),1);       % yaw rate (rad/s)
FSP.throttle_position   = zeros(numel(trackCurvature),1);       % throttle position (-)
FSP.brake_pressure      = zeros(numel(trackCurvature),1);       % brake pressure (bar)
FSP.wheel_rot_fl        = zeros(numel(trackCurvature),1);       % FL wheel angular velocity (rad/s)
FSP.wheel_rot_fr        = zeros(numel(trackCurvature),1);       % FR wheel angular velocity (rad/s)
FSP.wheel_rot_rl        = zeros(numel(trackCurvature),1);       % RL wheel angular velocity (rad/s)
FSP.wheel_rot_rr        = zeros(numel(trackCurvature),1);       % RR wheel angular velocity (rad/s)

% Output States - g(x)
FSP.Ay                  = zeros(numel(trackCurvature),1);
FSP.Ax                  = zeros(numel(trackCurvature),1);
FSP.Ax_control          = zeros(numel(trackCurvature),1);
FSP.F_tractive          = zeros(numel(trackCurvature),1);
FSP.F_braking           = zeros(numel(trackCurvature),1);
FSP.F_drag              = zeros(numel(trackCurvature),1);


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
        Kt = trackCurvature(i);
        V_current = FSP.V_current(i);
        ay_control = Kt * V_current^2;
        
        % find Maximum Forward Acceleration
        nlpFSP = []; 
        
        % Call Vehicle Model
        vehicleModels.forwardSpeed;
        
        % objective
        nlpFSP.minimize(-V_out);
        
        % initialization of decision variables
        nlpFSP.set_initial(delta,0);
        nlpFSP.set_initial(beta,0);
        nlpFSP.set_initial(throttle_position,1);
        nlpFSP.set_initial(wheel_rot_fl,LSP.wheel_rot_fl(i));
        nlpFSP.set_initial(wheel_rot_fr,LSP.wheel_rot_fr(i));
        nlpFSP.set_initial(wheel_rot_rl,LSP.wheel_rot_rl(i));
        nlpFSP.set_initial(wheel_rot_rr,LSP.wheel_rot_rr(i));
    
        % Constraints
        nlpFSP.subject_to(ay_out == ay_control);
        nlpFSP.subject_to(ax_out == ax_control);
        nlpFSP.subject_to(-10 <= Mz_out <= 10);
   
        nlpFSP.subject_to(-12*pi/180<=alpha_fl<=12*pi/180);
        nlpFSP.subject_to(-12*pi/180<=alpha_fr<=12*pi/180);
        nlpFSP.subject_to(-12*pi/180<=alpha_rl<=12*pi/180);
        nlpFSP.subject_to(-12*pi/180<=alpha_rr<=12*pi/180);
    
        nlpFSP.subject_to(-0.1<=kappa_fl<=0.1);
        nlpFSP.subject_to(-0.1<=kappa_fr<=0.1);
        nlpFSP.subject_to(-0.1<=kappa_rl<=0.1);
        nlpFSP.subject_to(-0.1<=kappa_rr<=0.1);

        
        if i == numel(trackDistance)
        
            nlpFSP.subject_to(V_out <= LSP.Vx(1));
    
            % solve
            plugin_opts = struct('print_time',0);
            solver_opts = struct('constr_viol_tol',0.1,'acceptable_obj_change_tol',0.01,'print_level',0);
            nlpFSP.solver('ipopt',plugin_opts,solver_opts);  
            sol = nlpFSP.solve();
            % extract results
            FSP.V_current(1)           = sol.value(V_out);                % Velocity (m/s)
            FSP.Ay(i)                  = sol.value(ay_out);            % Lateral Acceleration (m/s^2)
            FSP.Ax(i)                  = sol.value(ax_out);            % Lateral Acceleration (m/s^2)
            FSP.Ax_control(i)          = sol.value(ax_control);
            FSP.delta(i)               = sol.value(delta);             % steering angle (rad)
            FSP.beta(i)                = sol.value(beta);              % sideslip angle (rad)
            FSP.yaw_rate(i)            = sol.value(yaw_rate);          % yaw rate (rad/s)
            FSP.throttle_position(i)   = sol.value(throttle_position); % throttle position (-)
            FSP.wheel_rot_fl(i)        = sol.value(wheel_rot_fl);      % FL wheel angular velocity (rad/s)
            FSP.wheel_rot_fr(i)        = sol.value(wheel_rot_fr);      % FR wheel angular velocity (rad/s)
            FSP.wheel_rot_rl(i)        = sol.value(wheel_rot_rl);      % RL wheel angular velocity (rad/s)
            FSP.wheel_rot_rr(i)        = sol.value(wheel_rot_rr);      % RR wheel angular velocity (rad/s)
        
            FSP.F_tractive(i)   = sol.value(F_tractive);
            FSP.F_drag(i)       = sol.value(Fd);

        else
            nlpFSP.subject_to(V_out <= LSP.Vx(i+1));
            % solve
            plugin_opts = struct('print_time',0);
            solver_opts = struct('constr_viol_tol',0.1,'acceptable_obj_change_tol',0.01,'print_level',0);
            nlpFSP.solver('ipopt',plugin_opts,solver_opts);  
            sol = nlpFSP.solve();

            % extract results
            FSP.V_current(i+1)         = sol.value(V_out);                % Velocity (m/s)
            FSP.Ay(i)                  = sol.value(ay_out);            % Lateral Acceleration (m/s^2)
            FSP.Ax(i)                  = sol.value(ax_out);            % Lateral Acceleration (m/s^2)
            FSP.Ax_control(i)          = sol.value(ax_control);
            FSP.delta(i)               = sol.value(delta);             % steering angle (rad)
            FSP.beta(i)                = sol.value(beta);              % sideslip angle (rad)
            FSP.yaw_rate(i)            = sol.value(yaw_rate);          % yaw rate (rad/s)
            FSP.throttle_position(i)   = sol.value(throttle_position); % throttle position (-)
            FSP.wheel_rot_fl(i)        = sol.value(wheel_rot_fl);      % FL wheel angular velocity (rad/s)
            FSP.wheel_rot_fr(i)        = sol.value(wheel_rot_fr);      % FR wheel angular velocity (rad/s)
            FSP.wheel_rot_rl(i)        = sol.value(wheel_rot_rl);      % RL wheel angular velocity (rad/s)
            FSP.wheel_rot_rr(i)        = sol.value(wheel_rot_rr);      % RR wheel angular velocity (rad/s)
        
            FSP.F_tractive(i)   = sol.value(F_tractive);
            FSP.F_drag(i)       = sol.value(Fd);

        end

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
        Kt = trackCurvature(i);
        V_current = FSP.V_current(i);
        ay_control = Kt * V_current^2;
        
        % find Maximum Forward Acceleration
        nlpFSP = []; 
        
        % Call Vehice Model
        vehicleModels.forwardSpeed;
        
        % objective
        nlpFSP.minimize(-V_out);
        
        % initialization of decision variables
        nlpFSP.set_initial(delta,0);
        nlpFSP.set_initial(beta,0);
        nlpFSP.set_initial(throttle_position,1);
        nlpFSP.set_initial(wheel_rot_fl,LSP.wheel_rot_fl(i));
        nlpFSP.set_initial(wheel_rot_fr,LSP.wheel_rot_fr(i));
        nlpFSP.set_initial(wheel_rot_rl,LSP.wheel_rot_rl(i));
        nlpFSP.set_initial(wheel_rot_rr,LSP.wheel_rot_rr(i));
    
        % Constraints
        nlpFSP.subject_to(ay_out == ay_control);
        nlpFSP.subject_to(ax_out == ax_control);
        nlpFSP.subject_to(-10 <= Mz_out <= 10);
   
        nlpFSP.subject_to(-12*pi/180<=alpha_fl<=12*pi/180);
        nlpFSP.subject_to(-12*pi/180<=alpha_fr<=12*pi/180);
        nlpFSP.subject_to(-12*pi/180<=alpha_rl<=12*pi/180);
        nlpFSP.subject_to(-12*pi/180<=alpha_rr<=12*pi/180);
    
        nlpFSP.subject_to(-0.1<=kappa_fl<=0.1);
        nlpFSP.subject_to(-0.1<=kappa_fr<=0.1);
        nlpFSP.subject_to(-0.1<=kappa_rl<=0.1);
        nlpFSP.subject_to(-0.1<=kappa_rr<=0.1);
        
        nlpFSP.subject_to(V_out <= LSP.Vx(i+1));
        % solve
        plugin_opts = struct('print_time',0);
        solver_opts = struct('constr_viol_tol',0.1,'acceptable_obj_change_tol',0.01,'print_level',0);
        nlpFSP.solver('ipopt',plugin_opts,solver_opts);  
        sol = nlpFSP.solve();

        % extract results
        FSP.V_current(i+1)         = sol.value(V_out);                % Velocity (m/s)
        FSP.Ay(i)                  = sol.value(ay_out);            % Lateral Acceleration (m/s^2)
        FSP.Ax(i)                  = sol.value(ax_out);            % Lateral Acceleration (m/s^2)
        FSP.Ax_control(i)          = sol.value(ax_control);
        FSP.delta(i)               = sol.value(delta);             % steering angle (rad)
        FSP.beta(i)                = sol.value(beta);              % sideslip angle (rad)
        FSP.yaw_rate(i)            = sol.value(yaw_rate);          % yaw rate (rad/s)
        FSP.throttle_position(i)   = sol.value(throttle_position); % throttle position (-)
        FSP.wheel_rot_fl(i)        = sol.value(wheel_rot_fl);      % FL wheel angular velocity (rad/s)
        FSP.wheel_rot_fr(i)        = sol.value(wheel_rot_fr);      % FR wheel angular velocity (rad/s)
        FSP.wheel_rot_rl(i)        = sol.value(wheel_rot_rl);      % RL wheel angular velocity (rad/s)
        FSP.wheel_rot_rr(i)        = sol.value(wheel_rot_rr);      % RR wheel angular velocity (rad/s)
    
        FSP.F_tractive(i)   = sol.value(F_tractive);
        FSP.F_drag(i)       = sol.value(Fd);

    catch 
        FSP.V_current(i+1)         = LSP.Vx(i+1);                % Velocity (m/s)
        disp('---Infeasible State / Hit Boundary Speed------')
    end

    disp(['Forward Speed Profile -- ',  num2str(i), '/',num2str(numel(trackCurvature))])

end

%% Reverse Speed Profile

% Optimiser Inputs - Decision Variables + control inputs
RSP.V_current           = zeros(numel(trackCurvature),1);       % Velocity (m/s)
RSP.delta               = zeros(numel(trackCurvature),1);       % steering angle (rad)
RSP.beta                = zeros(numel(trackCurvature),1);       % sideslip angle (rad)
RSP.yaw_rate            = zeros(numel(trackCurvature),1);       % yaw rate (rad/s)
RSP.throttle_position   = zeros(numel(trackCurvature),1);       % throttle position (-)
RSP.brake_pressure      = zeros(numel(trackCurvature),1);       % brake pressure (bar)
RSP.wheel_rot_fl        = zeros(numel(trackCurvature),1);       % FL wheel angular velocity (rad/s)
RSP.wheel_rot_fr        = zeros(numel(trackCurvature),1);       % FR wheel angular velocity (rad/s)
RSP.wheel_rot_rl        = zeros(numel(trackCurvature),1);       % RL wheel angular velocity (rad/s)
RSP.wheel_rot_rr        = zeros(numel(trackCurvature),1);       % RR wheel angular velocity (rad/s)

% Output States - g(x)
RSP.Ay                  = zeros(numel(trackCurvature),1);
RSP.Ax                  = zeros(numel(trackCurvature),1);
RSP.Ax_control          = zeros(numel(trackCurvature),1);
RSP.F_tractive          = zeros(numel(trackCurvature),1);
RSP.F_braking           = zeros(numel(trackCurvature),1);
RSP.F_drag              = zeros(numel(trackCurvature),1);

RSP.V_current(end)           = LSP.Vx(end);

for i = numel(trackDistance):-1:2
     try    
        % Input Values
        Kt = trackCurvature(i);
        V_current = RSP.V_current(i);
        ay_control = Kt * V_current^2;
        
        % find Maximum Forward Acceleration
        nlpRSP = []; 
        
        % Call Vehice Model
        vehicleModels.reverseSpeed;
        
        % objective
        nlpRSP.minimize(-V_out);
        
        % initialization of decision variables
        nlpRSP.set_initial(V_out,V_current);
        nlpRSP.set_initial(delta,0);
        nlpRSP.set_initial(beta,0);
        nlpRSP.set_initial(brake_pressure,0);
        nlpRSP.set_initial(wheel_rot_fl,V_current/carData.Chassis.radWheel);
        nlpRSP.set_initial(wheel_rot_fr,V_current/carData.Chassis.radWheel);
        nlpRSP.set_initial(wheel_rot_rl,V_current/carData.Chassis.radWheel);
        nlpRSP.set_initial(wheel_rot_rr,V_current/carData.Chassis.radWheel);
    
        % Constraints
        nlpRSP.subject_to(ay_out == ay_control);
        nlpRSP.subject_to(ax_out == ax_control);
        nlpRSP.subject_to(-10 <= Mz_out <= 10);
        nlpRSP.subject_to(V_out == V_out_guess);
        nlpRSP.subject_to(V_out <= LSP.Vx(i-1));
   
        nlpRSP.subject_to(-12*pi/180<=alpha_fl<=12*pi/180);
        nlpRSP.subject_to(-12*pi/180<=alpha_fr<=12*pi/180);
        nlpRSP.subject_to(-12*pi/180<=alpha_rl<=12*pi/180);
        nlpRSP.subject_to(-12*pi/180<=alpha_rr<=12*pi/180);
    
        nlpRSP.subject_to(-0.1<=kappa_fl<=0.1);
        nlpRSP.subject_to(-0.1<=kappa_fr<=0.1);
        nlpRSP.subject_to(-0.1<=kappa_rl<=0.1);
        nlpRSP.subject_to(-0.1<=kappa_rr<=0.1);
        
        % solve
        plugin_opts = struct('print_time',0);
        solver_opts = struct('constr_viol_tol',0.1,'acceptable_obj_change_tol',0.01,'print_level',0);
        nlpRSP.solver('ipopt',plugin_opts,solver_opts);  
        sol = nlpRSP.solve();

        % extract results
        RSP.V_current(i-1)         = sol.value(V_out);                % Velocity (m/s)
        RSP.Ay(i)                  = sol.value(ay_out);            % Lateral Acceleration (m/s^2)
        RSP.Ax(i)                  = sol.value(ax_out);            % Lateral Acceleration (m/s^2)
        RSP.Ax_control(i)          = sol.value(ax_control);
        RSP.delta(i)               = sol.value(delta);             % steering angle (rad)
        RSP.beta(i)                = sol.value(beta);              % sideslip angle (rad)
        RSP.yaw_rate(i)            = sol.value(yaw_rate);          % yaw rate (rad/s)
        RSP.brake_pressure(i)      = sol.value(brake_pressure);    % throttle position (-)
        RSP.wheel_rot_fl(i)        = sol.value(wheel_rot_fl);      % FL wheel angular velocity (rad/s)
        RSP.wheel_rot_fr(i)        = sol.value(wheel_rot_fr);      % FR wheel angular velocity (rad/s)
        RSP.wheel_rot_rl(i)        = sol.value(wheel_rot_rl);      % RL wheel angular velocity (rad/s)
        RSP.wheel_rot_rr(i)        = sol.value(wheel_rot_rr);      % RR wheel angular velocity (rad/s)
    
        RSP.F_brake(i)      = sol.value(F_brake);
        RSP.F_drag(i)       = sol.value(Fd);

    catch 
        RSP.V_current(i-1)         = LSP.Vx(i-1);                % Velocity (m/s)
        disp('---Infeasible State / Hit Boundary Speed------')
    end

    disp(['Reverse Speed Profile -- ', num2str(i), '/',num2str(numel(trackCurvature))])

end

%%

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
