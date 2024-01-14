clear; clc; 

addpath(genpath('C:\Users\admin\Desktop\Git Repository\QSS-LTS-F3'))
addpath('C:\Users\admin\Documents\CasAdi')

%% Create Track
sectorDistance = 1;
[trackDistance, trackCurvature] = preProccess.trackModel('dataFiles\230722_Endurance_lap2.mat', sectorDistance);

%% Initialise Vehicle Model
carData = preProccess.initVehicleModel();

%% run GGV Calc
% Unable to run in function?
startTimer = tic;

velocityRange = linspace(10,carData.Powertrain.vMax-2,10);
clear GGV_out
GGV_out = [];

for i = 1:numel(velocityRange)
    velocity = velocityRange(i);
    % initialize GGV-data
    ax_steps = 10;  % number of discretization points
    
    GGV = struct();
    GGV.ax = zeros(ax_steps,1);
    GGV.ay = zeros(ax_steps,1);
    GGV.delta   = zeros(ax_steps,1);
    GGV.beta    = zeros(ax_steps,1);
    GGV.yaw_rate = zeros(ax_steps,1);
    GGV.wheel_rot_fl = zeros(ax_steps,1);
    GGV.wheel_rot_fr = zeros(ax_steps,1);
    GGV.wheel_rot_rl = zeros(ax_steps,1);
    GGV.wheel_rot_rr = zeros(ax_steps,1);
      
    % find Maximum Forward Acceleration
    ggvModels.acceleration;

    % objective
    FA.minimize(-ax_out);
    
    % initialization of decision variables
    FA.set_initial(delta,0);
    FA.set_initial(beta,0);
    FA.set_initial(yaw_rate,0);
    FA.set_initial(throttle_position,1);
    FA.set_initial(wheel_rot_fl,velocity/carData.Chassis.radWheel);
    FA.set_initial(wheel_rot_fr,velocity/carData.Chassis.radWheel);
    FA.set_initial(wheel_rot_rl,velocity/carData.Chassis.radWheel);
    FA.set_initial(wheel_rot_rr,velocity/carData.Chassis.radWheel);
    
    % Constraints
    FA.subject_to(ay_out == ay_control);
    FA.subject_to(ax_out == ax_control);
    FA.subject_to(ay_out == 0);
    FA.subject_to(kappa_fl == 0);
    FA.subject_to(kappa_fr == 0);
    
    plugin_opts = struct('print_time',0);
    solver_opts = struct('constr_viol_tol',0.1,'acceptable_obj_change_tol',0.001,'print_level',0);
    FA.solver('ipopt',plugin_opts,solver_opts);
    sol = FA.solve();
    
    % extract results
    GGV.ay(1) = sol.value(ay_out);
    GGV.ax(1) = sol.value(ax_out);
    
    % find Maximum Braking Deceleration 
    ggvModels.deceleration;

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
    
    % Combined Acceleration
    GGV.ax = linspace(GGV.ax(1),GGV.ax(end),length(GGV.ax));

    for i_ = 2:length(GGV.ax)-1
        
        if GGV.ax(i_) < 0
 
            % Model
            ggvModels.combinedAccelerationBraking;

            % objective
            CAB.minimize(-ay_out);
            % initialization of decision variables
            CAB.set_initial(delta,0);
            CAB.set_initial(beta,0);
            CAB.set_initial(yaw_rate,0);

            CAB.set_initial(brake_pressure,0);
            CAB.set_initial(wheel_rot_fl,velocity/carData.Chassis.radWheel);
            CAB.set_initial(wheel_rot_fr,velocity/carData.Chassis.radWheel);
            CAB.set_initial(wheel_rot_rl,velocity/carData.Chassis.radWheel);
            CAB.set_initial(wheel_rot_rr,velocity/carData.Chassis.radWheel);
        
            % steady state constraints
            CAB.subject_to(ay_out == ay_control);
            CAB.subject_to(ax_out == ax_control);
            CAB.subject_to(-10 <= Mz_out <= 10);

            % longitudinal acceleration constraint     
            CAB.subject_to(ax_out == GGV.ax(i_));
    
            % solve
            plugin_opts = struct('print_time',0);
            solver_opts = struct('constr_viol_tol',0.1,'acceptable_obj_change_tol',0.01,'print_level',8);
            CAB.solver('ipopt',plugin_opts,solver_opts);
            sol = CAB.solve();
            % extract results
            GGV.ay(i_) = sol.value(ay_out);
            GGV.ax(i_) = sol.value(ax_out);
            
        else
            
            % Model
            ggvModels.combinedAccelerationTractive;
            
            % objective
            CAT.minimize(-ay_out);
            % initialization of decision variables
            CAT.set_initial(delta,0);
            CAT.set_initial(beta,0);
            CAT.set_initial(yaw_rate,0);
            CAT.set_initial(throttle_position,1);

            CAT.set_initial(wheel_rot_fl,velocity/carData.Chassis.radWheel);
            CAT.set_initial(wheel_rot_fr,velocity/carData.Chassis.radWheel);
            CAT.set_initial(wheel_rot_rl,velocity/carData.Chassis.radWheel);
            CAT.set_initial(wheel_rot_rr,velocity/carData.Chassis.radWheel);
        
            % steady state constraints
            CAT.subject_to(ay_out == ay_control);
            CAT.subject_to(ax_out == ax_control);
            CAT.subject_to(kappa_fl == 0);
            CAT.subject_to(kappa_fr == 0);
            CAT.subject_to(-10 <= Mz_out <= 10);

            % longitudinal acceleration constraint     
            CAT.subject_to(ax_out == GGV.ax(i_));
    
            % solve
            plugin_opts = struct('print_time',0);
            solver_opts = struct('constr_viol_tol',0.1,'acceptable_obj_change_tol',0.001,'print_level',8);
            CAT.solver('ipopt',plugin_opts,solver_opts);
            sol = CAT.solve();
            % extract results
            GGV.ay(i_) = sol.value(ay_out);
            GGV.ax(i_) = sol.value(ax_out);

        end

    end

    GGV_out(i,:,1) = [velocity.*ones(2*ax_steps,1)]; % Vel
    GGV_out(i,:,2) = [(GGV.ax)'; (GGV.ax)']; % Ax
    GGV_out(i,:,3) = [(GGV.ay); -(GGV.ay)]; % Ay

end

%%

Ay = reshape(GGV_out(:,:,3),[], 1);
Ax = reshape(GGV_out(:,:,2),[], 1);
vel = reshape(GGV_out(:,:,1),[], 1);

GGVComplete = [vel, Ay, Ax];
GGVAcceleration = [vel(Ax>0), Ay(Ax>0), Ax(Ax>0)];
GGVDeceleration = [vel(Ax<0), Ay(Ax<0), Ax(Ax<0)];


%% run lap simulation 

outputs = runLapSim(GGVComplete, GGVAcceleration, GGVDeceleration, trackDistance, trackCurvature, sectorDistance);

clc;
stopTimer = toc(startTimer);
disp(['LTS Complete. Time taken: ', num2str(stopTimer), '(s)'])
%% post-process outputs

figure
t = tiledlayout(3,1);
title(t,'QSS Results')

nexttile
hold on
plot(outputs.dist,outputs.vCar)
ylabel('vCar (m/s)')
xlabel('sLap (m)')
grid on; grid minor; box on;
title(['Lap Time: ', num2str(outputs.time(end)), '(s)'])

nexttile
hold on
plot(outputs.dist,outputs.gLong,'r','DisplayName','Ax')
hold off
grid on; grid minor; box on;
ylabel('Ax (m/s^2)')
xlabel('sLap (m)')

nexttile
hold on
plot(outputs.dist,outputs.gLat,'b','DisplayName','Ay')
hold off
grid on; grid minor; box on;
ylabel('Ay (m/s^2)')
xlabel('sLap (m)')


% nexttile([2 2])
figure
grid on; grid minor; box on;
surf(GGV_out(:,:,3),GGV_out(:,:,2),GGV_out(:,:,1))
hold on
scatter3(outputs.gLat, outputs.gLong, outputs.vCar,'ro')
hold off
title('GGV Diagram')











