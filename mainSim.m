clear; clc; 

addpath(genpath('C:\Users\admin\Desktop\Git Repository\QSS-LTS-F3'))
addpath('C:\Users\admin\Documents\CasAdi')

%% Create Track
sectorDistance = 1;
[trackDistance, trackCurvature] = preProccess.trackModel('track_data.txt', sectorDistance);

%% Initialise Vehicle Model
carData = preProccess.initVehicleModel();

%% run GGV Calc
% Unable to run in function?
    
velocityRange = linspace(10,carData.Powertrain.vMax-6,10);
clear GGV_out
GGV_out = [];

for i = 1:numel(velocityRange)

    velocity = velocityRange(i);
    % initialize GGV-data
    ax_steps = 10;  % number of discretization points
    GGV = struct();
    GGV.ax = zeros(ax_steps,1);
    GGV.ay = zeros(ax_steps,1);
    GGV.delta = zeros(ax_steps,1);
    GGV.beta = zeros(ax_steps,1);
    GGV.Power = zeros(ax_steps,1);
    
    
    % find Maximum Forward Acceleration
    nlp = []; vehicleModel;
    % objective
    nlp.minimize(-ax_out);
    
    % initialization of decision variables
    nlp.set_initial(delta,0);
    nlp.set_initial(beta,0);
    nlp.set_initial(yaw_rate,0);
    nlp.set_initial(throttle_position,1);
    nlp.set_initial(brake_pressure,0);
    nlp.set_initial(wheel_rot_fl,velocity/carData.Chassis.radWheel);
    nlp.set_initial(wheel_rot_fr,velocity/carData.Chassis.radWheel);
    nlp.set_initial(wheel_rot_rl,velocity/carData.Chassis.radWheel);
    nlp.set_initial(wheel_rot_rr,velocity/carData.Chassis.radWheel);
    
    % Constraints
    nlp.subject_to(ay_out == ay_control);
    nlp.subject_to(ax_out == ax_control);
    nlp.subject_to(ay_out == 0);
    nlp.subject_to(kappa_fl == 0);
    nlp.subject_to(kappa_fr == 0);
    
    % solve
    nlp.solver('ipopt');
    sol = nlp.solve();
    
    % extract results
    GGV.ay(1) = sol.value(ay_out);
    GGV.ax(1) = sol.value(ax_out);
    GGV.yaw_rate(1) = sol.value(yaw_rate);
    GGV.delta(1) = sol.value(delta);
    GGV.beta(1) = sol.value(delta);
    GGV.throttle_position(1) = sol.value(throttle_position);
    GGV.brake_pressure(1) = sol.value(brake_pressure);
    
    % find Maximum Braking Deceleration
    nlp = []; vehicleModel;
    % objective
    nlp.minimize(ax_out);
    
    % initialization of decision variables
    nlp.set_initial(delta,0);
    nlp.set_initial(beta,0);
    nlp.set_initial(yaw_rate,0);
    nlp.set_initial(throttle_position,0);
    nlp.set_initial(brake_pressure,0.4);
    nlp.set_initial(wheel_rot_fl,velocity/carData.Chassis.radWheel);
    nlp.set_initial(wheel_rot_fr,velocity/carData.Chassis.radWheel);
    nlp.set_initial(wheel_rot_rl,velocity/carData.Chassis.radWheel);
    nlp.set_initial(wheel_rot_rr,velocity/carData.Chassis.radWheel);
    
    % Constraints
    nlp.subject_to(ay_out == ay_control);
    nlp.subject_to(ax_out == ax_control);
    nlp.subject_to(ay_out == 0);
    % solve
    nlp.solver('ipopt');
    sol = nlp.solve();
    % extract results
    GGV.ay(end) = sol.value(ay_out);
    GGV.ax(end) = sol.value(ax_out);
    GGV.yaw_rate(end) = sol.value(yaw_rate);
    GGV.delta(end) = sol.value(delta);
    GGV.beta(end) = sol.value(delta);
    GGV.throttle_position(end) = sol.value(throttle_position);
    GGV.brake_pressure(end) = sol.value(brake_pressure);
    
    % Combined Acceleration
    GGV.ax = linspace(GGV.ax(1),GGV.ax(end),length(GGV.ax));
    
    for i_ = 2:length(GGV.ax)-1
        nlp = []; vehicleModel;
        % objective
        nlp.minimize(-ay_out);
        % initialization of decision variables
        nlp.set_initial(delta,2*pi/180);
        nlp.set_initial(beta,-1*pi/180);
        nlp.set_initial(yaw_rate,1);
        nlp.set_initial(throttle_position,0);
        nlp.set_initial(brake_pressure,0);
        nlp.set_initial(wheel_rot_fl,velocity/carData.Chassis.radWheel);
        nlp.set_initial(wheel_rot_fr,velocity/carData.Chassis.radWheel);
        nlp.set_initial(wheel_rot_rl,velocity/carData.Chassis.radWheel);
        nlp.set_initial(wheel_rot_rr,velocity/carData.Chassis.radWheel);
    
        % steady state constraints
        nlp.subject_to(ay_out == ay_control);
        nlp.subject_to(ax_out == ax_control);
        nlp.subject_to(Mz_out == 0);
        % longitudinal acceleration constraint
        nlp.subject_to(ax_out == GGV.ax(i_));
        % solve
        nlp.solver('ipopt');
        sol = nlp.solve();
        % extract results
        GGV.ay(i_) = sol.value(ay_out);
        GGV.ax(i_) = sol.value(ax_out);
        GGV.yaw_rate(i_) = sol.value(yaw_rate);
        GGV.delta(i_) = sol.value(delta);
        GGV.beta(i_) = sol.value(delta);
        GGV.throttle_position(i_) = sol.value(throttle_position);
        GGV.brake_pressure(i_) = sol.value(brake_pressure);
    end

    GGV_out(i,:,1) = [velocity.*ones(2*ax_steps,1)]; % Vel
    GGV_out(i,:,2) = [(GGV.ax)'; (GGV.ax)']; % Ax
    GGV_out(i,:,3) = [(GGV.ay); -(GGV.ay)]; % Ay
end

Ay = reshape(GGV_out(:,:,3),[], 1);
Ax = reshape(GGV_out(:,:,2),[], 1);
vel = reshape(GGV_out(:,:,1),[], 1);

GGVAcceleration = [vel(Ax>0), Ay(Ax>0), Ax(Ax>0)];
GGVDeceleration = [vel(Ax<0), Ay(Ax<0), Ax(Ax<0)];

%% run lap simulation 

outputs = runLapSim(GGVAcceleration, GGVDeceleration, trackDistance, trackCurvature, sectorDistance);


%% post-process outputs

figure
t = tiledlayout(4,2);
title(t,'QSS Results')

nexttile([2 1])
hold on
plot(outputs.dist,outputs.vCar)
ylabel('vCar (m/s)')
xlabel('sLap (m)')
grid on; grid minor; box on;
title(['Lap Time: ', num2str(outputs.time(end)), '(s)'])

nexttile([2 1])
hold on
plot(outputs.dist,outputs.gLong,'r','DisplayName','Ax')
plot(outputs.dist,outputs.gLat,'b','DisplayName','Ay')
hold off
grid on; grid minor; box on;
ylabel('Ax/y (m/s^2)')
xlabel('sLap (m)')

nexttile([2 2])
grid on; grid minor; box on;
surf(GGV_out(:,:,3),GGV_out(:,:,2),GGV_out(:,:,1))
hold on
scatter3(outputs.gLat, outputs.gLong, outputs.vCar,'ro')
hold off
title('GGV Diagram')











