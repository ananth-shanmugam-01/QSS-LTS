clear; clc; 

addpath(genpath('C:\Users\admin\Desktop\Git Repository\QSS-LTS-F3'))
addpath('C:\Users\admin\Documents\CasAdi')

%% Load Track Parameterisation
sectorDistance = 1;
[trackDistance, trackCurvature] = preProccess.loadTrackModel('FSUK_2016.mat', sectorDistance);

%% Load Base Car Parametrisation
carData = preProccess.initVehicleModel();

%% Single Lap Simulation

% run GGV Calculation
[GGV_surf, GGVComplete, GGVAcceleration, GGVDeceleration] = simulate.runGGV(10, 10, carData);

% run lap simulation 

outputs = simulate.runLapSim(carData, GGVComplete, GGVAcceleration, GGVDeceleration, trackDistance, trackCurvature, sectorDistance);

disp(['Lap Time: ', num2str(outputs.time(end)), '(s)'])

%% Parameter Updates / Sweeps

% sweepRange = [0.4, 0.45, 0.5, 0.55, 0.6];
% results = [];
% 
% for i = 1:numel(sweepRange)
% 
%     carData.Brakes.rBrakeBias = sweepRange(i);
% 
%     % run GGV Calculation
%     [GGV_surf, GGVComplete, GGVAcceleration, GGVDeceleration] = simulate.runGGV(10, 10, carData);
%     
%     % run lap simulation 
%     
%     outputs = simulate.runLapSim(carData, GGVComplete, GGVAcceleration, GGVDeceleration, trackDistance, trackCurvature, sectorDistance);
%     
%     disp(['Lap Time: ', num2str(outputs.time(end)), '(s)'])
% 
%     results(i,1) = sweepRange(i);
%     results(i,2) = outputs.time(end);
%     disp(['Sweep Point-', num2str(i),'/',num2str(numel(sweepRange)),' Completed'])
% 
% end

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
surf(GGV_surf(:,:,3),GGV_surf(:,:,2),GGV_surf(:,:,1))
hold on
scatter3(outputs.gLat, outputs.gLong, outputs.vCar,'ro')
hold off
title('GGV Diagram')











