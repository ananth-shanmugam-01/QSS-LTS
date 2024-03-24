clear; clc; 

addpath(genpath(pwd))
addpath('C:\Users\admin\Documents\CasAdi')

%% Load Track Parameterisation
trackData = struct;
trackData.sectorDistance = 1;
[trackData.trackDistance, trackData.trackCurvature] = preProccess.loadTrackModel('FSUK_2016.mat', trackData.sectorDistance);

%% Load Base Car Parametrisation

carData = preProccess.initVehicleModel();

%% run GGV sim to determine performance envelope

[GGV_surf, GGVresults] = simulate.runGGV(10, 10, carData);

%% run lap simulation 

outputs = simulate.runLapSim(GGVresults, trackData);

disp(['Lap Time: ', num2str(outputs.time(end)), '(s)'])

%% run replay to get vehicle states

replay = simulate.fnGGVReplay(300, outputs, GGVresults, trackData, carData);

%% post-process outputs

% figure
% t = tiledlayout(3,1);
% title(t,'QSS Results')
% 
% nexttile
% hold on
% plot(outputs.dist,outputs.vCar)
% ylabel('vCar (m/s)')
% xlabel('sLap (m)')
% grid on; grid minor; box on;
% title(['Lap Time: ', num2str(outputs.time(end)), '(s)'])
% 
% nexttile
% hold on
% plot(outputs.dist,outputs.gLong,'r','DisplayName','Ax')
% hold off
% grid on; grid minor; box on;
% ylabel('Ax (m/s^2)')
% xlabel('sLap (m)')
% 
% nexttile
% hold on
% plot(outputs.dist,outputs.gLat,'b','DisplayName','Ay')
% hold off
% grid on; grid minor; box on;
% ylabel('Ay (m/s^2)')
% xlabel('sLap (m)')
% 
% figure
% grid on; grid minor; box on;
% surf(GGV_surf(:,:,3),GGV_surf(:,:,2),GGV_surf(:,:,1))
% hold on
% scatter3(outputs.gLat, outputs.gLong, outputs.vCar,'ro')
% hold off
% title('GGV Diagram')
% 
% figure(2)
% grid on; grid minor; box on;
% surf(GGV_surf(:,:,3),GGV_surf(:,:,2),GGV_surf(:,:,1),'FaceColor','none','CData',41)
% hold on
% scatter3(outputs.gLat(683:770), outputs.gLong(683:770), outputs.vCar(683:770),[],outputs.time(683:770),'o','filled')
% colorbar
% hold off
% title('GGV Diagram')

%% Parameter Updates / Sweeps

% sweepRange = [0.20, 0.24, 0.28, 0.32, 0.36];
% results = [];
% 
% for i = 1:numel(sweepRange)
% 
%     carData.Chassis.heightSprungCOG = sweepRange(i);
% 
%     % run GGV Calculation
%     [GGV_surf, GGVresults] = simulate.runGGV(10, 10, carData);
%     
%     % run lap simulation 
%     
%     outputs = simulate.runLapSim(GGVresults, trackDistance, trackCurvature, sectorDistance);
%     
%     disp(['Lap Time: ', num2str(outputs.time(end)), '(s)'])
% 
%     results(i,1) = sweepRange(i);
%     results(i,2) = outputs.time(end);
%     disp(['Sweep Point-', num2str(i),'/',num2str(numel(sweepRange)),' Completed'])
% 
% end










