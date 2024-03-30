clear; clc; 

addpath(genpath(pwd))
addpath('C:\Users\admin\Documents\CasAdi')

%% Load Track Parameterisation
trackData = struct;
trackData.sectorDistance = 1;
[trackData.trackDistance, trackData.trackCurvature] = preProccess.loadTrackModel('BicesterMotion_2023', trackData.sectorDistance);

%% Load Base Car Parametrisation

carData = preProccess.initVehicleModel();

%% run GGV sim to determine performance envelope

[GGV_surf, GGVresults] = simulate.runGGV(10, 10, carData);

%% run lap simulation 

outputs = simulate.runLapSim(GGVresults, trackData);
postProcess.plotResults(outputs,GGV_surf)

disp(['Lap Time: ', num2str(outputs.time(end)), '(s)'])

%% run replay to get vehicle states

replay = simulate.fnGGVReplay(300, outputs, GGVresults, trackData, carData);
postProcess.plotReplay(replay)


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










