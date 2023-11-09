clear; clc; warning off

load("GGVForward.mat");
load('GGVBrake.mat');
load("GGVsurf.mat");

addpath("Data Files\")
addpath("Lap Calculations\")
%%

tic

% LSP Calculations
posAyIdx = find(GGVAcc(:,2) >= 0);

maxAccelerationGGV= struct;
maxAccelerationGGV.kt = GGVAcc(posAyIdx,2)./(GGVAcc(posAyIdx,1).^2);
maxAccelerationGGV.Vel = GGVAcc(posAyIdx,1);
maxAccelerationGGV.Ax = GGVAcc(posAyIdx,3);
maxAccelerationGGV.Ay = GGVAcc(posAyIdx,2);

velRange = unique(maxAccelerationGGV.Vel);
for i = 1:numel(velRange)
    idx = find(maxAccelerationGGV.Vel == velRange(i));
    LSP.ktMax(i) = max(maxAccelerationGGV.kt(idx));
    LSP.vel(i) = velRange(i);
end

ktInterp = linspace(min(LSP.ktMax),max(LSP.ktMax),10000);
cornerVelInterp = griddedInterpolant(flip(LSP.ktMax),flip(LSP.vel),'linear','linear');

maxAccelerationInterp = scatteredInterpolant(maxAccelerationGGV.Vel, maxAccelerationGGV.Ay, maxAccelerationGGV.Ax,'linear','linear');

posAyIdx = find(GGVBrake(:,2) >= 0);
maxDecelerationGGV= struct;
maxDecelerationGGV.Vel = GGVBrake(posAyIdx,1);
maxDecelerationGGV.Ax = GGVBrake(posAyIdx,3);
maxDecelerationGGV.Ay = GGVBrake(posAyIdx,2);

maxDecelerationInterp = scatteredInterpolant(maxDecelerationGGV.Vel, maxDecelerationGGV.Ay, maxDecelerationGGV.Ax,'linear','linear');

%% Interpolation Testing

% range = 0.3:-0.001:0.001;
% velInterp = zeros(numel(range),1);
% 
% for i = 1:numel(range)
%     velInterp(i) = cornerVelInterp(range(i));
% end
% 
% figure
% hold on
% plot(LSP.ktMax,LSP.vel)
% plot(range, velInterp)
% hold off
    
%% Track Creation
sectorDistance = 1;
[trackDistance, trackCurvature] = trackModel('track_data.txt', sectorDistance);

%% Limit Speed Calculation

maxCornerVel = zeros(numel(trackCurvature),1);
for i = 1:numel(trackCurvature)
    maxCornerVel(i) = min(29.038,cornerVelInterp(abs(trackCurvature(i))));
end

%% Forward Speed Calculation

% Identify Apices
[val, locs] = findpeaks(-maxCornerVel,"MinPeakDistance",6);

forwardVel = zeros(numel(trackCurvature),1);
forwardVel(locs(val == max(val))) = maxCornerVel(locs(val == max(val)));

for i = locs(val == max(val)):length(trackCurvature)-1

    curvature = trackCurvature(i);
    currentVel = forwardVel(i);
    currentAy = currentVel^2 * abs(curvature);

    Ax = max(0,maxAccelerationInterp(currentVel,currentAy));
    
    forwardVel(i+1) = min(maxCornerVel(i+1),abs(sqrt((currentVel^2) + 2*Ax*sectorDistance)));

end

forwardVel(1) = forwardVel(end); % Connect end of lap and start of lap

for i = 1: locs(val == max(val))
    
    curvature = trackCurvature(i);
    currentVel = forwardVel(i);
    currentAy = currentVel^2 * abs(curvature);

    Ax = max(0,maxAccelerationInterp(currentVel,currentAy));
    
    forwardVel(i+1) = min(maxCornerVel(i+1),abs(sqrt((currentVel^2) + 2*Ax*sectorDistance)));

end 

%% Braking Speed Calculation
brakeVel = zeros(length(trackCurvature),1);

brakeVel(locs(end)) = maxCornerVel(locs(end));

for i = locs(end):-1:2

    curvature = trackCurvature(i);
    currentVel = brakeVel(i);
    currentAy = currentVel^2 * abs(curvature);

    Ax = min(0,maxDecelerationInterp(currentVel,currentAy)); % Incorrect simplification
    
    brakeVel(i-1) = min(maxCornerVel(i-1),abs(sqrt((currentVel^2) - 2*Ax*sectorDistance)));

end

for i = locs(end):length(trackDistance)
    brakeVel(i) = maxCornerVel(i); 
end 

%% Final Velocity Profile

finalVel = min([maxCornerVel';forwardVel';brakeVel'])';
lapTime = sum(sectorDistance./finalVel);

finalAx = zeros(length(trackDistance),1);

for i = 1:length(trackDistance)-1
    finalAx(i) = (finalVel(i+1)^2 - finalVel(i)^2)/(2*sectorDistance);
end
finalAx(length(finalVel)) = (finalVel(1)^2 - finalVel(length(finalVel))^2)/(2*sectorDistance);

finalAy = trackCurvature'.*finalVel.^2;

%% Plots

toc

figure
tiledlayout(2,1)

nexttile
plot(trackDistance,finalVel,'k','LineWidth',2)
ylabel('V (m/s)')
xlabel('sLap (m)')
title(['Lap Time: ', num2str(lapTime), '(s)'])

nexttile
hold on
plot(trackDistance,finalAx,'r','LineWidth',2,'DisplayName','Ax')
plot(trackDistance,finalAy,'b','LineWidth',2,'DisplayName','Ay')
hold off
ylabel('Ax/y (m/s^2)')
xlabel('sLap (m)')


