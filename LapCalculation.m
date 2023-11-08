clear; clc; warning off

load("GGVForward.mat");
load('GGVBrake.mat');
load("GGVsurf.mat");

addpath("Data Files\")
addpath("Lap Calculations\")
%%

tic

sectorDistance = 1;

[trackDistance, trackCurvature] = trackModel('track_data.txt', sectorDistance);

% GGVAcc = [GGVAccVel, GGVAccAy, GGVAccAx];

% LSP Calculations
posAyIdx = find(GGVAcc(:,2) >= 0);

InterpAy = scatteredInterpolant(GGVAcc(posAyIdx,1), GGVAcc(posAyIdx,3), GGVAcc(posAyIdx,2),"linear",'none'); % Vel, Ax, Ay

GGVSurfAy= struct;
GGVSurfAy.Vel = GGVAcc(posAyIdx,1);
GGVSurfAy.Ay = GGVAcc(posAyIdx,2);
GGVSurfAy.Ax = GGVAcc(posAyIdx,3);

posAyIdx = find(GGVBrake(:,2) >= 0);

GGVBrakeSurf = struct;
GGVBrakeSurf.Vel = GGVBrake(posAyIdx,1);
GGVBrakeSurf.Ay = GGVBrake(posAyIdx,2);
GGVBrakeSurf.Ax = GGVBrake(posAyIdx,3);

%% Limit Speed Profile

limitSpeedVel = zeros(numel(trackCurvature),1);

for i = 1:numel(trackCurvature)

    curvature = trackCurvature(i);

    x0 = [10, 1];
    Aeq = [];
    beq = [];
    lb = [0, 0];
    ub = [];
    [control_inputs, fnval, exitflag] = fmincon(@(x) limitSpeedCalc(x, curvature, GGVSurfAy),x0,[],[],Aeq,beq,lb,ub,@(x) nonLinearConstraintslimitSpeedCalc(x, curvature, GGVSurfAy)); 
    limitSpeedVel(i) = -fnval;

end

%% Maximum Forward Vel

% Identify Apices
[val, locs] = findpeaks(-limitSpeedVel,"MinPeakDistance",6);

% Start Acceleration from the slowest apex, this way the speed traces will align
forwardVel = zeros(numel(trackCurvature),1);

forwardVel(locs(val == max(val))) = limitSpeedVel(locs(val == max(val)));

options = optimoptions('fmincon','Algorithm','sqp','MaxFunctionEvaluations',3000,'ConstraintTolerance',0.01);

for i = locs(val == max(val)):length(trackCurvature)-1

    curvature = trackCurvature(i);
    currentVel = forwardVel(i);
    nextLSP = limitSpeedVel(i+1);

    % Maximise Ax
    % Current Velocity - Fixed (from previous calculation

    x0 = [1];
    Aeq = [];
    beq = [];
    lb = [0];
    ub = [5];
    [control_inputs, fnval, exitflag] = fmincon(@(x) ForwardSpeedCalc(x, GGVSurfAy,curvature, currentVel,sectorDistance,nextLSP),x0,[],[],Aeq,beq,lb,ub,...
                                                @(x) nonLinearConstraintsForwardSpeedCalc(x, GGVSurfAy, curvature,currentVel,sectorDistance,nextLSP),options); 
    
    forwardVel(i+1) = -fnval;
    
end

forwardVel(1) = forwardVel(end); % Connect end of lap and start of lap

for i = 1: locs(val == max(val))

    curvature = trackCurvature(i);
    currentVel = forwardVel(i);
    nextLSP = limitSpeedVel(i+1);

    % Maximise Ax
    % Control Inputs Vel(x)

    x0 = [1];
    Aeq = [];
    beq = [];
    lb = [0];
    ub = [5];
    [control_inputs, fnval, exitflag] = fmincon(@(x) ForwardSpeedCalc(x, GGVSurfAy,curvature, currentVel,sectorDistance,nextLSP),x0,[],[],Aeq,beq,lb,ub,...
                                                @(x) nonLinearConstraintsForwardSpeedCalc(x, GGVSurfAy, curvature,currentVel,sectorDistance,nextLSP),options); 
    forwardVel(i+1) = -fnval;

end 

%% Braking Speed Profile

% Determine Velocity at first node
brakeVel = zeros(length(trackCurvature),1);

brakeVel(locs(end)) = limitSpeedVel(locs(end));

for i = locs(end)-1:-1:1

    curvature = trackCurvature(i);
    previousVel = brakeVel(i+1);
    currentLSP = limitSpeedVel(i);

    % Maximise Ax
    % Control Inputs Vel(x)

    maxAx = griddata(GGVBrakeSurf.Vel, GGVBrakeSurf.Ay, GGVBrakeSurf.Ax,currentLSP,0,'cubic');

    x0 = [10,-1];
    Aeq = [];
    beq = [];
    lb = [0,maxAx];
    ub = [currentLSP, 0];
    [control_inputs, fnval, exitflag] = fmincon(@(x) BrakingSpeedCalc(x, GGVBrakeSurf,curvature, previousVel,sectorDistance,currentLSP),x0,[],[],Aeq,beq,lb,ub,...
                                                @(x) nonLinearConstraintsBrakingSpeedCalc(x, GGVBrakeSurf,curvature, previousVel,sectorDistance,currentLSP),options); 
       
    brakeVel(i) = -fnval;

end

for i = locs(end):length(trackDistance)
    brakeVel(i) = limitSpeedVel(i); % Prevent these inputs for affecting array
end 

%% Velocity Profiles

% figure
% hold on
% plot(trackDistance,limitSpeedVel)
% plot(trackDistance,forwardVel)
% plot(trackDistance,brakeVel)
% plot(trackDistance(locs),limitSpeedVel(locs),'ro')
% hold off


%%  Final Velocity Profile

finalVel = min([limitSpeedVel';forwardVel';brakeVel'])';
lapTime = sum(sectorDistance./finalVel);

finalAx = zeros(length(trackDistance),1);

for i = 1:length(trackDistance)-1
    finalAx(i) = (finalVel(i+1)^2 - finalVel(i)^2)/(2*sectorDistance);
end
finalAx(length(finalVel)) = (finalVel(1)^2 - finalVel(length(finalVel))^2)/(2*sectorDistance);

finalAy = trackCurvature'.*finalVel.^2;

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



