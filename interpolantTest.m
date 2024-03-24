    clear; clc;    
% Load Track Parameterisation
    sectorDistance = 1;
    [trackDistance, trackCurvature] = preProccess.loadTrackModel('FSUK_2016.mat', sectorDistance);

    load("GGVresults.mat")

    % LSP Calculations    
    velRange = unique(GGVresults.vel);
    for i = 1:numel(velRange)
        idx = find(GGVresults.vel == velRange(i));
        kt = GGVresults.Ay(idx)./(GGVresults.vel(idx).^2);
        LSP.ktMax(i) = max(kt);
        LSP.vel(i) = velRange(i);
    end
    
    ktInterp = linspace(min(LSP.ktMax),max(LSP.ktMax),100);
    cornerVelInterp = griddedInterpolant(flip(sort(LSP.ktMax,'descend')),flip(LSP.vel),'linear');
    
    % Forward Velocity Profile
    posAyIdx = GGVresults.Ay >= 0 & GGVresults.Ax >= 0;   
    maxAccelerationInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx),'natural','linear');
    
    posAyIdx = GGVresults.Ax >= 0;   
    AccelerationDeltaInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.delta(posAyIdx),'linear');
    AccelerationBeltaInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.beta(posAyIdx),'linear');
    AccelerationYawRateInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.yaw_rate(posAyIdx),'linear');
    AccelerationWheelRotFlInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.wheel_rot_fl(posAyIdx),'linear');
    AccelerationWheelRotFRInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.wheel_rot_fr(posAyIdx),'linear');
    AccelerationWheelRotRlInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.wheel_rot_rl(posAyIdx),'linear');
    AccelerationWheelRotRRInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.wheel_rot_rr(posAyIdx),'linear');
    
    % Reverse Velocity Profile
    posAyIdx = GGVresults.Ay >= 0 & GGVresults.Ax <= 0;    
    maxDecelerationInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx),'linear');
    
    posAyIdx = GGVresults.Ax <= 0;
    DecelerationDeltaInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.delta(posAyIdx),'linear');
    DecelerationBeltaInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.beta(posAyIdx),'linear');
    DecelerationYawRateInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.yaw_rate(posAyIdx),'linear');
    DecelerationWheelRotFlInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.wheel_rot_fl(posAyIdx),'linear');
    DecelerationWheelRotFRInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.wheel_rot_fr(posAyIdx),'linear');
    DecelerationWheelRotRlInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.wheel_rot_rl(posAyIdx),'linear');
    DecelerationWheelRotRRInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.wheel_rot_rr(posAyIdx),'linear');

    
    % Interpolation Testing
    
    range = 0.3:-0.001:0.001;
    velInterp = zeros(numel(range),1);
    
    for i = 1:numel(range)
        velInterp(i) = cornerVelInterp(range(i));
    end
    
%     figure
%     hold on
%     plot(LSP.ktMax,LSP.vel)
%     plot(range, velInterp)
%     hold off
    
    % Limit Speed Calculation
    
    maxCornerVel = zeros(numel(trackCurvature),1);
    for i = 1:numel(trackCurvature)
        maxCornerVel(i) = min(29.038,cornerVelInterp(abs(trackCurvature(i))));
        maxCornerVel(i) = max(maxCornerVel(i),5);
    end
    
    % Forward Speed Calculation
    
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
    
    % Braking Speed Calculation
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
    
    % Final Velocity Profile
    
    finalVel = min([maxCornerVel';forwardVel';brakeVel'])';
    lapTime = sum(sectorDistance./finalVel);
    
    finalAx = zeros(length(trackDistance),1);
    
    for i = 1:length(trackDistance)-1
        finalAx(i) = (finalVel(i+1)^2 - finalVel(i)^2)/(2*sectorDistance);
    end
    finalAx(length(finalVel)) = (finalVel(2)^2 - finalVel(length(finalVel))^2)/(2*sectorDistance);
    
    finalAy = trackCurvature'.*finalVel.^2;
    
    outputs = struct;
    outputs.time = cumsum(sectorDistance./finalVel);
    outputs.dist = trackDistance';
    outputs.vCar = finalVel;
    outputs.gLat = finalAy;
    outputs.gLong = finalAx;

%% Low Pass Filter

d1 = designfilt("lowpassiir",FilterOrder=4,HalfPowerFrequency=0.1,DesignMethod="butter");
filtAx = filtfilt(d1, finalAx);

%% Interpolate and Save to Struct

GGVinterp = struct;

GGVinterp.vCar = zeros(numel(finalVel),1);
GGVinterp.Ay = zeros(numel(finalVel),1);
GGVinterp.Ax = zeros(numel(finalVel),1); 
GGVinterp.delta = zeros(numel(finalVel),1);
GGVinterp.beta = zeros(numel(finalVel),1);
GGVinterp.yaw_rate = zeros(numel(finalVel),1);
GGVinterp.wheel_rot_fl = zeros(numel(finalVel),1);
GGVinterp.wheel_rot_fr = zeros(numel(finalVel),1);
GGVinterp.wheel_rot_rl = zeros(numel(finalVel),1);
GGVinterp.wheel_rot_rr = zeros(numel(finalVel),1);


i = find(finalVel == forwardVel);
GGVinterp.vCar(i)       = finalVel(i);
GGVinterp.Ay(i)         = finalAy(i);
GGVinterp.Ax(i)         = filtAx(i); 
GGVinterp.delta(i)      = filtfilt(d1, AccelerationDeltaInterp(finalVel(i),finalAy(i), filtAx(i)));
GGVinterp.beta(i)       = filtfilt(d1, AccelerationBeltaInterp(finalVel(i),finalAy(i), filtAx(i)));
GGVinterp.yaw_rate(i)   = AccelerationYawRateInterp(finalVel(i),finalAy(i), filtAx(i));
GGVinterp.wheel_rot_fl(i) = AccelerationWheelRotFlInterp(finalVel(i),finalAy(i), filtAx(i));
GGVinterp.wheel_rot_fr(i) = AccelerationWheelRotFRInterp(finalVel(i),finalAy(i), filtAx(i));
GGVinterp.wheel_rot_rl(i) = AccelerationWheelRotRlInterp(finalVel(i),finalAy(i), filtAx(i));
GGVinterp.wheel_rot_rr(i) = AccelerationWheelRotRRInterp(finalVel(i),finalAy(i), filtAx(i));

j = find(finalVel == maxCornerVel);
GGVinterp.vCar(j)       = finalVel(j);
GGVinterp.Ay(j)         = finalAy(j);
GGVinterp.Ax(j)         = filtAx(j); 
GGVinterp.delta(j)      = filtfilt(d1, AccelerationDeltaInterp(finalVel(j),finalAy(j), filtAx(j)));
GGVinterp.beta(j)       = filtfilt(d1, AccelerationBeltaInterp(finalVel(j),finalAy(j), filtAx(j)));
GGVinterp.yaw_rate(j)   = AccelerationYawRateInterp(finalVel(j),finalAy(j), filtAx(j));
GGVinterp.wheel_rot_fl(j) = AccelerationWheelRotFlInterp(finalVel(j),finalAy(j), filtAx(j));
GGVinterp.wheel_rot_fr(j) = AccelerationWheelRotFRInterp(finalVel(j),finalAy(j), filtAx(j));
GGVinterp.wheel_rot_rl(j) = AccelerationWheelRotRlInterp(finalVel(j),finalAy(j), filtAx(j));
GGVinterp.wheel_rot_rr(j) = AccelerationWheelRotRRInterp(finalVel(j),finalAy(j), filtAx(j));

k = find(finalVel == brakeVel);
GGVinterp.vCar(k)       = finalVel(k);
GGVinterp.Ay(k)         = finalAy(k);
GGVinterp.Ax(k)         = filtAx(k); 
GGVinterp.delta(k)      = filtfilt(d1, DecelerationDeltaInterp(finalVel(k),finalAy(k), filtAx(k)));
GGVinterp.beta(k)       = filtfilt(d1, DecelerationBeltaInterp(finalVel(k),finalAy(k), filtAx(k)));
GGVinterp.yaw_rate(k)   = DecelerationYawRateInterp(finalVel(k),finalAy(k), filtAx(k));
GGVinterp.wheel_rot_fl(k) = DecelerationWheelRotFlInterp(finalVel(k),finalAy(k), filtAx(k));
GGVinterp.wheel_rot_fr(k) = DecelerationWheelRotFRInterp(finalVel(k),finalAy(k), filtAx(k));
GGVinterp.wheel_rot_rl(k) = DecelerationWheelRotRlInterp(finalVel(k),finalAy(k), filtAx(k));
GGVinterp.wheel_rot_rr(k) = DecelerationWheelRotRRInterp(finalVel(k),finalAy(k), filtAx(k));

GGVinterp.time = outputs.time;
GGVinterp.dist = outputs.dist;
GGVinterp.GGVresults = outputs;

% Further Smoothen controls
GGVinterp.delta = csaps(outputs.dist,GGVinterp.delta,0.3,outputs.dist);
GGVinterp.beta = csaps(outputs.dist,GGVinterp.beta,0.3,outputs.dist);


%%
fields = fieldnames(GGVinterp);
figure; tiledlayout(numel(fields),1);

for i = 1:numel(fields)
    if ~isstruct(GGVinterp.(fields{i}))
    nexttile
    plot(outputs.dist, GGVinterp.(fields{i}));
    end
      

end
