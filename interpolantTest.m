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
    cornerVelInterp = griddedInterpolant(flip(sort(LSP.ktMax,'descend')),flip(LSP.vel),'linear','linear');
    
    % Forward Velocity Profile
    posAyIdx = GGVresults.Ay >= 0 & GGVresults.Ax >= 0;   
    maxAccelerationInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx),'linear','linear');
    
    posAyIdx = GGVresults.Ax >= 0;   
    AccelerationDeltaInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.delta(posAyIdx),'linear','linear');
    AccelerationBeltaInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.beta(posAyIdx),'linear','linear');
    AccelerationYawRateInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.yaw_rate(posAyIdx),'linear','linear');
    AccelerationWheelRotFlInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.wheel_rot_fl(posAyIdx),'linear','linear');
    AccelerationWheelRotFRInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.wheel_rot_fr(posAyIdx),'linear','linear');
    AccelerationWheelRotRlInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.wheel_rot_rl(posAyIdx),'linear','linear');
    AccelerationWheelRotRRInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.wheel_rot_rr(posAyIdx),'linear','linear');
    
    % Reverse Velocity Profile
    posAyIdx = GGVresults.Ay >= 0 & GGVresults.Ax <= 0;    
    maxDecelerationInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx),'linear','linear');
    
    posAyIdx = GGVresults.Ax <= 0;
    DecelerationDeltaInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.delta(posAyIdx),'linear','linear');
    DecelerationBeltaInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.beta(posAyIdx),'linear','linear');
    DecelerationYawRateInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.yaw_rate(posAyIdx),'linear','linear');
    DecelerationWheelRotFlInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.wheel_rot_fl(posAyIdx),'linear','linear');
    DecelerationWheelRotFRInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.wheel_rot_fr(posAyIdx),'linear','linear');
    DecelerationWheelRotRlInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.wheel_rot_rl(posAyIdx),'linear','linear');
    DecelerationWheelRotRRInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.wheel_rot_rr(posAyIdx),'linear','linear');

    
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

    %%
    test = zeros(numel(finalAy),1);

    test(finalVel == forwardVel) =  AccelerationDeltaInterp(finalVel(finalVel==forwardVel),finalAy(finalVel==forwardVel),finalAx(finalVel==forwardVel));
    test(finalVel == maxCornerVel) =  AccelerationDeltaInterp(finalVel(finalVel == maxCornerVel),finalAy(finalVel == maxCornerVel),finalAx(finalVel == maxCornerVel));
    test(finalVel == brakeVel) =  DecelerationDeltaInterp(finalVel(finalVel == brakeVel),finalAy(finalVel == brakeVel),finalAx(finalVel == brakeVel));


    figure
    yyaxis left
    plot(test)
    yyaxis right
    plot(finalAy)

