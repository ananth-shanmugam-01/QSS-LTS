function outputs = runLapSim(GGVComplete, GGVAcceleration, GGVDeceleration, trackDistance, trackCurvature, sectorDistance)

    tic
    
    % LSP Calculations
    posAyIdx = find(GGVAcceleration(:,2) >= 0);
    
    maxAccelerationGGV= struct;
    maxAccelerationGGV.kt = GGVAcceleration(posAyIdx,2)./(GGVAcceleration(posAyIdx,1).^2);
    maxAccelerationGGV.Vel = GGVAcceleration(posAyIdx,1);
    maxAccelerationGGV.Ax = GGVAcceleration(posAyIdx,3);
    maxAccelerationGGV.Ay = GGVAcceleration(posAyIdx,2);
    
    velRange = unique(GGVComplete(:,1));
    for i = 1:numel(velRange)
        idx = find(GGVComplete(:,1) == velRange(i));
        kt = GGVComplete(idx,2)./(GGVComplete(idx,1).^2);
        LSP.ktMax(i) = max(kt);
        LSP.vel(i) = velRange(i);
    end
    
    ktInterp = linspace(min(LSP.ktMax),max(LSP.ktMax),10000);
    cornerVelInterp = griddedInterpolant(flip(sort(LSP.ktMax,'descend')),flip(LSP.vel),'linear','linear');
    
    maxAccelerationInterp = scatteredInterpolant(maxAccelerationGGV.Vel, maxAccelerationGGV.Ay, maxAccelerationGGV.Ax,'linear','nearest');
    
    posAyIdx = find(GGVDeceleration(:,2) >= 0);
    maxDecelerationGGV= struct;
    maxDecelerationGGV.Vel = GGVDeceleration(posAyIdx,1);
    maxDecelerationGGV.Ax = GGVDeceleration(posAyIdx,3);
    maxDecelerationGGV.Ay = GGVDeceleration(posAyIdx,2);
    
    maxDecelerationInterp = scatteredInterpolant(maxDecelerationGGV.Vel, maxDecelerationGGV.Ay, maxDecelerationGGV.Ax,'linear','linear');
    
    %% Interpolation Testing
    
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
    
    %% Limit Speed Calculation
    
    maxCornerVel = zeros(numel(trackCurvature),1);
    for i = 1:numel(trackCurvature)
        maxCornerVel(i) = min(29.038,cornerVelInterp(abs(trackCurvature(i))));
        maxCornerVel(i) = max(maxCornerVel(i),5);
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
    finalAx(length(finalVel)) = (finalVel(2)^2 - finalVel(length(finalVel))^2)/(2*sectorDistance);
    
    finalAy = trackCurvature'.*finalVel.^2;
    
    outputs = struct;
    outputs.time = cumsum(sectorDistance./finalVel);
    outputs.dist = trackDistance';
    outputs.vCar = finalVel;
    outputs.gLat = finalAy;
    outputs.gLong = finalAx;

end

