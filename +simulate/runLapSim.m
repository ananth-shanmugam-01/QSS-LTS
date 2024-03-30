function outputs = runLapSim(GGVresults, trackData)

    startTimer = tic;
    
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
   
    % Reverse Velocity Profile
    posAyIdx = GGVresults.Ay >= 0 & GGVresults.Ax <= 0;    
    maxDecelerationInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx),'linear','linear');
   
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
    
    maxCornerVel = zeros(numel(trackData.trackCurvature),1);
    for i = 1:numel(trackData.trackCurvature)
        maxCornerVel(i) = min(29.038,cornerVelInterp(abs(trackData.trackCurvature(i))));
        maxCornerVel(i) = max(maxCornerVel(i),5);
    end
    
    %% Forward Speed Calculation
    
    % Identify Apices
    [val, locs] = findpeaks(-maxCornerVel,"MinPeakDistance",6);
    
    forwardVel = zeros(numel(trackData.trackCurvature),1);
    forwardVel(locs(val == max(val))) = maxCornerVel(locs(val == max(val)));
    
    for i = locs(val == max(val)):length(trackData.trackCurvature)-1
    
        curvature = trackData.trackCurvature(i);
        currentVel = forwardVel(i);
        currentAy = currentVel^2 * abs(curvature);
    
        Ax = max(0,maxAccelerationInterp(currentVel,currentAy));
        
        forwardVel(i+1) = min(maxCornerVel(i+1),abs(sqrt((currentVel^2) + 2*Ax*trackData.sectorDistance)));
    
    end
    
    forwardVel(1) = forwardVel(end); % Connect end of lap and start of lap
    
    for i = 1: locs(val == max(val))
        
        curvature = trackData.trackCurvature(i);
        currentVel = forwardVel(i);
        currentAy = currentVel^2 * abs(curvature);
    
        Ax = max(0,maxAccelerationInterp(currentVel,currentAy));
        
        forwardVel(i+1) = min(maxCornerVel(i+1),abs(sqrt((currentVel^2) + 2*Ax*trackData.sectorDistance)));
    
    end 
    
    %% Braking Speed Calculation
    brakeVel = zeros(length(trackData.trackCurvature),1);
    
    brakeVel(locs(end)) = maxCornerVel(locs(end));
    
    for i = locs(end):-1:2
    
        curvature = trackData.trackCurvature(i);
        currentVel = brakeVel(i);
        currentAy = currentVel^2 * abs(curvature);
    
        Ax = min(0,maxDecelerationInterp(currentVel,currentAy)); % Protection against surface extrapolation to negative velocities
        
        brakeVel(i-1) = min(maxCornerVel(i-1),abs(sqrt((currentVel^2) - 2*Ax*trackData.sectorDistance)));
    
    end
    
    for i = locs(end):length(trackData.trackDistance)
        brakeVel(i) = maxCornerVel(i); 
    end 
    
    %% Final Velocity Profile
    
    finalVel = min([maxCornerVel';forwardVel';brakeVel'])';
    lapTime = sum(trackData.sectorDistance./finalVel);
    
    finalAx = zeros(length(trackData.trackDistance),1);
    
    for i = 1:length(trackData.trackDistance)-1
        finalAx(i) = (finalVel(i+1)^2 - finalVel(i)^2)/(2*trackData.sectorDistance);
    end
    finalAx(length(finalVel)) = (finalVel(2)^2 - finalVel(length(finalVel))^2)/(2*trackData.sectorDistance);
    
    finalAy = trackData.trackCurvature'.*finalVel.^2;
    
    outputs = struct;
    outputs.time = cumsum(trackData.sectorDistance./finalVel);
    outputs.dist = trackData.trackDistance';
    outputs.vCar = finalVel;
    outputs.gLat = finalAy;
    outputs.gLong = finalAx;
    outputs.yawRate = finalAy./finalVel;
    outputs.velocityProfiles.forwardVel = forwardVel;
    outputs.velocityProfiles.boundaryVel = maxCornerVel;
    outputs.velocityProfiles.reverseVel = brakeVel;


    outputs.interpolants.cornerVelInterp = cornerVelInterp;
    outputs.interpolants.maxAccelerationInterp = maxAccelerationInterp;
    outputs.interpolants.maxDecelerationInterp = maxDecelerationInterp;

    outputs.track.sLap = trackData.trackDistance;
    outputs.track.curvature = trackData.trackCurvature;


    stopTimer = toc(startTimer);

    disp(['Lap Time Simulation Complete. Time taken: ', num2str(stopTimer), '(s)'])

end

