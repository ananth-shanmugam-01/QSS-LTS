
function OptOut = BrakingSpeedCalc(OptInputs, GGVSurfAy, trackCurvature, previousVel, sectorDistance, currentLSP)

    % Control Inputs
    
    Vx = OptInputs(1); % Velocity Iteration for current  point
    Ax = OptInputs(2); % Acceleration (deceleration) capability at current point
    Ay = abs((Vx^2) * trackCurvature); % Check to ensure that cornering capacity is sufficient for curvature
    
    AyGGV = griddata(GGVSurfAy.Vel, GGVSurfAy.Ax, GGVSurfAy.Ay,Vx,Ax,'cubic');

    if isnan(AyGGV)
        AyGGV = 0;
    end
    
    Vx = abs(sqrt(previousVel^2 - 2*(Ax)*sectorDistance));

    if Vx > currentLSP
        Vx = currentLSP; % nextLSP refers to LSP at the 
    end

    OptOut = -Vx;

end