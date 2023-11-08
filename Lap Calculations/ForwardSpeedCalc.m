
function OptOut = ForwardSpeedCalc(OptInputs, GGVSurfAy, trackCurvature, currentVel, sectorDistance, nextLSP)

    % Control Inputs
    
    Ax = OptInputs(1);
    Ay = abs((currentVel^2) * trackCurvature);
    
    AyGGV = griddata(GGVSurfAy.Vel, GGVSurfAy.Ax, GGVSurfAy.Ay,currentVel,Ax,'cubic');
    % AxGGV = griddata(GGVSurfAy.Vel, GGVSurfAy.Ay, GGVSurfAy.Ax,currentVel,Ay,'cubic');
    
%     AyGGV = GGVSurfAy(currentVel, Ax);
%     AxGGV = GGVSurfAx(currentVel, Ay);

    if isnan(AyGGV)
        AyGGV = 0;
    end

%     if isnan(AxGGV)
%         AxGGV = 0;
%     end 
    
    NextVel = abs(sqrt(currentVel^2 + 2*Ax*sectorDistance));

    if NextVel > nextLSP
        NextVel = nextLSP;
    end

    OptOut = -NextVel;

end