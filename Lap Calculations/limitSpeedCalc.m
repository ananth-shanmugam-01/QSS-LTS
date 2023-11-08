% Limit Speed Function

function velocityLSP = limitSpeedCalc(OptInputs, trackCurvature, GGVSurfAy)

    % Control Inputs
    
    Vx = OptInputs(1);
    Ax = OptInputs(2);

    Ay = (Vx^2)*abs(trackCurvature);

    AyGGV = griddata(GGVSurfAy.Vel, GGVSurfAy.Ax, GGVSurfAy.Ay,Vx,0,'cubic');
    % AxGGV = griddata(GGVSurfAy.Vel, GGVSurfAy.Ay, GGVSurfAy.Ax,Vx,Ay,'cubic');

    if isnan(AyGGV)
        AyGGV = 0;
    else
        AyGGV;
    end
    
    velocityLSP = -Vx;

end