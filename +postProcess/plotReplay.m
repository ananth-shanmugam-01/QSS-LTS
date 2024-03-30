function plotReplay(replay)

    % function to create post-processed plots
    
    figure(Name='States'); clf; t = tiledlayout(4,1);
    title(t, 'QSS LTS');
    subtitle(t, ['Lap Time: ', num2str(round(replay.time(end),2)), '(s)'])
    nexttile
    plot(replay.sLap, replay.vCar)
    title('vCar')
    ylabel('[m/s]')
    grid minor
    
    nexttile
    plot(replay.sLap, replay.gLat)
    title('gLat')
    ylabel('[m/s^2]')
    grid minor
    
    nexttile
    plot(replay.sLap, replay.gLong)
    title('gLong')
    ylabel('[m/s^2]')
    grid minor
    
    nexttile
    plot(replay.sLap, replay.yawRate)
    title('yawRate')
    ylabel('[rad/s]')
    grid minor
    
    figure(Name='Controls'); clf; t = tiledlayout(3,1);
    title(t, 'QSS LTS');
    subtitle(t, ['Lap Time: ', num2str(round(replay.time(end),2)), '(s)'])
    nexttile
    plot(replay.sLap, replay.vCar)
    title('vCar')
    ylabel('[m/s]')
    grid minor
    
    nexttile
    plot(replay.sLap, rad2deg(replay.aSteer))
    title('aSteer')
    ylabel('[deg]')
    grid minor
    
    nexttile
    yyaxis left
    plot(replay.sLap, replay.rThrottle)
    title('rThrottle')
    ylabel('[TP]')
    yyaxis right
    plot(replay.sLap, replay.pBrake)
    ylabel('pBrake')
    ylabel('[bar]')
    grid minor
    
    figure(Name='Auxiliary Outputs'); clf; t = tiledlayout(6,1);
    title(t, 'QSS LTS');
    subtitle(t, ['Lap Time: ', num2str(round(replay.time(end),2)), '(s)'])
    nexttile
    plot(replay.sLap, replay.vCar)
    title('vCar')
    ylabel('[m/s]')
    grid minor
    
    nexttile
    hold on
    plot(replay.sLap, rad2deg(replay.aSlipAngleFL),'DisplayName','FL')
    plot(replay.sLap, rad2deg(replay.aSlipAngleFR),'DisplayName','FR')
    plot(replay.sLap, rad2deg(replay.aSlipAngleRL),'DisplayName','RL')
    plot(replay.sLap, rad2deg(replay.aSlipAngleRR),'DisplayName','RR')
    hold off; legend
    title('Tyre Slip Angle')
    ylabel('[deg]')
    grid minor
    
    nexttile
    hold on
    plot(replay.sLap, replay.aSlipRatioFL,'DisplayName','FL')
    plot(replay.sLap, replay.aSlipRatioFR,'DisplayName','FR')
    plot(replay.sLap, replay.aSlipRatioRL,'DisplayName','RL')
    plot(replay.sLap, replay.aSlipRatioRR,'DisplayName','RR')
    hold off
    title('Tyre Slip Ratio')
    ylabel('[-]')
    grid minor
    
    nexttile
    hold on
    plot(replay.sLap, replay.FzTyreFL,'DisplayName','FL')
    plot(replay.sLap, replay.FzTyreFR,'DisplayName','FR')
    plot(replay.sLap, replay.FzTyreRL,'DisplayName','RL')
    plot(replay.sLap, replay.FzTyreRR,'DisplayName','RR')
    hold off;
    title('Fz Tyre')
    ylabel('[-]')
    grid minor
    
    nexttile
    hold on
    plot(replay.sLap, replay.FyTyreFL,'DisplayName','FL')
    plot(replay.sLap, replay.FyTyreFR,'DisplayName','FR')
    plot(replay.sLap, replay.FyTyreRL,'DisplayName','RL')
    plot(replay.sLap, replay.FyTyreRR,'DisplayName','RR')
    hold off;
    title('Fy Tyre')
    ylabel('[-]')
    grid minor
    
    nexttile
    hold on
    plot(replay.sLap, replay.FxTyreFL,'DisplayName','FL')
    plot(replay.sLap, replay.FxTyreFR,'DisplayName','FR')
    plot(replay.sLap, replay.FxTyreRL,'DisplayName','RL')
    plot(replay.sLap, replay.FxTyreRR,'DisplayName','RR')
    hold off;
    title('Fx Tyre')
    ylabel('[-]')
    grid minor

end