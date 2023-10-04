function [ay_meas, dist, vel_ms, time, track_dist, curvature_spline, curvature] = logged_data(InputData,sector_length)

    data = readtable(InputData);
    ay_meas = data.Acc_y_g.*9.81; % Smooth Noisy data due to low sampling rate
    dist = data.dist;
    vel_ms = data.gps_speed_kmh./3.6;
    time = data.time;
    meas_lap_time = time(end);
    yaw_rate = data.yaw_rate_deg_sec;
    
    curvature = smoothdata((ay_meas)./(vel_ms.^2)); % Smooth the noisy stuff :(
    curvature = lowpass((ay_meas)./(vel_ms.^2),0.075);
    
    %% Generate Track Curvature
    track_dist = 0:sector_length:max(dist);
    fitting_factor = 0.7;
    
    curvature_spline = csaps(dist, curvature,fitting_factor,track_dist);
    
    d1 = designfilt("lowpassiir",FilterOrder=12, ...
        HalfPowerFrequency=0.20,DesignMethod="butter");
    curvature = filtfilt(d1,(ay_meas)./(vel_ms.^2));
    curvature_spline = csaps(dist, curvature,fitting_factor,track_dist);
    
    figure
    hold on
    plot(dist,(ay_meas)./(vel_ms.^2),'r','DisplayName','Measured','LineWidth',1)
    plot(track_dist,curvature_spline,'k','DisplayName','FILTFILT','LineWidth',2)
    legend 
    ylabel('Curvature (1/m)')
    xlabel('Distance (m)')
    title('Track Modelling')
    subtitle('Track Curvature Fitting Comparison')
    box on
    hold off


end