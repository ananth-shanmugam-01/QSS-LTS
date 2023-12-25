function [trackDist, curvatureSpline] = trackModel(fileName,sector_length)

    % sector_length = 0.5;
    data = readtable(fileName);
    ay_meas = data.Acc_y_g.*9.81; % Smooth Noisy data due to low sampling rate
    dist = data.dist;
    vel_ms = data.gps_speed_kmh./3.6;
    curvature = lowpass((ay_meas)./(vel_ms.^2),0.1);
    
    % Generate Track Curvature
    trackDist = 0:sector_length:max(dist);
    fitting_factor = 0.7;
    curvatureSpline = csaps(dist, curvature,fitting_factor,trackDist);

end