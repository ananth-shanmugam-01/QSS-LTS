function [trackDist, curvatureSpline] = trackModel(fileName,sector_length)

    % sector_length = 0.5;
    data = readtable(fileName);
    ay_meas = data.Acc_y_g.*9.81; % Smooth Noisy data due to low sampling rate
    dist = data.dist;
    vel_ms = data.gps_speed_kmh./3.6;
    curvature = lowpass((ay_meas)./(vel_ms.^2),0.075);
    
    % Generate Track Curvature
    trackDist = 0:sector_length:max(dist);
    fitting_factor = 0.7;
    curvatureSpline = csaps(dist, curvature,fitting_factor,trackDist);
    
    % figure
% hold on
% plot(dist,(ay_meas)./(vel_ms.^2),'DisplayName','Measured','LineWidth',3)
% plot(track_dist,curvature_high_spline,'DisplayName','LPF 0.6','LineWidth',3)
% plot(track_dist,curvature_low_spline,'DisplayName','LPF 0.001','LineWidth',3)
% legend 
% ylabel('Curvature (1/m)')
% xlabel('Distance (m)')
% title('Track Modelling')
% subtitle('Track Curvature Fitting Comparison')
% box on
% hold off
% fontsize(gca,13,'points')
% 
% figure
% hold on
% plot(dist,(ay_meas)./(vel_ms.^2),'r','DisplayName','Measured','LineWidth',3)
% plot(track_dist,curvature_spline,'k','DisplayName','LPF 0.075','LineWidth',2)
% legend 
% ylabel('Curvature (1/m)')
% xlabel('Distance (m)')
% title('Track Modelling')
% subtitle('Track Curvature Fitting Comparison')
% box on
% hold off
% fontsize(gca,13,'points')

end