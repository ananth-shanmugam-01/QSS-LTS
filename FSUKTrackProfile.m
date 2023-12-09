%% Track Model
function [track_dist, curvature_spline, measVelocity, dist, time] = FSUKTrackProfile(sector_dist)

    rawData = load("C:\Users\Ananth\Downloads\230722_Endurance_lap2.mat");
    time = rawData.data_lap2.t;
    measVelocity = rawData.data_lap2.Chassis_Speed_mps;
    latData = smoothdata(rawData.data_lap2.GPS_Latitude_deg,"gaussian","SmoothingFactor",0.001);
    longData = smoothdata(rawData.data_lap2.GPS_Longitude_deg,"gaussian","SmoothingFactor",0.001);
    altData = rawData.data_lap2.GPS_Altitude_m;
    
    [kt, calcSpeed, dist] = GPScurvature(latData,longData,altData,time);
    
    track_dist = 0:sector_dist:dist(end);
    curvature_spline = csaps(dist, kt,0.7,track_dist);
     
    % figure
    % tiledlayout(2,1)
    % nexttile
    % hold on
    % plot(dist,calcSpeed,'DisplayName','GPS Speed')
    % plot(dist,vel,'DisplayName','Meas Chassis Speed')
    % hold off
    % legend
    % grid minor
    % 
    % nexttile
    % hold on
    % plot(dist,kt,'DisplayName','GPS')
    % plot(track_dist,curvature_spline,'DisplayName','Interp')
    % hold off
    
    function [kt, calcSpeed, dist] = GPScurvature(latData,longData,altData,time)
    
    origin = [latData(1),longData(1),altData(1)];
    [latCart,longCart,~] = latlon2local(latData,longData,altData,origin); % convert to cartesian coords
    
    xdot = gradient(latCart)./gradient(time);
    ydot = gradient(longCart)./gradient(time);
    xddot = gradient(xdot)./gradient(time);
    yddot = gradient(ydot)./gradient(time);
    kt = (xdot.*yddot - ydot.*xddot)./(( xdot.^2 + ydot.^2).^1.5);
    calcSpeed = sqrt(xdot.^2 + ydot.^2);
    dist = cumtrapz(time,calcSpeed);
    
    end

end


