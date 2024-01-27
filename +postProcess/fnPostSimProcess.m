function lapData = fnPostSimProcess(trackData,LSP,FSP,RSP)

lapData = struct;

lapSegments = gradient(trackData.trackDistance)'; % prep for curvature based adaptive meshing
lapData.vCar = min([LSP.Vx'; FSP.V_current'; RSP.V_current'])';
lapData.time = cumsum(lapSegments./lapData.vCar);
lapData.sLap = trackData.trackDistance';

disp(['Simulated Lap Time - ', num2str(lapData.time(numel(lapData.time))),'(s)'])

% Calculate Longitudinal Acceleration
lapData.gLong = zeros(length(trackData.trackDistance),1);

for i = 1:length(trackData.trackDistance)-1
    lapData.gLong(i) = (1/9.81)*(lapData.vCar(i+1)^2 - lapData.vCar(i)^2)/(2*lapSegments(i));
end

lapData.gLong(length(lapData.vCar)) = (1/9.81)*(lapData.vCar(1)^2 - lapData.vCar(length(lapData.vCar))^2)/(2*lapSegments(end));
lapData.gLat = (1/9.81).*trackData.trackCurvature'.*lapData.vCar.^2;
lapData.vYaw = trackData.trackCurvature'.*lapData.vCar;

%% Control Inputs
lapData.pBrake = zeros(length(trackData.trackDistance),1);
lapData.rThrottle = zeros(length(trackData.trackDistance),1);
lapData.aSteer = zeros(length(trackData.trackDistance),1);
lapData.aBeta = zeros(length(trackData.trackDistance),1);

lapData.pBrake(RSP.V_current == lapData.vCar) = RSP.brake_pressure(RSP.V_current == lapData.vCar);
lapData.rThrottle(FSP.V_current == lapData.vCar) = FSP.throttle_position(FSP.V_current == lapData.vCar);

lapData.aSteer(RSP.V_current == lapData.vCar) = rad2deg(RSP.delta(RSP.V_current == lapData.vCar));
lapData.aSteer(FSP.V_current == lapData.vCar) = rad2deg(FSP.delta(FSP.V_current == lapData.vCar));
lapData.aSteer(LSP.Vx == lapData.vCar) = rad2deg(LSP.delta(LSP.Vx == lapData.vCar));

end