torqueData = readtable("dataFiles\KTM450SXF_Powertrain.xlsx",'Sheet','Torque Curve');
driveTrain = readtable("dataFiles\KTM450SXF_Powertrain.xlsx",'Sheet','Drivetrain');

powertrain = struct;

for i = 1:numel(driveTrain.VariableName)
    powertrain.(driveTrain.VariableName{i}) = driveTrain.Value(i);
end

carData.Chassis.radWheel = 0.196; % Loaded radius

vMaxWheel = (torqueData.RPM(end)*2*pi/60)/(powertrain.primaryRatio * powertrain.finalDriveRatio * powertrain.gearRatio5);

%% Wheel Velocity Range

wheelVelRange = linspace(0, vMaxWheel, 1000)';

engineTorque = zeros(1000,5);

% Torque for Gear 1
for i = 1:5
    engineRPM = (60/2*pi) * wheelVelRange * (powertrain.primaryRatio * powertrain.finalDriveRatio * powertrain.(['gearRatio' num2str(i)]));
    engineTorque(:,i) = interp1(torqueData.RPM,torqueData.Torque_Nm_,engineRPM,"linear");
    engineTorque(engineRPM > torqueData.RPM(end)) = 0;
end


figure
scatter(repmat(wheelVelRange,1,5), engineTorque)




