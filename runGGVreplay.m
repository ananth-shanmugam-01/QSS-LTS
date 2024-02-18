clear; clc;
load("outputs.mat");
carData = preProccess.initVehicleModel();

result = struct;

result.vel          = zeros(numel(outputs.time),1);
result.ay           = zeros(numel(outputs.time),1);
result.ax           = zeros(numel(outputs.time),1);
result.throttle     = zeros(numel(outputs.time),1);
result.brake        = zeros(numel(outputs.time),1);
result.delta        = zeros(numel(outputs.time),1);
result.beta         = zeros(numel(outputs.time),1);
result.yaw_rate     = zeros(numel(outputs.time),1);
result.wheel_rot_fl = zeros(numel(outputs.time),1);
result.wheel_rot_fr = zeros(numel(outputs.time),1);
result.wheel_rot_rl = zeros(numel(outputs.time),1);
result.wheel_rot_rr = zeros(numel(outputs.time),1);

for index = 2:numel(outputs.time)
    
    result = replay.getReplayStates(index, outputs, carData, result);

end

%%

result.dist = outputs.dist;
result.time = outputs.time;
result.gLat_ref = outputs.gLat;
result.gLong_ref = outputs.gLong;
result.vCar_ref = outputs.vCar;
result.yawRate_ref = outputs.yawRate;

%% 