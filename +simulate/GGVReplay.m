function replay = GGVReplay(reMeshLength, outputs, GGVresults, trackData, carData) 
% Create Interpolants for each variable (states and controls) as a function
% of forward velocity, lateral acceleration, longitudinal acceleration,
% used only for initial solution to replay solver
% replay states are only from the NLP solve - coherent with physics

% Forward Velocity Profile
posAyIdx = GGVresults.Ax >= 0; % For parts of the GGV results that correspond to acceleration
AccelerationDeltaInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.delta(posAyIdx),'linear');
AccelerationBeltaInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.beta(posAyIdx),'linear');
AccelerationYawRateInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.yaw_rate(posAyIdx),'linear');
AccelerationWheelRotFlInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.wheel_rot_fl(posAyIdx),'linear');
AccelerationWheelRotFRInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.wheel_rot_fr(posAyIdx),'linear');
AccelerationWheelRotRlInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.wheel_rot_rl(posAyIdx),'linear');
AccelerationWheelRotRRInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.wheel_rot_rr(posAyIdx),'linear');

% Reverse Velocity Profile
posAyIdx = GGVresults.Ax <= 0; % For parts of the GGV results that correspond to braking
DecelerationDeltaInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.delta(posAyIdx),'linear');
DecelerationBeltaInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.beta(posAyIdx),'linear');
DecelerationYawRateInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.yaw_rate(posAyIdx),'linear');
DecelerationWheelRotFlInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.wheel_rot_fl(posAyIdx),'linear');
DecelerationWheelRotFRInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.wheel_rot_fr(posAyIdx),'linear');
DecelerationWheelRotRlInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.wheel_rot_rl(posAyIdx),'linear');
DecelerationWheelRotRRInterp = scatteredInterpolant(GGVresults.vel(posAyIdx), GGVresults.Ay(posAyIdx), GGVresults.Ax(posAyIdx), GGVresults.wheel_rot_rr(posAyIdx),'linear');


% Low Pass Filter
% Bring back dissertation work on a more representative filter~~~
d1 = designfilt("lowpassiir",FilterOrder=4,HalfPowerFrequency=0.1,DesignMethod="butter");

GGVinterp = struct;

GGVinterp.vCar = zeros(numel(outputs.vCar),1);
GGVinterp.Ay = zeros(numel(outputs.vCar),1);
GGVinterp.Ax = zeros(numel(outputs.vCar),1); 
GGVinterp.delta = zeros(numel(outputs.vCar),1);
GGVinterp.beta = zeros(numel(outputs.vCar),1);
GGVinterp.yaw_rate = zeros(numel(outputs.vCar),1);
GGVinterp.wheel_rot_fl = zeros(numel(outputs.vCar),1);
GGVinterp.wheel_rot_fr = zeros(numel(outputs.vCar),1);
GGVinterp.wheel_rot_rl = zeros(numel(outputs.vCar),1);
GGVinterp.wheel_rot_rr = zeros(numel(outputs.vCar),1);


i = find(outputs.vCar == outputs.velocityProfiles.forwardVel);
GGVinterp.vCar(i)       = outputs.vCar(i);
GGVinterp.Ay(i)         = outputs.gLat(i);
GGVinterp.Ax(i)         = outputs.gLong(i); 
GGVinterp.delta(i)      = filtfilt(d1, AccelerationDeltaInterp(outputs.vCar(i),outputs.gLat(i), outputs.gLong(i)));
GGVinterp.beta(i)       = filtfilt(d1, AccelerationBeltaInterp(outputs.vCar(i),outputs.gLat(i), outputs.gLong(i)));
GGVinterp.yaw_rate(i)   = AccelerationYawRateInterp(outputs.vCar(i),outputs.gLat(i), outputs.gLong(i));
GGVinterp.wheel_rot_fl(i) = AccelerationWheelRotFlInterp(outputs.vCar(i),outputs.gLat(i), outputs.gLong(i));
GGVinterp.wheel_rot_fr(i) = AccelerationWheelRotFRInterp(outputs.vCar(i),outputs.gLat(i), outputs.gLong(i));
GGVinterp.wheel_rot_rl(i) = AccelerationWheelRotRlInterp(outputs.vCar(i),outputs.gLat(i), outputs.gLong(i));
GGVinterp.wheel_rot_rr(i) = AccelerationWheelRotRRInterp(outputs.vCar(i),outputs.gLat(i), outputs.gLong(i));

j = find(outputs.vCar == outputs.velocityProfiles.boundaryVel);
GGVinterp.vCar(j)       = outputs.vCar(j);
GGVinterp.Ay(j)         = outputs.gLat(j);
GGVinterp.Ax(j)         = outputs.gLong(j); 
GGVinterp.delta(j)      = filtfilt(d1, AccelerationDeltaInterp(outputs.vCar(j),outputs.gLat(j), outputs.gLong(j)));
GGVinterp.beta(j)       = filtfilt(d1, AccelerationBeltaInterp(outputs.vCar(j),outputs.gLat(j), outputs.gLong(j)));
GGVinterp.yaw_rate(j)   = AccelerationYawRateInterp(outputs.vCar(j),outputs.gLat(j), outputs.gLong(j));
GGVinterp.wheel_rot_fl(j) = AccelerationWheelRotFlInterp(outputs.vCar(j),outputs.gLat(j), outputs.gLong(j));
GGVinterp.wheel_rot_fr(j) = AccelerationWheelRotFRInterp(outputs.vCar(j),outputs.gLat(j), outputs.gLong(j));
GGVinterp.wheel_rot_rl(j) = AccelerationWheelRotRlInterp(outputs.vCar(j),outputs.gLat(j), outputs.gLong(j));
GGVinterp.wheel_rot_rr(j) = AccelerationWheelRotRRInterp(outputs.vCar(j),outputs.gLat(j), outputs.gLong(j));

k = find(outputs.vCar == outputs.velocityProfiles.reverseVel);
GGVinterp.vCar(k)       = outputs.vCar(k);
GGVinterp.Ay(k)         = outputs.gLat(k);
GGVinterp.Ax(k)         = outputs.gLong(k); 
GGVinterp.delta(k)      = filtfilt(d1, DecelerationDeltaInterp(outputs.vCar(k),outputs.gLat(k), outputs.gLong(k)));
GGVinterp.beta(k)       = filtfilt(d1, DecelerationBeltaInterp(outputs.vCar(k),outputs.gLat(k), outputs.gLong(k)));
GGVinterp.yaw_rate(k)   = DecelerationYawRateInterp(outputs.vCar(k),outputs.gLat(k), outputs.gLong(k));
GGVinterp.wheel_rot_fl(k) = DecelerationWheelRotFlInterp(outputs.vCar(k),outputs.gLat(k), outputs.gLong(k));
GGVinterp.wheel_rot_fr(k) = DecelerationWheelRotFRInterp(outputs.vCar(k),outputs.gLat(k), outputs.gLong(k));
GGVinterp.wheel_rot_rl(k) = DecelerationWheelRotRlInterp(outputs.vCar(k),outputs.gLat(k), outputs.gLong(k));
GGVinterp.wheel_rot_rr(k) = DecelerationWheelRotRRInterp(outputs.vCar(k),outputs.gLat(k), outputs.gLong(k));

GGVinterp.time = outputs.time;
GGVinterp.dist = outputs.dist;
% GGVinterp.GGVresults = outputs;

% Further Smoothen controls
GGVinterp.delta = csaps(GGVinterp.dist,GGVinterp.delta,0.3,GGVinterp.dist);
GGVinterp.beta = csaps(GGVinterp.dist,GGVinterp.beta,0.3,GGVinterp.dist);

% Resample track based on input remesh length - not necessary to do the full-track
inputs = struct;

inputs.trackDistance    = linspace(0, trackData.trackDistance(end), reMeshLength)';
inputs.sectorDistance   = gradient(inputs.trackDistance);
inputs.trackCurvature   = interp1(trackData.trackDistance,trackData.trackCurvature,inputs.trackDistance,'makima');

inputs.Vel = interp1(outputs.dist,outputs.vCar,inputs.trackDistance,'makima')';
inputs.gLat = interp1(outputs.dist,outputs.gLat,inputs.trackDistance,'makima')';
inputs.gLong = interp1(outputs.dist,outputs.gLong,inputs.trackDistance,'makima')'; % Filtered g-long used here
inputs.yawRate = inputs.Vel .* inputs.trackCurvature;

% Initial OUtput struct

replay = struct;

% Bring over input parameters
replay.sLap                  = inputs.trackDistance;
replay.trackCurvature        = inputs.trackCurvature;
replay.inputStates.vCar      = inputs.Vel;
replay.inputStates.gLat      = inputs.gLat;
replay.inputStates.gLong     = inputs.gLong;
replay.inputStates.yawRate   = inputs.yawRate;

% extract results
replay.vCar          = zeros(numel(inputs.trackDistance),1);
replay.gLat          = zeros(numel(inputs.trackDistance),1);
replay.gLong         = zeros(numel(inputs.trackDistance),1);
replay.rThrottle     = zeros(numel(inputs.trackDistance),1);
replay.pBrake        = zeros(numel(inputs.trackDistance),1); 
replay.aSteer        = zeros(numel(inputs.trackDistance),1);
replay.aBeta         = zeros(numel(inputs.trackDistance),1);
replay.yawRate       = zeros(numel(inputs.trackDistance),1);
replay.nWheelRotFL   = zeros(numel(inputs.trackDistance),1);
replay.nWheelRotFR   = zeros(numel(inputs.trackDistance),1);
replay.nWheelRotRL   = zeros(numel(inputs.trackDistance),1);
replay.nWheelRotRR   = zeros(numel(inputs.trackDistance),1);
% Wheel Kinematics
replay.aSlipAngleFL = zeros(numel(inputs.trackDistance),1);
replay.aSlipAngleFR = zeros(numel(inputs.trackDistance),1);
replay.aSlipAngleRL = zeros(numel(inputs.trackDistance),1);
replay.aSlipAngleRR = zeros(numel(inputs.trackDistance),1);
replay.aSlipRatioFL = zeros(numel(inputs.trackDistance),1);
replay.aSlipRatioFR = zeros(numel(inputs.trackDistance),1);
replay.aSlipRatioRL = zeros(numel(inputs.trackDistance),1);
replay.aSlipRatioRR = zeros(numel(inputs.trackDistance),1);
% Wheel Forces
replay.FzTyreFL = zeros(numel(inputs.trackDistance),1);
replay.FzTyreFR = zeros(numel(inputs.trackDistance),1);
replay.FzTyreRL = zeros(numel(inputs.trackDistance),1);
replay.FzTyreRR = zeros(numel(inputs.trackDistance),1);
replay.FyTyreFL = zeros(numel(inputs.trackDistance),1);
replay.FyTyreFR = zeros(numel(inputs.trackDistance),1);
replay.FyTyreRL = zeros(numel(inputs.trackDistance),1);
replay.FyTyreRR = zeros(numel(inputs.trackDistance),1);
replay.FxTyreFL = zeros(numel(inputs.trackDistance),1);
replay.FxTyreFR = zeros(numel(inputs.trackDistance),1);
replay.FxTyreRL = zeros(numel(inputs.trackDistance),1);
replay.FxTyreRR = zeros(numel(inputs.trackDistance),1);
% Body Forces
replay.FBrakeFront = zeros(numel(inputs.trackDistance),1);
replay.FBrakeRear  = zeros(numel(inputs.trackDistance),1);
replay.FTractive   = zeros(numel(inputs.trackDistance),1);
replay.FDownforceTotal = zeros(numel(inputs.trackDistance),1);
replay.FDownforceFront = zeros(numel(inputs.trackDistance),1);
replay.FDownforceRear  = zeros(numel(inputs.trackDistance),1);
replay.FDrag           = zeros(numel(inputs.trackDistance),1);

% Create Initial Guess Array from Interpolated States

initialSolution = struct;
initialSolution.throttle         = zeros(numel(inputs.trackDistance),1);
initialSolution.brake            = zeros(numel(inputs.trackDistance),1);
initialSolution.delta            = interp1(GGVinterp.dist,GGVinterp.delta, inputs.trackDistance,'makima');
initialSolution.beta             = interp1(GGVinterp.dist,GGVinterp.beta, inputs.trackDistance,'makima');
initialSolution.wheel_rot_fl     = interp1(GGVinterp.dist,GGVinterp.wheel_rot_fl, inputs.trackDistance,'makima');
initialSolution.wheel_rot_fr     = interp1(GGVinterp.dist,GGVinterp.wheel_rot_fr, inputs.trackDistance,'makima');
initialSolution.wheel_rot_rl     = interp1(GGVinterp.dist,GGVinterp.wheel_rot_rl, inputs.trackDistance,'makima');
initialSolution.wheel_rot_rr     = interp1(GGVinterp.dist,GGVinterp.wheel_rot_rr, inputs.trackDistance,'makima');

%% Loop across new mesh points

targets = inputs;

for i = 1:numel(inputs.trackDistance)
   
    replay = ggv.replay(i, initialSolution, targets, replay, carData);

end

% Time metric
replay.time = cumsum(gradient(replay.sLap) ./ replay.vCar);

end
%%

% % Return solved outputs
% states = fieldnames(replay);
% figure(1); tiledlayout(7,2);
% 
% for i = 1:numel(states)
%     nexttile
%     plot(inputs.trackDistance, replay.(states{i}))
%     title(char(states{i}),'Interpreter','none')
% end
% 
% %%
% 
% figure(2); clf; tiledlayout(3,1)
% nexttile
% plot(inputs.trackDistance,replay.vel)
% 
% nexttile
% hold on
% plot(inputs.trackDistance,replay.ax,'DisplayName','GGV')
% plot(inputs.trackDistance,inputs.gLong,'DisplayName','Replay')
% hold off; legend
% 
% 
% nexttile
% hold on
% plot(inputs.trackDistance, replay.ay,'DisplayName','GGV')
% plot(inputs.trackDistance,inputs.gLat,'DisplayName','Replay')
% hold off; legend
