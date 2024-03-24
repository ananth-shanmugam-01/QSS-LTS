%% GGV Calc
function [GGV_surf, GGVresults] = runGGV(velocitySteps, axSteps, carData)

    startTimer = tic;
    
    velocityRange = linspace(7,carData.Powertrain.vMax-2,velocitySteps);
    
    GGV_surf = zeros(numel(velocityRange),2*axSteps,10); % 
    
    for i = 1:numel(velocityRange)
        velocity = velocityRange(i);
                             
        GGV = struct();
        GGV.ax = zeros(axSteps,1);
        GGV.ay = zeros(axSteps,1);
        GGV.delta   = zeros(axSteps,1);
        GGV.beta    = zeros(axSteps,1);
        GGV.yaw_rate = zeros(axSteps,1);
        GGV.wheel_rot_fl = zeros(axSteps,1);
        GGV.wheel_rot_fr = zeros(axSteps,1);
        GGV.wheel_rot_rl = zeros(axSteps,1);
        GGV.wheel_rot_rr = zeros(axSteps,1);
        
        % find Maximum Forward Acceleration
        GGV = ggvModels.acceleration(GGV, carData,velocity);
       
        % find Maximum Braking Deceleration 
        GGV = ggvModels.deceleration(GGV, carData, velocity);
        
        % Combined Acceleration
        GGV.ax = linspace(GGV.ax(1),GGV.ax(end),length(GGV.ax))';
    
        for i_ = 2:length(GGV.ax)-1
            
            if GGV.ax(i_) < 0
    
                % Model
                GGV = ggvModels.combinedAccelerationBraking(i_, GGV, carData, velocity);
               
            else
                
                % Model
                GGV = ggvModels.combinedAccelerationTractive(i_, GGV, carData, velocity);
    
            end
    
        end
    
        GGV_surf(i,:,1) =   [velocity.*ones(2*axSteps,1)];          % Vel [m/s]
        GGV_surf(i,:,2) =   [(GGV.ax) ;    (GGV.ax)];               % Ax [m/s^2]
        GGV_surf(i,:,3) =   [(GGV.ay) ;    -(GGV.ay)];              % Ay [m/s^2]
        GGV_surf(i,:,4) =   [GGV.delta;    -GGV.delta] .*180/pi;    % delta, converted to [deg]
        GGV_surf(i,:,5) =   [GGV.beta;     -GGV.beta]  .*180/pi;    % beta, converted to [deg]
        GGV_surf(i,:,6) =   [GGV.yaw_rate; -(GGV.yaw_rate)];        % yaw_rate [rad/s]
        GGV_surf(i,:,7) =   [GGV.wheel_rot_fl; GGV.wheel_rot_fr];   % wheel_rot_fl, opposite becomes fr [rad/s]
        GGV_surf(i,:,8) =   [GGV.wheel_rot_fr; GGV.wheel_rot_fl];   % wheel_rot_fr, [rad/s]
        GGV_surf(i,:,9) =   [GGV.wheel_rot_rl; GGV.wheel_rot_rr];   % wheel_rot_rl, [rad/s]
        GGV_surf(i,:,10) =  [GGV.wheel_rot_rr; GGV.wheel_rot_rl];   % wheel_rot_rr, [rad/s]


        disp(['-------- ', 'Calculated Step: ', num2str(i) ,'/', num2str(velocitySteps), ' --------'])
    
    end

GGVresults = struct;  

GGVresults.vel = reshape(GGV_surf(:,:,1),[], 1);
GGVresults.Ax = reshape(GGV_surf(:,:,2),[], 1);
GGVresults.Ay = reshape(GGV_surf(:,:,3),[], 1);
GGVresults.delta = reshape(GGV_surf(:,:,4),[], 1);
GGVresults.beta = reshape(GGV_surf(:,:,5),[], 1);
GGVresults.yaw_rate = reshape(GGV_surf(:,:,6),[], 1);
GGVresults.wheel_rot_fl = reshape(GGV_surf(:,:,7),[], 1);
GGVresults.wheel_rot_fr = reshape(GGV_surf(:,:,8),[], 1);
GGVresults.wheel_rot_rl = reshape(GGV_surf(:,:,9),[], 1);
GGVresults.wheel_rot_rr = reshape(GGV_surf(:,:,10),[], 1);

stopTimer = toc(startTimer);

disp(['GGV Calculation Complete. Time taken: ', num2str(stopTimer), '(s)'])

end