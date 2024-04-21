%% GGV Calc
function [GGV_surf, GGVresults] = runGGV(numelVelocitySteps, numelAxSteps, carData)

    startTimer = tic;
    
    velocityRange = linspace(7,carData.Powertrain.vMax-2,numelVelocitySteps);
    
    GGV_surf = zeros(numel(velocityRange),2*numelAxSteps,10); % 
    
    for i = 1:numel(velocityRange)
        
        velocity = velocityRange(i);
    
        GGV              = struct();
        GGV.ax           = zeros(numelAxSteps,1);
        GGV.ay           = zeros(numelAxSteps,1);
        GGV.delta        = zeros(numelAxSteps,1);
        GGV.beta         = zeros(numelAxSteps,1);
        GGV.yaw_rate     = zeros(numelAxSteps,1);
        GGV.wheel_rot_fl = zeros(numelAxSteps,1);
        GGV.wheel_rot_fr = zeros(numelAxSteps,1);
        GGV.wheel_rot_rl = zeros(numelAxSteps,1);
        GGV.wheel_rot_rr = zeros(numelAxSteps,1);
        
        % Maximum Acceleration at this velocity
        GGV = ggv.optiProblem(velocity, carData, GGV, 1, 'acceleration', nan);
        
        % Maximum Deceleration at this velocity
        GGV = ggv.optiProblem(velocity, carData, GGV, numelAxSteps, 'deceleration', nan);
        
        % Range of longitudinal accelerations to evaluate
        AxTarget = linspace(GGV.ax(1),GGV.ax(end),length(GGV.ax))';
        
        for i_ = 2:length(AxTarget)-1
            
            if AxTarget(i_) < 0
        
                % Combined Deceleration Calculation
                GGV = ggv.optiProblem(velocity, carData, GGV, i_, 'combineddeceleration', AxTarget(i_));
               
            else
                
                % Combined Acceleration Calculation
                GGV = ggv.optiProblem(velocity, carData, GGV, i_, 'combinedacceleration', AxTarget(i_));
        
            end
        
        end
    
        GGV_surf(i,:,1) =   [velocity.*ones(2*numelAxSteps,1)];          % Vel [m/s]
        GGV_surf(i,:,2) =   [(GGV.ax) ;    (GGV.ax)];               % Ax [m/s^2]
        GGV_surf(i,:,3) =   [(GGV.ay) ;    -(GGV.ay)];              % Ay [m/s^2]
        GGV_surf(i,:,4) =   [GGV.delta;    -GGV.delta] .*180/pi;    % delta, converted to [deg]
        GGV_surf(i,:,5) =   [GGV.beta;     -GGV.beta]  .*180/pi;    % beta, converted to [deg]
        GGV_surf(i,:,6) =   [GGV.yaw_rate; -(GGV.yaw_rate)];        % yaw_rate [rad/s]
        GGV_surf(i,:,7) =   [GGV.wheel_rot_fl; GGV.wheel_rot_fr];   % wheel_rot_fl, opposite becomes fr [rad/s]
        GGV_surf(i,:,8) =   [GGV.wheel_rot_fr; GGV.wheel_rot_fl];   % wheel_rot_fr, [rad/s]
        GGV_surf(i,:,9) =   [GGV.wheel_rot_rl; GGV.wheel_rot_rr];   % wheel_rot_rl, [rad/s]
        GGV_surf(i,:,10) =  [GGV.wheel_rot_rr; GGV.wheel_rot_rl];   % wheel_rot_rr, [rad/s]
    
    
        disp(['-------- ', 'Calculated Step: ', num2str(i) ,'/', num2str(numelVelocitySteps), ' --------'])
    
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