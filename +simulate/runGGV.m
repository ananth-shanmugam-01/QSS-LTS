%% GGV Calc
function [GGV_surf, GGVComplete, GGVAcceleration, GGVDeceleration] = runGGV(velocitySteps, axSteps, carData)

    startTimer = tic;
    
    velocityRange = linspace(7,carData.Powertrain.vMax-2,velocitySteps);
    
    GGV_surf = [];
    
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
        GGV.ax = linspace(GGV.ax(1),GGV.ax(end),length(GGV.ax));
    
        for i_ = 2:length(GGV.ax)-1
            
            if GGV.ax(i_) < 0
    
                % Model
                GGV = ggvModels.combinedAccelerationBraking(i_, GGV, carData, velocity);
               
            else
                
                % Model
                GGV = ggvModels.combinedAccelerationTractive(i_, GGV, carData, velocity);
    
            end
    
        end
    
        GGV_surf(i,:,1) = [velocity.*ones(2*axSteps,1)]; % Vel
        GGV_surf(i,:,2) = [(GGV.ax)'; (GGV.ax)']; % Ax
        GGV_surf(i,:,3) = [(GGV.ay); -(GGV.ay)]; % Ay

        disp(['-------- ', 'Calculated Step: ', num2str(i) ,'/', num2str(velocitySteps), ' --------'])
    
    end

Ay = reshape(GGV_surf(:,:,3),[], 1);
Ax = reshape(GGV_surf(:,:,2),[], 1);
vel = reshape(GGV_surf(:,:,1),[], 1);

GGVComplete = [vel, Ay, Ax];
GGVAcceleration = [vel(Ax>0), Ay(Ax>0), Ax(Ax>0)];
GGVDeceleration = [vel(Ax<0), Ay(Ax<0), Ax(Ax<0)];

stopTimer = toc(startTimer);

disp(['GGV Calculation Complete. Time taken: ', num2str(stopTimer), '(s)'])

end