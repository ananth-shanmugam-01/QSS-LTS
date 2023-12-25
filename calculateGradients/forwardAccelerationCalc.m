%% Forward Acceleration Objective Function, Constraints and Jacobians

function [ObjectiveFunction,ConstraintFunction] = forwardAccelerationCalc(carData)
    
    % Input Values
    x = sym('x',[1 6],'real');
    
    % Vx = x(1);
    % throttle_position = x(2);
    % wheel_rot_fl = x(3);
    % wheel_rot_fr = x(4);
    % wheel_rot_rl = x(5);
    % wheel_rot_rr = x(6);
    
    DF_total = 0.5*1.225*carData.Aero.CLA*x(1)^2;
    DF_front = carData.Aero.rAeroBalance*DF_total;
    DF_rear = (1-carData.Aero.rAeroBalance)*DF_front;
    Fd = 0.5*1.225*carData.Aero.CDA*x(1)^2;
    
    g = 9.81;

    % Motor Output
    wheel_avg_vel = 0.5*(x(5) + x(6));
    motor_rot_vel = wheel_avg_vel * carData.Powertrain.rGear *60/(2*pi);
%    Ft = x(2) * carData.Powertrain.effPU * carData.Powertrain.nDrive * 20 * carData.Powertrain.rGear/carData.Chassis.radWheel;
    Ft = x(2) * carData.Powertrain.effPU * carData.Powertrain.nDrive / (motor_rot_vel * (2*pi) / 60) *...
        (8.65035700647986e-17*(motor_rot_vel.^5) - 4.41602015686910e-12*(motor_rot_vel^4) + 7.31673593263643e-08*(motor_rot_vel^3) - 0.000481455921495143*(motor_rot_vel^2) + 3.34888694342485*(motor_rot_vel) - 609.934138992043)....
        * carData.Powertrain.rGear/carData.Chassis.radWheel;
    
    ax_tractive = (Ft - Fd) / carData.Chassis.mass;
    
    % Slip Ratios
    % Simplification for Pure Lateral Slip at Apex
    kappa_fl = (x(3)*carData.Chassis.radWheel - x(1))/x(1);
    kappa_fr = (x(4)*carData.Chassis.radWheel - x(1))/x(1);
    kappa_rl = (x(5)*carData.Chassis.radWheel - x(1))/x(1);
    kappa_rr = (x(6)*carData.Chassis.radWheel - x(1))/x(1);
    
    % Longitudinal Load Transfer
    longLT = carData.Chassis.mass * ax_tractive * carData.Chassis.heightSprungCOG / (2 * carData.Chassis.wheelBase);
    
    % Wheel Loads 
    w_fl = (carData.Chassis.massFront * g / 2) - longLT + (DF_front/2);
    w_fr = (carData.Chassis.massFront * g / 2) - longLT + (DF_front/2);
    w_rl = (carData.Chassis.massRear * g / 2)  + longLT + (DF_rear/2);
    w_rr = (carData.Chassis.massRear * g / 2)  + longLT + (DF_rear/2);
    
    % Wheel Forces
    [~, fx_fl] = MF52_Combined(kappa_fl,0,w_fl,0);
    [~, fx_fr] = MF52_Combined(kappa_fr,0,w_fr,0);
    [~, fx_rl] = MF52_Combined(kappa_rl,0,w_rl,0);
    [~, fx_rr] = MF52_Combined(kappa_rr,0,w_rr,0);
    
    % Vehicle Acceleration
    ax_out = (fx_rl + fx_rr)/carData.Chassis.mass;
    
    % Objective Function
    opt_out = -ax_out; % Maximise Forward Acceleration
    
    gradientObjectiveForwardAcc = jacobian(opt_out,x).';
    
    % hessianObjectiveForwardAcc = jacobian(gradientObjectiveForwardAcc,x);
    
    ObjectiveFunction = matlabFunction(opt_out,gradientObjectiveForwardAcc,'vars',{x});
    
    % HessianOfObjectiveFunction = matlabFunction(hessianObjectiveForwardAcc,'vars',{x});
    
    %% NonLinear Constraints
    
    % Inequality Constraints
    c = sym(zeros(1,6));
    
    c(1) = x(1) - carData.Powertrain.vMax; % Below Maximum Cornering Velocity for the next point
    c(2) = x(2) - 1;
    c(3) = kappa_fl - 0.15; % Measured Slip Ratio Limits
    c(4) = kappa_fr - 0.15;
    c(5) = kappa_rl - 0.15;
    c(6) = kappa_rr - 0.15;
    
    gradc = jacobian(c,x).'; % .' performs transpose
    
%     hessc = cell(1, length(c)); % Initialise Empty Cell for Hessian of Inequality Constraints
%     constraints = struct;
%     
%     % Calculate Hessians and assign to struct containing jacobians of constraint function
%     for i = 1:length(c)
%         hessc{i} = jacobian(gradc(:,i),x);
%         ii = num2str(i);
%         thename = ['hessc',ii];
%         constraints.(thename) = matlabFunction(hessc{i},'vars',{x});
%     end
    
    % Equality Constraints
    ceq = sym(zeros(1,4));
    
    ceq(1) = (0.5*carData.Chassis.trackWidth*(fx_fl + fx_rl) - 0.5*carData.Chassis.trackWidth*(fx_fr + fx_rr))/carData.Chassis.yawInertia;
    ceq(2) = ax_tractive - ax_out;
    ceq(3) = kappa_fl; % Measured Slip Ratio Limits
    ceq(4) = kappa_fr;
    
    gradceq = jacobian(ceq,x).'; % .' performs transpose
%     hessceq = cell(1, length(ceq)); % Initialise Empty Cell for Hessian of Equality Constraints
%     
%     for i = 1:length(ceq)
%         hessceq{i} = jacobian(gradceq(:,i),x);
%         ii = num2str(i);
%         thename = ['hessceq',ii];
%         constraints.(thename) = matlabFunction(hessceq{i},'vars',{x});
%     end
    
    % Create Constraint Function File
    ConstraintFunction = matlabFunction(c,ceq,gradc,gradceq,'vars',{x},'outputs',{'c','ceq','gradc','gradceq'});

end
