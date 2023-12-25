%% Calculate - Braking Objective Function and Gradients

function [ObjectiveFunction,ConstraintFunction] = BrakingDecelerationCalc(carData)

    x = sym('x',[1 6],'real');
    
    % brakePressure = x(1).*100;
    % wheel_rot_fl = x(2);
    % wheel_rot_fr = x(3);
    % wheel_rot_rl = x(4);
    % wheel_rot_rr = x(5);
    % Vx = x(6);
    
    DF_total = 0.5*1.225*carData.Aero.CLA*x(6)^2;
    DF_front = carData.Aero.rAeroBalance * DF_total;
    DF_rear = (1-carData.Aero.rAeroBalance)*DF_front;
    Fd = 0.5*1.225*carData.Aero.CDA*x(6)^2;
    
    g = 9.81;
    
    % Braking Torque
    % Brake Model with brake pressure as input and brake bias
    Front_Brake_Force = 2*(x(1) * carData.Brakes.rBrakeBias .*10^5 .*carData.Brakes.areaPiston .*carData.Brakes.nPistonFront) * carData.Brakes.muBrakePad * carData.Brakes.radBrakeDisc / carData.Chassis.radWheel; %N
    Rear_Brake_Force = 2*(x(1)*(1-carData.Brakes.rBrakeBias) .*10^5 .*carData.Brakes.areaPiston .*carData.Brakes.nPistonFront) * carData.Brakes.muBrakePad * carData.Brakes.radBrakeDisc / carData.Chassis.radWheel; %N
    ax_brake = -(Front_Brake_Force + Rear_Brake_Force + Fd) / carData.Chassis.mass;
    
    % Slip Ratios
    % Simplification for Pure Lateral Slip at Apex
    kappa_fl = (x(2)*carData.Chassis.radWheel - x(6))/x(6);
    kappa_fr = (x(3)*carData.Chassis.radWheel - x(6))/x(6);
    kappa_rl = (x(4)*carData.Chassis.radWheel - x(6))/x(6);
    kappa_rr = (x(5)*carData.Chassis.radWheel - x(6))/x(6);
    
    % Longitudinal Load Transfer
    longLT = carData.Chassis.mass * ax_brake * carData.Chassis.heightSprungCOG / (2 * carData.Chassis.wheelBase);
    
    % Wheel Loads 
    w_fl = (carData.Chassis.massFront * g / 2)  - longLT + (DF_front/2);
    w_fr = (carData.Chassis.massFront * g / 2)  - longLT + (DF_front/2);
    w_rl = (carData.Chassis.massRear * g / 2)  + longLT + (DF_rear/2);
    w_rr = (carData.Chassis.massRear * g / 2)  + longLT + (DF_rear/2);
    
    % Wheel Forces
    [~, fx_fl] = MF52_Combined(kappa_fl,0,w_fl,0);
    [~, fx_fr] = MF52_Combined(kappa_fr,0,w_fr,0);
    [~, fx_rl] = MF52_Combined(kappa_rl,0,w_rl,0);
    [~, fx_rr] = MF52_Combined(kappa_rr,0,w_rr,0);
    
    % Vehicle Acceleration
    ax_out = (fx_fl + fx_fr + fx_rl + fx_rr - Fd) / carData.Chassis.mass;
    
    opt_out = ax_out; % Maximise negative longitudinal acceleration
    
    gradientObjectiveForwardAcc = jacobian(opt_out,x).';
    
    % hessianObjectiveForwardAcc = jacobian(gradientObjectiveForwardAcc,x);
    
    ObjectiveFunction = matlabFunction(opt_out,gradientObjectiveForwardAcc,'vars',{x});
    
    % HessianOfObjectiveFunction = matlabFunction(hessianObjectiveForwardAcc,'vars',{x});

    % Inequality Constraints
    c = sym(zeros(1,6));
    
    c(1) = x(6) - carData.Powertrain.vMax;
    c(2) = x(1) - 100;
    c(3) = -0.15 - kappa_fl; % Measured Slip Ratio Limits
    c(4) = -0.15 - kappa_fr;
    c(5) = -0.15 - kappa_rl;
    c(6) = -0.15 - kappa_rr;

    gradc = jacobian(c,x).'; % .' performs transpose

    ceq = sym(zeros(1,2));

    ceq(1) = (0.5*carData.Chassis.trackWidth*(fx_fl + fx_rl) - 0.5*carData.Chassis.trackWidth*(fx_fr + fx_rr))/carData.Chassis.yawInertia; 
    ceq(2) = ax_brake - ax_out;

    gradceq = jacobian(ceq,x).';

    % Create Constraint Function File
    ConstraintFunction = matlabFunction(c,ceq,gradc,gradceq,'vars',{x},'outputs',{'c','ceq','gradc','gradceq'});

end