%% combined Deceleration Calc

function [ObjectiveFunction,ConstraintFunction] = combinedDecelerationCalc(carData)

    % Input Values
    x = sym('x',[1 9],'real');    

    % Input Values
    %     del = x(1);
    %     beta = x(2);
    %     wheel_rot_fl = x(3);
    %     wheel_rot_fr = x(4);
    %     wheel_rot_rl = x(5);
    %     wheel_rot_rr = x(6);
    %     Vx = x(7);
    %     Ax = x(8);
    %     Ay = x(9);
    
    DF_total = 0.5*1.225*carData.Aero.CLA*x(7)^2;
    DF_front = carData.Aero.rAeroBalance*DF_total;
    DF_rear = (1-carData.Aero.rAeroBalance)*DF_front;
    Fd = 0.5*1.225*carData.Aero.CDA*x(7)^2;

    yaw_rate = x(9)/x(7);
    
    g = 9.81;
        
    % Slip Angles
    alpha_fl = ((x(7)*tan(deg2rad(x(2))) + carData.Chassis.frontMomentArm*yaw_rate) / (x(7) + yaw_rate*carData.Chassis.trackWidth*0.5)) - deg2rad(x(1));
    alpha_fr = ((x(7)*tan(deg2rad(x(2))) + carData.Chassis.frontMomentArm*yaw_rate) / (x(7) - yaw_rate*carData.Chassis.trackWidth*0.5)) - deg2rad(x(1));
    alpha_rl = (x(7)*tan(deg2rad(x(2))) - carData.Chassis.rearMomentArm*yaw_rate) / (x(7) + yaw_rate*carData.Chassis.trackWidth*0.5);
    alpha_rr = (x(7)*tan(deg2rad(x(2))) - carData.Chassis.rearMomentArm*yaw_rate) / (x(7) - yaw_rate*carData.Chassis.trackWidth*0.5);
    
    % Slip Ratios
    % Simplification for Pure Lateral Slip at Apex
    kappa_fl = (x(3)*carData.Chassis.radWheel - x(7))/x(7);
    kappa_fr = (x(4)*carData.Chassis.radWheel - x(7))/x(7);
    kappa_rl = (x(5)*carData.Chassis.radWheel - x(7))/x(7);
    kappa_rr = (x(6)*carData.Chassis.radWheel - x(7))/x(7);
    
    % Lateral Load Transfer
    del_w_f = (carData.Chassis.SprungMass * x(9) * carData.Suspension.heightCG2rollAxis * carData.Suspension.mechanicalBalance/carData.Chassis.trackWidth)...
        + (carData.Chassis.sprungMassFront* x(9) * carData.Suspension.rollCentreFront / carData.Chassis.trackWidth) + (carData.Chassis.unsprungMass * carData.Chassis.heightUnsprungCOG * x(9) / carData.Chassis.trackWidth);
    del_w_r = (carData.Chassis.SprungMass * x(9) * carData.Suspension.heightCG2rollAxis * (1-carData.Suspension.mechanicalBalance)/carData.Chassis.trackWidth)...
        + (carData.Chassis.sprungMassRear * x(9) * carData.Suspension.rollCentreRear / carData.Chassis.trackWidth) + (carData.Chassis.unsprungMass * carData.Chassis.heightUnsprungCOG * x(9) / carData.Chassis.trackWidth);
    
    % Longitudinal Load Transfer
    longLT = carData.Chassis.mass * x(8) * carData.Chassis.heightSprungCOG / (2 * carData.Chassis.wheelBase);
    
    % Wheel Loads 
    w_fl = (carData.Chassis.massFront * g / 2) + (del_w_f) - longLT + (DF_front/2);
    w_fr = (carData.Chassis.massFront * g / 2) - (del_w_f) - longLT + (DF_front/2);
    w_rl = (carData.Chassis.massRear * g / 2) + (del_w_r) + longLT + (DF_rear/2);
    w_rr = (carData.Chassis.massRear * g / 2) - (del_w_r) + longLT + (DF_rear/2);
    
    % Wheel Forces
    [fy_fl, fx_fl] = MF52_Combined(kappa_fl,alpha_fl,w_fl,0);
    [fy_fr, fx_fr] = MF52_Combined(kappa_fr,alpha_fr,w_fr,0);
    [fy_rl, fx_rl] = MF52_Combined(kappa_rl,alpha_rl,w_rl,0);
    [fy_rr, fx_rr] = MF52_Combined(kappa_rr,alpha_rr,w_rr,0);
    
    % Vehicle Acceleration
    ay_out = (fy_fl + fy_fr + fy_rl + fy_rr)/carData.Chassis.mass;
    ax_out = (fx_fl + fx_fr + fx_rl + fx_rr - Fd)/carData.Chassis.mass;
    
    opt_out = -ay_out; % Maximise Lateral Acceleration

    gradientObjectiveForwardAcc = jacobian(opt_out,x).';
    
    % hessianObjectiveForwardAcc = jacobian(gradientObjectiveForwardAcc,x);
    
    ObjectiveFunction = matlabFunction(opt_out,gradientObjectiveForwardAcc,'vars',{x});

    % Inequality Constraints
    % Merge Acceleration and Braking

    % Inequality Constraints
    c = sym(zeros(1,8));

    c(1) = -0.15 - kappa_fl; % Measured Slip Ratio Limits
    c(2) = -0.15 - kappa_fr;
    c(3) = -0.15 - kappa_rl;
    c(4) = -0.15 - kappa_rr;
    c(5) = alpha_fl - deg2rad(-12); % Measured Slip Angle Limits
    c(6) = alpha_fr - deg2rad(-12);
    c(7) = alpha_rl - deg2rad(-12);
    c(8) = alpha_rr - deg2rad(-12);
    
    gradc = jacobian(c,x).'; % .' performs transpose
    
    % Equality Constraints: Yaw Moment, Front Slip = 0, Ax = ax_out, Ay_out
    ceq = sym(zeros(1,4));

    ceq(1) = (carData.Chassis.frontMomentArm*(fy_fl + fy_fr) - carData.Chassis.rearMomentArm*(fy_rl + fy_rr))/(1000*carData.Chassis.yawInertia); % Yaw Acceleration = 0, for quasi-steady state
    ceq(2) = 0; %(0.5*carData.Chassis.trackWidth*(fx_fl + fx_rl) - 0.5*carData.Chassis.trackWidth*(fx_fr + fx_rr))/carData.Chassis.yawInertia; % Not used to solution instability
    ceq(3) = x(9) - ay_out; % Converge between input and out Ay
    ceq(4) = x(8) - ax_out;

    gradceq = jacobian(ceq,x).'; % .' performs transpose

    ConstraintFunction = matlabFunction(c,ceq,gradc,gradceq,'vars',{x},'outputs',{'c','ceq','gradc','gradceq'});

end