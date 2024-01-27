function [rpmInterp, torqInterp] = powerCurveInterpolation(filename)

    % Interpolation for Powertrain Model
    % Smooth Powertrain Curves would be nice but not here :( 
    % 1 RPM = 0.10472 rad/s
    
    motor_data = load(filename);
    AMK_21Nm_Trq = motor_data(:,2); % + (linspace(0, 1, length(motor_data(:,2)))*1E-10)'; % Creating fake unique values
    AMK_Motor_RPM = motor_data(:,1)';
    AMK_Motor_Torque = AMK_21Nm_Trq';
    rpmInterp = linspace(AMK_Motor_RPM(1),AMK_Motor_RPM(end),1000)';
    torqInterp = interp1(AMK_Motor_RPM,AMK_Motor_Torque,rpmInterp);

end

