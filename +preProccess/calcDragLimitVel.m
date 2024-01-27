% Crude Drag Limited Velocity Check
function dragLimitVel = calcDragLimitVel(carData)

    wheel_avg_vel = 0:1:carData.Powertrain.vMaxRPM;
    motor_rot_vel = wheel_avg_vel .* carData.Powertrain.rGear *60/(2*pi) / carData.Chassis.radWheel;
    Ft = carData.Powertrain.effPU * carData.Powertrain.nDrive .* interp1(carData.Powertrain.RPM,carData.Powertrain.torqueMotor,motor_rot_vel,'spline') * carData.Powertrain.rGear/carData.Chassis.radWheel;
    Fd = 0.5*1.225*carData.Aero.CDA.*wheel_avg_vel.^2;
    Ax = sign((Ft - Fd)./carData.Chassis.mass);
    Ax = diff(Ax);
    if isempty(find(Ax == -2))
        wheel_avg_vel = 0:1:carData.Powertrain.vMaxRPM*2;
        motor_rot_vel = wheel_avg_vel .* carData.Powertrain.rGear *60/(2*pi) / carData.Chassis.radWheel;
        Ft = carData.Powertrain.effPU * carData.Powertrain.nDrive .* interp1(carData.Powertrain.RPM,carData.Powertrain.torqueMotor,motor_rot_vel,'spline') * carData.Powertrain.rGear/carData.Chassis.radWheel;
        Fd = 0.5*1.225*carData.Aero.CDA.*wheel_avg_vel.^2;
        Ax = sign((Ft - Fd)./carData.Chassis.mass);
        Ax = diff(Ax);       
        dragLimitVel = wheel_avg_vel(find(Ax == -2)); 
    else 
        dragLimitVel = wheel_avg_vel(find(Ax == -2));
    end

end