function [LSP, FSP, RSP] = initialiseResultStructs(size)

% Think of a cleaner way to implement this...

%     controls = struct;
%     controls.Vx                  = zeros(size,1);       % Velocity (m/s)
%     controls.delta               = zeros(size,1);       % steering angle (rad)
%     controls.beta                = zeros(size,1);       % sideslip angle (rad)
%     controls.yaw_rate            = zeros(size,1);       % yaw rate (rad/s)
%     controls.throttle_position   = zeros(size,1);       % throttle position (-)
%     controls.brake_pressure      = zeros(size,1);       % brake pressure (bar)
%     controls.wheel_rot_fl        = zeros(size,1);       % FL wheel angular velocity (rad/s)
%     controls.wheel_rot_fr        = zeros(size,1);       % FR wheel angular velocity (rad/s)
%     controls.wheel_rot_rl        = zeros(size,1);       % RL wheel angular velocity (rad/s)
%     controls.wheel_rot_rr        = zeros(size,1);       % RR wheel angular velocity (rad/s)
%     
%     outputs = struct;
%     outputs.Ay                  = zeros(size,1);
%     outputs.Ax                  = zeros(size,1);
%     outputs.Ax_control          = zeros(size,1);
%     outputs.F_tractive          = zeros(size,1);
%     outputs.F_braking           = zeros(size,1);
%     outputs.F_drag              = zeros(size,1);
%     
%     LSP = struct;
%     LSP.controls = controls;
%     LSP.outputs = outputs;
%     
%     FSP = struct;
%     FSP.controls = controls;
%     FSP.outputs = outputs;
%     
%     RSP = struct;
%     RSP.controls = controls;
%     RSP.outputs = outputs;
    
    LSP = struct;
    FSP = struct;
    RSP = struct; 
    % Optimiser Inputs - Decision Variables + control inputs
    LSP.Vx                  = zeros(size,1);       % Velocity (m/s)
    LSP.delta               = zeros(size,1);       % steering angle (rad)
    LSP.beta                = zeros(size,1);       % sideslip angle (rad)
    LSP.yaw_rate            = zeros(size,1);       % yaw rate (rad/s)
    LSP.throttle_position   = zeros(size,1);       % throttle position (-)
    LSP.brake_pressure      = zeros(size,1);       % brake pressure (bar)
    LSP.wheel_rot_fl        = zeros(size,1);       % FL wheel angular velocity (rad/s)
    LSP.wheel_rot_fr        = zeros(size,1);       % FR wheel angular velocity (rad/s)
    LSP.wheel_rot_rl        = zeros(size,1);       % RL wheel angular velocity (rad/s)
    LSP.wheel_rot_rr        = zeros(size,1);       % RR wheel angular velocity (rad/s)
    
    % Output States - g(x)
    LSP.Ay                  = zeros(size,1);
    LSP.Ax                  = zeros(size,1);
    LSP.Ax_control          = zeros(size,1);
    LSP.F_tractive          = zeros(size,1);
    LSP.F_braking           = zeros(size,1);
    LSP.F_drag              = zeros(size,1);
    
    % Optimiser Inputs - Decision Variables + control inputs
    FSP.V_current           = zeros(size,1);       % Velocity (m/s)
    FSP.delta               = zeros(size,1);       % steering angle (rad)
    FSP.beta                = zeros(size,1);       % sideslip angle (rad)
    FSP.yaw_rate            = zeros(size,1);       % yaw rate (rad/s)
    FSP.throttle_position   = zeros(size,1);       % throttle position (-)
    FSP.brake_pressure      = zeros(size,1);       % brake pressure (bar)
    FSP.wheel_rot_fl        = zeros(size,1);       % FL wheel angular velocity (rad/s)
    FSP.wheel_rot_fr        = zeros(size,1);       % FR wheel angular velocity (rad/s)
    FSP.wheel_rot_rl        = zeros(size,1);       % RL wheel angular velocity (rad/s)
    FSP.wheel_rot_rr        = zeros(size,1);       % RR wheel angular velocity (rad/s)
    
    % Output States - g(x)
    FSP.Ay                  = zeros(size,1);
    FSP.Ax                  = zeros(size,1);
    FSP.Ax_control          = zeros(size,1);
    FSP.F_tractive          = zeros(size,1);
    FSP.F_braking           = zeros(size,1);
    FSP.F_drag              = zeros(size,1);
    
    % Optimiser Inputs - Decision Variables + control inputs
    RSP.V_current           = zeros(size,1);       % Velocity (m/s)
    RSP.delta               = zeros(size,1);       % steering angle (rad)
    RSP.beta                = zeros(size,1);       % sideslip angle (rad)
    RSP.yaw_rate            = zeros(size,1);       % yaw rate (rad/s)
    RSP.throttle_position   = zeros(size,1);       % throttle position (-)
    RSP.brake_pressure      = zeros(size,1);       % brake pressure (bar)
    RSP.wheel_rot_fl        = zeros(size,1);       % FL wheel angular velocity (rad/s)
    RSP.wheel_rot_fr        = zeros(size,1);       % FR wheel angular velocity (rad/s)
    RSP.wheel_rot_rl        = zeros(size,1);       % RL wheel angular velocity (rad/s)
    RSP.wheel_rot_rr        = zeros(size,1);       % RR wheel angular velocity (rad/s)
    
    % Output States - g(x)
    RSP.Ay                  = zeros(size,1);
    RSP.Ax                  = zeros(size,1);
    RSP.Ax_control          = zeros(size,1);
    RSP.F_tractive          = zeros(size,1);
    RSP.F_braking           = zeros(size,1);
    RSP.F_drag              = zeros(size,1);

end