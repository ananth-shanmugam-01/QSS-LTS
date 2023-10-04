function [c,ceq] = nonlcon_acceleration(control_variables, curvature, velocity, max_cornering_vel)

global mass g h_cg wd m_f m_r t wb a b whl_radius Izz V_max ...
    gear_ratio n_motor pt_eff AMK_21Nm_Trq AMK_Motor_RPM AMK_Motor_Torque...
    CLA CDA aero_balance sector_dist

global unsprung_mass h_cg_unsprung mech_balance h_cg_roll_axis rc_f rc_r...
    sprung_mass sprung_mass_front sprung_mass_rear

% Input Values
Vx = velocity;

del = control_variables(1);
beta = control_variables(2);
throttle_position = control_variables(3);
wheel_rot_fl = control_variables(4);
wheel_rot_fr = control_variables(5);
wheel_rot_rl = control_variables(6);
wheel_rot_rr = control_variables(7);

DF_total = 0.5*1.225*CLA*Vx^2;
DF_front = aero_balance*DF_total;
DF_rear = (1-aero_balance)*DF_front;
Fd = 0.5*1.225*CDA*Vx^2;

kt = curvature;
yaw_rate = Vx*kt;
ay = kt*Vx^2;

% Motor Output
wheel_avg_vel = 0.5*(wheel_rot_rl + wheel_rot_rr);
motor_rot_vel = wheel_avg_vel*gear_ratio*60/(2*pi);
Ft = throttle_position*pt_eff*n_motor*interp1(AMK_Motor_RPM,AMK_Motor_Torque,motor_rot_vel)*gear_ratio/whl_radius;
ax_tractive = (Ft - Fd)/mass;

% Slip Angles
alpha_fl = ((Vx*tan(deg2rad(beta)) + a*yaw_rate) / (Vx + yaw_rate*t*0.5)) - deg2rad(del);
alpha_fr = ((Vx*tan(deg2rad(beta)) + a*yaw_rate) / (Vx - yaw_rate*t*0.5)) - deg2rad(del);
alpha_rl = (Vx*tan(deg2rad(beta)) - b*yaw_rate) / (Vx + yaw_rate*t*0.5);
alpha_rr = (Vx*tan(deg2rad(beta)) - b*yaw_rate) / (Vx - yaw_rate*t*0.5);

% Slip Ratios
% Simplification for Pure Lateral Slip at Apex
kappa_fl = (wheel_rot_fl*whl_radius - Vx)/Vx;
kappa_fr = (wheel_rot_fr*whl_radius - Vx)/Vx;
kappa_rl = (wheel_rot_rl*whl_radius - Vx)/Vx;
kappa_rr = (wheel_rot_rr*whl_radius - Vx)/Vx;

% Lateral Load Transfer
del_w_f = (sprung_mass*ay*h_cg_roll_axis*mech_balance/t) + (sprung_mass_front*ay*rc_f/t) + (unsprung_mass*h_cg_unsprung*ay/t);
del_w_r = (sprung_mass*ay*h_cg_roll_axis*(1-mech_balance)/t) + (sprung_mass_rear*ay*rc_r/t) + (unsprung_mass*h_cg_unsprung*ay/t);

% Wheel Loads 
w_fl = (m_f*g/2) + (del_w_f) - (mass*ax_tractive*h_cg/(2*wb)) + (DF_front/2);
w_fr = (m_f*g/2) - (del_w_f) - (mass*ax_tractive*h_cg/(2*wb)) + (DF_front/2);
w_rl = (m_r*g/2) + (del_w_r) + (mass*ax_tractive*h_cg/(2*wb)) + (DF_rear/2);
w_rr = (m_r*g/2) - (del_w_r) + (mass*ax_tractive*h_cg/(2*wb)) + (DF_rear/2);

% Wheel Forces
[fy_fl, fx_fl] = MF52_Combined(kappa_fl,alpha_fl,w_fl,0);
[fy_fr, fx_fr] = MF52_Combined(kappa_fr,alpha_fr,w_fr,0);
[fy_rl, fx_rl] = MF52_Combined(kappa_rl,alpha_rl,w_rl,0);
[fy_rr, fx_rr] = MF52_Combined(kappa_rr,alpha_rr,w_rr,0);

% Vehicle Acceleration
ay_out = (fy_fl + fy_fr + fy_rl + fy_rr)/mass;
ax_out = (fx_rl + fx_rr)/mass;

V_out = abs(sqrt(Vx^2 + 2*ax_tractive*sector_dist));

% Inequality Constraints
c(1) = V_out - max_cornering_vel; % Below Maximum Cornering Velocity for the next point
c(2) = abs(alpha_fl) - abs(deg2rad(8)); % Measured Slip Angle Limits
c(3) = abs(alpha_fr) - abs(deg2rad(8));
c(4) = abs(alpha_rl) - abs(deg2rad(8));
c(5) = abs(alpha_rr) - abs(deg2rad(8));
c(6) = -w_fl; % Positive Wheel Loads
c(7) = -w_fr;
c(8) = -w_rl;
c(9) = -w_rr;
c(10) = abs(kappa_fl) - 0.1; % Measured Slip Ratio Limits
c(11) = abs(kappa_fr) - 0.1;
c(12) = abs(kappa_rl) - 0.1;
c(13) = abs(kappa_rr) - 0.1;
c(14) = throttle_position - 1;

ceq(1) = (a*(fy_fl + fy_fr) - b*(fy_rl + fy_rr))/Izz; % Yaw Acceleration = 0, for quasi-steady state
ceq(2) = (0.5*t*(fx_fl + fx_rl) - 0.5*t*(fx_fr + fx_rr))/Izz; % Not used to solution instability
ceq(2) = sign(kt) + sign(alpha_fl); % Control sign of slip angle
ceq(3) = sign(kt) + sign(alpha_fr);
ceq(4) = sign(kt) + sign(alpha_rl);
ceq(5) = sign(kt) + sign(alpha_rr);
ceq(6) = ay - ay_out; % Converge between input and out Ay
ceq(7) = ax_tractive - ax_out;

end