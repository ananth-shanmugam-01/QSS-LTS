function opt_out = max_deceleration(control_variables, curvature, prev_velocity)

global mass g h_cg wd m_f m_r t wb a b whl_radius Izz V_max ...
    gear_ratio n_motor pt_eff AMK_21Nm_Trq AMK_Motor_RPM AMK_Motor_Torque...
    CLA CDA aero_balance sector_dist

global unsprung_mass h_cg_unsprung mech_balance h_cg_roll_axis rc_f rc_r...
    sprung_mass sprung_mass_front sprung_mass_rear

del = control_variables(1);
beta = control_variables(2);
ax_brake = control_variables(3);
wheel_rot_fl = control_variables(4);
wheel_rot_fr = control_variables(5);
wheel_rot_rl = control_variables(6);
wheel_rot_rr = control_variables(7);
Vx = control_variables(8); % Constraints for Vx defined in nonlcon

DF_total = 0.5*1.225*CLA*Vx^2;
DF_front = aero_balance*DF_total;
DF_rear = (1-aero_balance)*DF_front;
Fd = 0.5*1.225*CDA*Vx^2;

kt = curvature;
yaw_rate = Vx*kt;
ay = kt*Vx^2;

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
w_fl = (m_f*g/2) + (del_w_f) - (mass*ax_brake*h_cg/(2*wb)) + (DF_front/2);
w_fr = (m_f*g/2) - (del_w_f) - (mass*ax_brake*h_cg/(2*wb)) + (DF_front/2);
w_rl = (m_r*g/2) + (del_w_r) + (mass*ax_brake*h_cg/(2*wb)) + (DF_rear/2);
w_rr = (m_r*g/2) - (del_w_r) + (mass*ax_brake*h_cg/(2*wb)) + (DF_rear/2);

% Wheel Forces
[fy_fl, fx_fl] = MF52_Combined(kappa_fl,alpha_fl,w_fl,0);
[fy_fr, fx_fr] = MF52_Combined(kappa_fr,alpha_fr,w_fr,0);
[fy_rl, fx_rl] = MF52_Combined(kappa_rl,alpha_rl,w_rl,0);
[fy_rr, fx_rr] = MF52_Combined(kappa_rr,alpha_rr,w_rr,0);

% Vehicle Acceleration
ay_out = (fy_fl + fy_fr + fy_rl + fy_rr)/mass;
ax_out = (fx_fl + fx_fr + fx_rl + fx_rr - Fd)/mass;

Vx = abs(sqrt(prev_velocity^2 - 2*(ax_out)*sector_dist));

opt_out = -Vx; % Maximise Existing Sector Velocity (Moving Backwards)

end