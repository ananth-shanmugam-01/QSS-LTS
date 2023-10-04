function [c,ceq] = nonlcon_cornering(control_variables, curvature)

global mass g h_cg wd m_f m_r t wb a b whl_radius Izz V_max ...
    gear_ratio n_motor pt_eff AMK_21Nm_Trq AMK_Motor_RPM AMK_Motor_Torque...
    CLA CDA aero_balance

global unsprung_mass h_cg_unsprung mech_balance h_cg_roll_axis rc_f rc_r...
    sprung_mass sprung_mass_front sprung_mass_rear

Vx = control_variables(1);
del = control_variables(2);
beta = control_variables(3);
kt = curvature;

yaw_rate = Vx*kt;
ay = kt*Vx^2;
V_max = 33; % m/s

DF_total = 0.5*1.225*CLA*Vx^2;
DF_front = aero_balance*DF_total;
DF_rear = (1-aero_balance)*DF_front;

% Slip Angles

alpha_fl = ((Vx*tan(deg2rad(beta)) + a*yaw_rate) / (Vx + yaw_rate*t*0.5)) - deg2rad(del);
alpha_fr = ((Vx*tan(deg2rad(beta)) + a*yaw_rate) / (Vx - yaw_rate*t*0.5)) - deg2rad(del);
alpha_rl = (Vx*tan(deg2rad(beta)) - b*yaw_rate) / (Vx + yaw_rate*t*0.5);
alpha_rr = (Vx*tan(deg2rad(beta)) - b*yaw_rate) / (Vx - yaw_rate*t*0.5);

% Slip Ratios

kappa_fl = 0;
kappa_fr = 0;
kappa_rl = 0;
kappa_rr = 0;

% Lateral Load Transfer
del_w_f = (sprung_mass*ay*h_cg_roll_axis*mech_balance/t) + (sprung_mass_front*ay*rc_f/t) + (unsprung_mass*h_cg_unsprung*ay/t);
del_w_r = (sprung_mass*ay*h_cg_roll_axis*(1-mech_balance)/t) + (sprung_mass_rear*ay*rc_r/t) + (unsprung_mass*h_cg_unsprung*ay/t);

% Wheel Loads
w_fl = (m_f*g/2) + (del_w_f) + (DF_front/2); % - (mass*ax*h_cg/(2*wb));
w_fr = (m_f*g/2) - (del_w_f) + (DF_front/2); % - (mass*ax*h_cg/(2*wb));
w_rl = (m_r*g/2) + (del_w_r) + (DF_rear/2); % + (mass*ax*h_cg/(2*wb));
w_rr = (m_r*g/2) - (del_w_r) + (DF_rear/2); % + (mass*ax*h_cg/(2*wb));

% Wheel Forces
[fy_fl, fx_fl] = MF52_Combined(kappa_fl,alpha_fl,w_fl,0);
[fy_fr, fx_fr] = MF52_Combined(kappa_fr,alpha_fr,w_fr,0);
[fy_rl, fx_rl] = MF52_Combined(kappa_rl,alpha_rl,w_rl,0);
[fy_rr, fx_rr] = MF52_Combined(kappa_rr,alpha_rr,w_rr,0);

% Vehicle Acceleration
ay_out = (fy_fl + fy_fr + fy_rl + fy_rr)/mass;

c(1) = Vx - V_max; % Below Maximum Forward Velocity
c(2) = abs(alpha_fl) - abs(deg2rad(10)); % Measured Slip Angle Limits
c(3) = abs(alpha_fr) - abs(deg2rad(10));
c(4) = abs(alpha_rl) - abs(deg2rad(10));
c(5) = abs(alpha_rr) - abs(deg2rad(10));
c(6) = -w_fl; % Positive Wheel Loads
c(7) = -w_fr;
c(8) = -w_rl;
c(9) = -w_rr;

ceq(1) = (a*(fy_fl + fy_fr) - b*(fy_rl + fy_rr))/Izz; % Yaw Acceleration = 0, for quasi-steady state
ceq(2) = sign(kt) + sign(alpha_fl); % Control sign of slip angle
ceq(3) = sign(kt) + sign(alpha_fr);
ceq(4) = sign(kt) + sign(alpha_rl);
ceq(5) = sign(kt) + sign(alpha_rr);
ceq(6) = ay - ay_out; % Converge between input and out Ay
ceq(7) = (kt*Vx^2) - ay; % Relationship between Vx and Ay
ceq(8) = Vx*kt - yaw_rate; % Relationship between Vx and Yaw Rate