function opt_out = max_cornering(control_variables,curvature)

global mass g h_cg wd m_f m_r t wb a b whl_radius Izz V_max ...
    gear_ratio n_motor pt_eff AMK_21Nm_Trq AMK_Motor_RPM AMK_Motor_Torque...
    CLA CDA aero_balance

global unsprung_mass h_cg_unsprung mech_balance h_cg_roll_axis rc_f rc_r...
    sprung_mass sprung_mass_front sprung_mass_rear

% Input Values
Vx = control_variables(1);
del = control_variables(2);
beta = control_variables(3);

kt = curvature;

yaw_rate = Vx*kt;
ay = kt*Vx^2;

% Vehicle Parameters
mass = 300;
g = 9.81;
h_cg = 0.280;
wd = 0.45; % [-]
m_f = wd*mass; % Newtons
m_r = (1-wd)*mass; % Newtons
t = 1.23; 
wb = 1.535;
a = (1-wd)*wb;
b = (wd)*wb;

DF_total = 0.5*1.225*CLA*Vx^2;
DF_front = aero_balance*DF_total;
DF_rear = (1-aero_balance)*DF_front;

% Slip Angles
alpha_fl = ((Vx*tan(deg2rad(beta)) + a*yaw_rate) / (Vx + yaw_rate*t*0.5)) - deg2rad(del);
alpha_fr = ((Vx*tan(deg2rad(beta)) + a*yaw_rate) / (Vx - yaw_rate*t*0.5)) - deg2rad(del);
alpha_rl = (Vx*tan(deg2rad(beta)) - b*yaw_rate) / (Vx + yaw_rate*t*0.5);
alpha_rr = (Vx*tan(deg2rad(beta)) - b*yaw_rate) / (Vx - yaw_rate*t*0.5);

% Slip Ratios
% Simplification for Pure Lateral Slip at Apex
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
[fy_fl, ~] = MF52_Combined(kappa_fl,alpha_fl,w_fl,0);
[fy_fr, ~] = MF52_Combined(kappa_fr,alpha_fr,w_fr,0);
[fy_rl, ~] = MF52_Combined(kappa_rl,alpha_rl,w_rl,0);
[fy_rr, ~] = MF52_Combined(kappa_rr,alpha_rr,w_rr,0);


% Vehicle Acceleration
ay_out = (fy_fl + fy_fr + fy_rl + fy_rr)/mass;

opt_out = -Vx; % Maximise velocity through corner
end