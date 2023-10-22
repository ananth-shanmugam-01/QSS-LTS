function [c,ceq] = nonLinearConstraintsGGVCombinedBrakingAx(carData, control_variables, velocity, Ax)

% Control Variables
Vx = velocity;
ax_brake = Ax;

brakePressure = control_variables(1)*100;

FLmux = control_variables(2); % No tractive force at front axle
FRmux = control_variables(3);
RLmux = control_variables(4);
RRmux = control_variables(5);

FrontFYScalar = control_variables(6);
RearFYScalar = control_variables(7);

ay = control_variables(8);

DF_total = 0.5*1.225*carData.Aero.CLA*Vx^2;
DF_front = carData.Aero.rAeroBalance*DF_total;
DF_rear = (1-carData.Aero.rAeroBalance)*DF_front;

% Fd = 0.5*1.225*carData.Aero.CDA*Vx^2; % Drag influence not used in load transfer

% Braking Torque
% Brake Model with brake pressure as input and brake bias
Front_Brake_Force = 2*(brakePressure * carData.Brakes.rBrakeBias .*10^5 .*carData.Brakes.areaPiston .*carData.Brakes.nPistonFront) * carData.Brakes.muBrakePad * carData.Brakes.radBrakeDisc / carData.Chassis.radWheel; %N
Rear_Brake_Force = 2*(brakePressure*(1-carData.Brakes.rBrakeBias) .*10^5 .*carData.Brakes.areaPiston .*carData.Brakes.nPistonFront) * carData.Brakes.muBrakePad * carData.Brakes.radBrakeDisc / carData.Chassis.radWheel; %N

g = 9.81;

% Lateral Load Transfer
del_w_f = (carData.Chassis.SprungMass * ay * carData.Suspension.heightCG2rollAxis * carData.Suspension.mechanicalBalance/carData.Chassis.trackWidth)...
    + (carData.Chassis.sprungMassFront* ay * carData.Suspension.rollCentreFront / carData.Chassis.trackWidth) + (carData.Chassis.unsprungMass * carData.Chassis.heightUnsprungCOG * ay / carData.Chassis.trackWidth);
del_w_r = (carData.Chassis.SprungMass * ay * carData.Suspension.heightCG2rollAxis * (1-carData.Suspension.mechanicalBalance)/carData.Chassis.trackWidth)...
    + (carData.Chassis.sprungMassRear * ay * carData.Suspension.rollCentreRear / carData.Chassis.trackWidth) + (carData.Chassis.unsprungMass * carData.Chassis.heightUnsprungCOG * ay / carData.Chassis.trackWidth);

% Longitudinal Load Transfer
longLT = carData.Chassis.mass * ax_brake * carData.Chassis.heightSprungCOG / (2 * carData.Chassis.wheelBase);

% Wheel Loads 
w_fl = (carData.Chassis.massFront * g / 2) + (del_w_f) - longLT + (DF_front/2);
w_fr = (carData.Chassis.massFront * g / 2) - (del_w_f) - longLT + (DF_front/2);
w_rl = (carData.Chassis.massRear * g / 2) + (del_w_r) + longLT + (DF_rear/2);
w_rr = (carData.Chassis.massRear * g / 2) - (del_w_r) + longLT + (DF_rear/2);

% Wheel Forces
% [MUY,MUX] = FYFrictionEllipse(carData,FZ,inputMUX)
[fyFL,fxFL] = FYFrictionEllipse(carData,w_fl,FLmux);
[fyFR,fxFR] = FYFrictionEllipse(carData,w_fr,FRmux);
[fyRL,fxRL] = FYFrictionEllipse(carData,w_rl,RLmux);
[fyRR,fxRR] = FYFrictionEllipse(carData,w_rr,RRmux);

% Lateral Saturation Scalars
FYAxleFront = FrontFYScalar*(fyFL + fyFR);
FYAxleRear = RearFYScalar*(fyRL + fyRR);

% Longitudinal Saturation Scalars
ScaledFX = (fxFL + fxFR + fxRL + fxRR);

% Vehicle Acceleration
ax_out = -ScaledFX/carData.Chassis.mass; % Meet tractive demand
ay_out = (FYAxleFront + FYAxleRear)/carData.Chassis.mass;

optAy = -ay_out; % Maximise Lateral Acceleration

% Inequality Constraints
c(1) = -w_fl; % Positive Wheel Loads
c(2) = -w_fr;
c(3) = -w_rl;
c(4) = -w_rr;
c(5) = -brakePressure;

ceq(1) = (0.5*carData.Chassis.trackWidth*(fxFL + fxRL) - 0.5*carData.Chassis.trackWidth*(fxFR + fxRR))/carData.Chassis.yawInertia;
ceq(2) = (carData.Chassis.frontMomentArm*(FYAxleFront) - carData.Chassis.rearMomentArm*(FYAxleRear))/carData.Chassis.yawInertia;
ceq(3) = Ax - ax_out;
ceq(4) = ay - ay_out;
% ceq(5) = carData.Brakes.rBrakeBias - ((fxFL + fxFR) / (fxFL + fxFR + fxRL + fxRR));

end