[test_fy, test_fx] = MF52_Combined(0.0126,rad2deg(-0.0401),1400,0)

%%
normalLoadSweep = 300:200:1100;

for i = 1:length(normalLoadSweep)
    [~,fx(i)] = MF52_Combined(0,0,normalLoadSweep(i),0)
end

%%
clear fx
sr_sweep = -0.1:0.01:0.1;
sa_sweep = -deg2rad(8):0.001:deg2rad(8);
normalLoadSweep = 300:200:1100;
[X,Y,Z] = meshgrid(sr_sweep,sa_sweep,normalLoadSweep);
X = X(:);
Y = Y(:);
Z = Z(:);

for i = 1:length(X)
    [fy(i),fx(i)] = MF52_Combined(X(i),Y(i),Z(i),0);
end

figure
plot(fy,fx,'.')
grid on
axis equal

%% control_inputs check
velocity = iterVel
Ax = iterAx

[slipAngles, slipRatios, latForces, longForces, wheelLoads, accelerations] = GGVcheck(carData, control_inputs, velocity, Ax)