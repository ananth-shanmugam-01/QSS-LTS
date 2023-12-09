%% Tire Model Test
addpath('C:\Users\Ananth\Desktop\Local LTS Git Folder\QSS-LTS\')
%%
sa_range = linspace(deg2rad(-12),deg2rad(12),50);
sr_range = linspace(-0.2,0.2,50);
[SA, SL] = meshgrid(sa_range,sr_range);
SA = SA(:);
SL = SL(:);
FZ = 1100.*ones(numel(SA),1);
GAMMA = deg2rad(0).*ones(numel(SA),1);

[FY,FX] = MF52_Combined(SL,SA,FZ,GAMMA);

figure
plot(FY,FX,'.')
