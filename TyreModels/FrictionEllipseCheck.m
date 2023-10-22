
FZ = 1000;

% friction ellipse at ref load 800 N
muLoadSensitivity = -2.5e-4;
refMUY = 2.4;
refMUX = 2.2;
refFZ = 800;

maxMUY = refMUY + muLoadSensitivity*(FZ - refFZ);
maxMUX = refMUX + muLoadSensitivity*(FZ - refFZ);

% Equation of Ellipse - muy^2 / maxMUY^2 + mux^2 / maxMUX^2 = 1;
MUXellipse  = @(muy) abs(sqrt((maxMUX^2) * (1 - muy^2 / maxMUY^2))); 
MUYellipse  = @(mux) abs(sqrt((maxMUY^2) * (1 - mux^2 / maxMUX^2)));

% create friction circle

muyRange = (-maxMUY:0.001:maxMUY)';
outMUX = zeros(numel(muyRange),1);
for i = 1:numel(muyRange)
    outMUX(i) = MUXellipse(muyRange(i));
end

frictionEllipse = [muyRange, outMUX; flip(muyRange,1), flip(-outMUX,1)];

figure
plot(frictionEllipse(:,1),frictionEllipse(:,2))
axis equal
axis padded
grid on
xlabel('MUY')
ylabel('MUX')

%% Tyre Load Sensitivity Check

FZtest = 200:50:1300;

for i = 1:numel(FZtest)
    TLS_MUY(i) = refMUY + muLoadSensitivity*(FZtest(i) - refFZ);
    TLS_MUX(i) = refMUX + muLoadSensitivity*(FZtest(i) - refFZ);
end

figure
tiledlayout(2,1)

nexttile
plot(FZtest,TLS_MUY)
ylabel('MUY')

nexttile
plot(FZtest,TLS_MUX)
ylabel('MUX')












