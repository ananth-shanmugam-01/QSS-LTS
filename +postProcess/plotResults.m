function plotResults(outputs, GGV_surf)

% post-process outputs

figure
t = tiledlayout(3,1);
title(t,'QSS Lap Time Simulation')
subtitle(t, 'GGV Outputs')

nexttile
hold on
plot(outputs.dist,outputs.vCar)
ylabel('vCar (m/s)')
xlabel('sLap (m)')
grid on; grid minor; box on;
title(['Lap Time: ', num2str(outputs.time(end)), '(s)'])

nexttile
hold on
plot(outputs.dist,outputs.gLong,'r','DisplayName','Ax')
hold off
grid on; grid minor; box on;
ylabel('Ax (m/s^2)')
xlabel('sLap (m)')

nexttile
hold on
plot(outputs.dist,outputs.gLat,'b','DisplayName','Ay')
hold off
grid on; grid minor; box on;
ylabel('Ay (m/s^2)')
xlabel('sLap (m)')

figure
grid on; grid minor; box on;
surf(GGV_surf(:,:,3),GGV_surf(:,:,2),GGV_surf(:,:,1))
hold on
scatter3(outputs.gLat, outputs.gLong, outputs.vCar,'ro')
hold off
title('GGV Diagram')

end