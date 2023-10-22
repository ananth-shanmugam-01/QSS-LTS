% Friction Ellipse Model

function [FY,FX] = FrictionEllipseModel(carData,FZ,inputMUY,inputMUX)

    maxMUY = carData.Tyre.LambdaMUY * (carData.Tyre.refMUY + carData.Tyre.muLoadSensitivity*(FZ - carData.Tyre.refFZ));
    maxMUX = carData.Tyre.LambdaMUX * (carData.Tyre.refMUX + carData.Tyre.muLoadSensitivity*(FZ - carData.Tyre.refFZ));
    
    % Equation of Ellipse - muy^2 / maxMUY^2 + mux^2 / maxMUX^2 = 1;
    MUXellipse  = @(muy) abs(sqrt((maxMUX^2) * (1 - muy^2 / maxMUY^2))); 
    MUYellipse  = @(mux) abs(sqrt((maxMUY^2) * (1 - mux^2 / maxMUX^2)));

    FX = FZ * MUXellipse(inputMUY);
    FY = FZ * MUYellipse(inputMUX);
end














