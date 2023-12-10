function H = hessfinal(X,lambda)
% Call the function hessenergy to start
X = X';
H = objectiveHessian(X);

% Add the Lagrange multipliers * the constraint Hessians
H = H + hessc1(X) * lambda.ineqnonlin(1);
H = H + hessc2(X) * lambda.ineqnonlin(2);
H = H + hessc3(X) * lambda.ineqnonlin(3);
H = H + hessc4(X) * lambda.ineqnonlin(4);
H = H + hessc5(X) * lambda.ineqnonlin(5);
H = H + hessc6(X) * lambda.ineqnonlin(6);
H = H + hessc7(X) * lambda.ineqnonlin(7);
H = H + hessc8(X) * lambda.ineqnonlin(8);
H = H + hessc9(X) * lambda.ineqnonlin(9);
% H = H + hessc10(X) * lambda.ineqnonlin(10);
% H = H + hessc11(X) * lambda.ineqnonlin(11);
% H = H + hessc12(X) * lambda.ineqnonlin(12);
% H = H + hessc13(X) * lambda.ineqnonlin(13);
% H = H + hessc14(X) * lambda.ineqnonlin(14);
% H = H + hessc15(X) * lambda.ineqnonlin(15);
% H = H + hessc16(X) * lambda.ineqnonlin(16);
% H = H + hessc17(X) * lambda.ineqnonlin(17);
% H = H + hessc18(X) * lambda.ineqnonlin(18);
% H = H + hessc19(X) * lambda.ineqnonlin(19);
% H = H + hessc20(X) * lambda.ineqnonlin(20);
% H = H + hessc21(X) * lambda.ineqnonlin(21);

H = H + hessceq1(X) * lambda.eqnonlin(1);
H = H + hessceq2(X) * lambda.eqnonlin(2);
H = H + hessceq3(X) * lambda.eqnonlin(3);
H = H + hessceq4(X) * lambda.eqnonlin(4);
H = H + hessceq5(X) * lambda.eqnonlin(5);
H = H + hessceq6(X) * lambda.eqnonlin(6);
% H = H + hessceq7(X) * lambda.eqnonlin(7);

end

