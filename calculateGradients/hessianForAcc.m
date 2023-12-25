function H = hessianForAcc(X,lambda)
% Call the function hessenergy to start
X = X';
H = 0; % HessianOfObjectiveFunction(X)

% Add the Lagrange multipliers * the constraint Hessians
H = H + constraints.hessc1(X) * lambda.ineqnonlin(1);
H = H + constraints.hessc2(X) * lambda.ineqnonlin(2);
H = H + constraints.hessc3(X) * lambda.ineqnonlin(3);
H = H + constraints.hessc4(X) * lambda.ineqnonlin(4);
H = H + constraints.hessc5(X) * lambda.ineqnonlin(5);
H = H + constraints.hessc6(X) * lambda.ineqnonlin(6);

H = H + constraints.hessceq1(X) * lambda.eqnonlin(1);
H = H + constraints.hessceq2(X) * lambda.eqnonlin(2);
H = H + constraints.hessceq3(X) * lambda.eqnonlin(3);
H = H + constraints.hessceq4(X) * lambda.eqnonlin(4);

end