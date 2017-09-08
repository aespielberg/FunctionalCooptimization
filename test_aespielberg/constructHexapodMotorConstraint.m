function constraint = constructHexapodMotorConstraint(param_size, u_size, N)

%keep it positive
lb = zeros(u_size * N * 4, 1);
ub = inf(u_size * N * 4, 1);

constraint = FunctionHandleConstraint(lb, ub, u_size*N + param_size, @hexapodMotorConstraint);
constraint.grad_method = 'numerical';
constraint.grad_level = 0;
    
end