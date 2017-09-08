function constraint = constructBipedMotorConstraint(param_size, u_size, N)

%keep it positive
lb = zeros(u_size * N * 5, 1);
ub = inf(u_size * N * 5, 1);

constraint = FunctionHandleConstraint(lb, ub, u_size*N + param_size, @bipedMotorConstraint);
constraint.grad_method = 'numerical';
constraint.grad_level = 0;
    
end