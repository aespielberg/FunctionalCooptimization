function constraint = constructActuationConstraint(u_size, suppl_size)
%inf norm rewritten as
%t = suppl
%t - u >= 0
%t + u >= 0
%TODO: for now assume suppl is size 1
lb = zeros(2 * u_size, 1);
ub = inf(2 * u_size, 1); %don't care how high it gets
u = sym('u', [u_size, 1]);
suppl = sym('suppl', [suppl_size, 1]);
f = [suppl * ones(u_size, 1) - u; suppl * ones(u_size, 1) + u];
df = jacobian(f, [u; suppl]);

constraintHandle = matlabFunction(f, df, 'vars', {[u; suppl]});

constraint = FunctionHandleConstraint(lb, ub, (u_size + suppl_size), constraintHandle);


end