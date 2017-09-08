function f = parametricDynamics(p, t, x, u, params)
    p = p.setParams(params);
    
    f = p.dynamics(t, x, u);
end