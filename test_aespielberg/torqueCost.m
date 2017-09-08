function [g,dg] = torqueCost(dt,x,u, params)
    R = 1;
    g = sum((R*u).*u,1);
    dg = [zeros(1,1+size(x,1)),2*u'*R, zeros(1, length(params))];

    return;
end