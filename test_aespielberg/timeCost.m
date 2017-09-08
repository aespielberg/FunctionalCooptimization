function [g,dg] = timeCost(T,x,params)
    g = T;
    dg = [1,zeros(1, length(x)) zeros(1, length(params))];
end

