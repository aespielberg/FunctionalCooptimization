function [f, df] = nonAirborneConstraint(l)

z_up = l(1:6:end);

f = sum(z_up);

df = zeros(1, length(l));

df(1:6:end) = 1;

end