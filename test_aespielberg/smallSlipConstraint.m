function [f, df] = smallSlipConstraint(l)

num_el = length(l);
f = zeros(num_el / 6, 1);
df = zeros(num_el / 6, num_el);

for i = 1:1:num_el / 6
    
    sub_vector = l(6 * (i-1) + 2: 6*(i-1) + 4);
    f(i) = 0.5 * norm(sub_vector)^2;
    df(i, 6 * (i-1) + 2: 6*(i-1) + 4) = l(6 * (i-1) + 2: 6*(i-1) + 4);
end
 

end