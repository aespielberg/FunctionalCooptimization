function [f, df] = nonlinear_compl_combinedBiped(y)
f = nonlinear_complBiped(y);
df = nonlinear_compl_jacobianBiped(y);
end