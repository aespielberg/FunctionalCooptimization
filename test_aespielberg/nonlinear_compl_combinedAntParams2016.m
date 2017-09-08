function [f, df] = nonlinear_compl_combinedAntParams2016(y)
f = nonlinear_complAntParams2016(y);
df = nonlinear_compl_jacobianAntParams2016(y);
end