function [f, df] = nonlinear_compl_combinedAnt(y)
f = nonlinear_complAnt(y);
df = nonlinear_compl_jacobianAnt(y);
end