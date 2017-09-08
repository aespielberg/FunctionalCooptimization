function [f, df] = nonlinear_compl_combinedAntSeparateLegs(y)
f = nonlinear_complAntSeparateLegs(y);
df = nonlinear_compl_jacobianAntSeparateLegs(y);
end