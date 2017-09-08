function [f, df] = nonlinear_compl_combinedQuadruped(y)
f = nonlinear_complQuadruped(y);
df = nonlinear_compl_jacobianQuadruped(y);
end