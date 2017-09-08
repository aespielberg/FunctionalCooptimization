function [f, df] = nonlinear_compl_combinedQuadrupedFull(y)
f = nonlinear_complQuadrupedFull(y);
df = nonlinear_compl_jacobianQuadrupedFull(y);
end