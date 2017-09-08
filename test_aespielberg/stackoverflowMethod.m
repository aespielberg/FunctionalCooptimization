[m, n] = size(df);

dfstring = cell(m, n);

for i = 1:1:m
    i
    for j = 1:1:n
        j
        dfstring{i,j}=char(vpa(df(i,j)));
    end
end

fid = fopen('dynamics_jacobianAnt.c', 'w');


%print header:
fprintf(fid, '#include "mex.h"\n');
fprintf(fid, '#include <math.h>\n');
fprintf(fid, 'void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){\n');
fprintf(fid, 'double * h = mxGetPr(prhs[0]);\n');
fprintf(fid, 'double * x = mxGetPr(prhs[1]);\n');
fprintf(fid, 'double * u = mxGetPr(prhs[2]);\n');
fprintf(fid, 'double * l = mxGetPr(prhs[3]);\n');
fprintf(fid, 'double * jl = mxGetPr(prhs[4]);\n');
fprintf(fid, 'double * params = mxGetPr(prhs[5]);\n');

%TODO: unhardcode these things - extract from an input plant
num_state = 24;
num_inp = 6;
num_l = 36;
num_lj = 0;

params = {'BODY_MASS', 'LEG_MASS', 'BASE_LINK_EXTENTS_X', 'BASE_LINK_EXTENTS_Y', 'BASE_LINK_EXTENTS_Z', ... 
'LEG_LINK_EXTENTS_X', 'LEG_LINK_EXTENTS_Y', 'LEG_LINK_EXTENTS_Z', 'BASE_ORIGIN_X', 'BASE_ORIGIN_Y',  'BASE_ORIGIN_Z'};

fprintf(fid, 'double h1 = h[0];\n');

for i = 1:1:num_state
   fprintf(fid, 'double x%d = x[%d];\n', i, i-1 ) ;
end

for i = 1:1:num_inp
   fprintf(fid, 'double u%d = u[%d];\n', i, i-1 ) ;
end

for i = 1:1:num_l
   fprintf(fid, 'double l%d = l[%d];\n', i, i-1 ) ;
end
    
for i = 1:1:length(params)
   fprintf(fid, 'double %s = params[%d];\n', params{i}, i-1 );
end


%TODO: replace appropriate ^ and cos/sin


fprintf(fid, 'double * df = mxGetPr(plhs[0]);\n');
fprintf(fid, 'int M = 24;\n'); %TODO: unhardcode this


states = 24;

dict = cell(states*2, 2);




for k = 1:1:states
    dict{k, 1} = sprintf('cos(x%d/2)', k);
    dict{k, 2} = sprintf('c%d', k);
    dict{k + states, 1} = sprintf('sin(x%d/2)', k);
    dict{k + states, 2} = sprintf('s%d', k);
    fprintf(fid, sprintf('double s%d = sin(x%d/2);\n', k, k));
    fprintf(fid, sprintf('double c%d = cos(x%d/2);\n', k, k));
end



for i = 1:1:m
    i
    for j = 1:1:n
        j
        modfstr = fstring{i, j};
        for k = 1:1:states*2
            modfstr = strrep(modfstr, dict{k, 1}, dict{k, 2});
        end
        fprintf(fid, 'df[%d + M * %d] = %s;\n', i-1, j-1, modfstr);
    end
end

fprintf(fid, '}');
fclose(fid);


% vars = {};
% for i = 1:1:m
%      i
%      for j = 1:1:n
%          j
%          vars = union(vars, symvar(fstring{i, j}));
%      end
% end