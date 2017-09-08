function fToM(f, plant, nContactForces)

n = length(f);
fstring = cell(n, 1);

for i = 1:1:n
    i
    fstring{i}=char(vpa(f(i), 16));
end

name = plant.name{1};
c_file_name = sprintf('dynamics_function%s.c', name);
fid = fopen(c_file_name, 'w');


%print header:
fprintf(fid, '#include "mex.h"\n');
fprintf(fid, '#include <math.h>\n');
fprintf(fid, 'void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){\n');
fprintf(fid, 'double * h = mxGetPr(prhs[0]);\n');
fprintf(fid, 'double * x0 = mxGetPr(prhs[1]);\n');
fprintf(fid, 'double * x1 = mxGetPr(prhs[2]);\n');
fprintf(fid, 'double * u = mxGetPr(prhs[3]);\n');
fprintf(fid, 'double * l = mxGetPr(prhs[4]);\n');
fprintf(fid, 'double * jl = mxGetPr(prhs[5]);\n');
fprintf(fid, 'double * params = mxGetPr(prhs[6]);\n');

%TODO: unhardcode these things - extract from an input plant
num_state = plant.getNumOutputs();
num_inp = plant.getNumInputs();
num_l = nContactForces;
num_lj = sum(abs(plant.getJointLimits()) ~= Inf)/2;

paramsSym = sym(plant.getParamFrame().getCoordinateNames(), 'real');
params = cell(length(paramsSym), 1);
for i = 1:1:length(paramsSym)
    params{i} = char(paramsSym(i)); 
end

fprintf(fid, 'double h1 = h[0];\n');

for i = 1:1:num_state
   fprintf(fid, 'double x0%d = x0[%d];\n', i, i-1 ) ;
end

for i = 1:1:num_state
   fprintf(fid, 'double x1%d = x1[%d];\n', i, i-1 ) ;
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

setZeroLine = sprintf('plhs[0] = mxCreateDoubleMatrix(%d,1,mxREAL);\n', n);
fprintf(fid, setZeroLine);
fprintf(fid, 'double * df = mxGetPr(plhs[0]);\n');


states = num_state;

dict = cell(states*2, 2);



for k = 1:1:states
    dict{k, 1} = sprintf('cos(0.5*x0%d + 0.5*x1%d)', k, k);
    dict{k, 2} = sprintf('c%d', k);
    dict{k + states, 1} = sprintf('sin(0.5*x0%d + 0.5*x1%d)', k, k);
    dict{k + states, 2} = sprintf('s%d', k);
    fprintf(fid, sprintf('double s%d = sin(0.5*x0%d + 0.5*x1%d);\n', k, k, k));
    fprintf(fid, sprintf('double c%d = cos(0.5*x0%d + 0.5*x1%d);\n', k, k, k));
   
end




for i = 1:1:n
    i
    modfstr = fstring{i};
    for k = 1:1:states*2
        modfstr = strrep(modfstr, dict{k, 1}, dict{k, 2});
    end
    fprintf(fid, 'df[%d] = %s;\n', i-1, modfstr);
end


fprintf(fid, '}');
fclose(fid);

disp('done')
% vars = {};
% for i = 1:1:m
%      i
%      for j = 1:1:n
%          j
%          vars = union(vars, symvar(fstring{i, j}));
%      end
% end

end