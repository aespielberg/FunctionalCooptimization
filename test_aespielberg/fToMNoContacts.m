function fToMNoContacts(f, plant, nContactForces)

n = length(f);
fstring = cell(n, 1);

for i = 1:1:n
    fstring{i}=char(vpa((f(i)), 16));
    %     fstring{i} = strrep(fstring{i}, '.*', '*');
    %     fstring{i} = strrep(fstring{i}, './', '/');
    %     fstring{i} = strrep(fstring{i}, '.^', '^');
    %     fstring{i} = strrep(fstring{i}, 'e', '*10^');
    %     fstring{i} = strrep(fstring{i}, '_', '');
    %     math('<<~/mathematica_for_matlab/ToMatlab.m');
    %     math(['A2=ToExpression["' fstring{i} '",TraditionalForm]']);
    %     math('Afullsimp=FullSimplify[A2]');
    %     fstring{i}=math('ToMatlab[Afullsimp]');
    %     math('quit')
    
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
fprintf(fid, 'double * u0 = mxGetPr(prhs[3]);\n');
fprintf(fid, 'double * u1 = mxGetPr(prhs[4]);\n');
fprintf(fid, 'double * params = mxGetPr(prhs[5]);\n');

%TODO: unhardcode these things - extract from an input plant
num_state = plant.getNumOutputs();
num_inp = plant.getNumInputs();

%params = {'BODY_MASS', 'LEG_MASS', 'BASE_LINK_EXTENTS_X', 'BASE_LINK_EXTENTS_Y', 'BASE_LINK_EXTENTS_Z', ...
%'LEG_LINK_EXTENTS_X', 'LEG_LINK_EXTENTS_Y', 'LEG_LINK_EXTENTS_Z', 'BASE_ORIGIN_X', 'BASE_ORIGIN_Y',  'BASE_ORIGIN_Z'};

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
    fprintf(fid, 'double u0%d = u0[%d];\n', i, i-1 ) ;
end

for i = 1:1:num_inp
    fprintf(fid, 'double u1%d = u1[%d];\n', i, i-1 ) ;
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
    fstring{i} = modfstr;
end

for i = 1:1:n
    i
    modfstr = fstring{i};
    modfstr = strrep(modfstr, '*', ' * ');
    fstring{i} = modfstr;
end
    

%Now let's create a second dictionary, to compute common values and try to
%precompute these values.
dict = {};

cntr = 1;
for q = 1:1:states
    for p = 1:1:states
        dict{cntr, 1} = sprintf(' c%d * c%d ', q, p);
        dict{cntr, 2} = sprintf(' c%dc%d ', q, p);
        cntr = cntr + 1;
        fprintf(fid, sprintf('double c%dc%d = c%d * c%d;\n', q, p, q, p));
        dict{cntr, 1} = sprintf(' c%d * s%d ', q, p);
        dict{cntr, 2} = sprintf(' c%ds%d ', q, p);
        cntr = cntr + 1;
        fprintf(fid, sprintf('double s%dc%d = s%d * c%d;\n', q, p, q, p));
        dict{cntr, 1} = sprintf(' s%d * c%d ', q, p);
        dict{cntr, 2} = sprintf(' s%dc%d ', q, p);
        cntr = cntr + 1;
        fprintf(fid, sprintf('double c%ds%d = c%d * s%d;\n', q, p, q, p));
        dict{cntr, 1} = sprintf(' s%d * s%d ', q, p);
        dict{cntr, 2} = sprintf(' s%ds%d ', q, p);
        fprintf(fid, sprintf('double s%ds%d = s%d * s%d;\n', q, p, q, p));
        cntr = cntr + 1;
        

    end
    
    for power = 1:8
        dict{cntr, 1} = sprintf(' c%d^%d ', q, power);
        dict{cntr, 2} = sprintf(' c%dpow%d ', q, power);
        fprintf(fid, sprintf('double c%dpow%d = pow(c%d, %d);\n', q, power, q, power));
        cntr = cntr + 1;
        
        dict{cntr, 1} = sprintf(' s%d^%d ', q, power);
        dict{cntr, 2} = sprintf(' s%dpow%d ', q, power);
        fprintf(fid, sprintf('double s%dpow%d = pow(s%d, %d);\n', q, power, q, power));
        cntr = cntr + 1;
    end
end

for i = 1:1:n
    modfstr = fstring{i};
    for k = 1:1:size(dict, 1)
        modfstr = strrep(modfstr, dict{k, 1}, dict{k, 2});
    end
    fstring{i} = modfstr;
end

%Now, finally, let's create our function:

for i = 1:1:n
    
    fprintf(fid, 'df[%d] = %s;\n', i-1, fstring{i});
    
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