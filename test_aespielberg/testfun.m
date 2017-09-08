function f = testfun(cnstr, filename)

load(filename)

%f = cnstr.eval(1, x0, x1, u, l, [])
%f = cnstr([y; 1])
f = cnstr([compl_vec; 1])
end