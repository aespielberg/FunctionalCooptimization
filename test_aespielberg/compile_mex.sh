clang -c -DMX_COMPAT_32   -D_GNU_SOURCE -DMATLAB_MEX_FILE  -I"/usr/local/MATLAB/R2015a/extern/include" -I"/usr/local/MATLAB/R2015a/simulink/include" -ansi -fexceptions -fPIC -fno-omit-frame-pointer -pthread -O -DNDEBUG /home/aespielberg/drake_unparameterized/drake-distro/drake/examples/Acrobot/cooptimization/test_aespielberg/$1.c -o $1.o
clang -pthread -Wl,--no-undefined -Wl,-rpath-link,/usr/local/MATLAB/R2015a/bin/glnxa64 -shared  -O -Wl,--version-script,"/usr/local/MATLAB/R2015a/extern/lib/glnxa64/mexFunction.map" $1.o   -L"/usr/local/MATLAB/R2015a/bin/glnxa64" -lmx -lmex -lmat -lm -lstdc++ -o $1.mexa64
rm -f $1.o