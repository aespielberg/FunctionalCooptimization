function runSwingUpKinematicBP()
%% runs trajectory optimization and animates open-loop playback

p = RigidBodyManipulator('BranchingPendulum.urdf');

x0 = zeros(6,1);
xf = double([pi;0;0;0; 0; 0]);
tf0 = 4;

N = 21;
prog = KinematicDirtranTest(p,N,[0 6]);
prog = prog.setRegularizerWeight(1.0);
options.penalty = 1;
prog = prog.setOptimizationType(options);


prog = prog.addStateConstraint(ConstantConstraint(x0),1);
constraint = constructPositionDiffConstraint( [0; -inf; 0; -inf; -inf; -inf]...
    ,[0; inf; 0; inf; inf; inf],p, 'bottom_link', [0; 0; 0.0],...
    [0; 0; 10; 0; 0; 0]);




constraint.grad_method = 'numerical';
constraint.grad_level = 0;

x_inds = prog.x_inds(:);
last_x_inds = x_inds(end-5:end);
prog = prog.addConstraint(constraint,  [last_x_inds; prog.param_inds(:)]);


constraint = constructPositionDiffConstraint( [0; -inf; 0; -inf; -inf; -inf]...
    ,[0; inf; 0; inf; inf; inf],p, 'lower_link', [0; 0; 0.0],...
    [5; 0; 5; 0; 0; 0]);


constraint.grad_method = 'numerical';
constraint.grad_level = 0;

prog = prog.addConstraint(constraint,  [last_x_inds; prog.param_inds(:)]);




prog = prog.setSolverOptions('snopt','MajorIterationsLimit',2000);
prog = prog.setSolverOptions('snopt','MinorIterationsLimit',200000);
prog = prog.setSolverOptions('snopt','IterationsLimit',2000000);


%prog = prog.addStateConstraint(ConstantConstraint(xf),N);
%prog = prog.addRunningCost(@cost);
%prog = prog.addFinalCost(@finalCost);

traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));

tic
[xtraj,z,F,info] = prog.solveTraj(tf0,traj_init);
toc

p = p.setParams(z(end-6:end)); %TODO: make this cleaner
v2 = p.constructVisualizer()
v2.playback(xtraj);
%v.playback(xtraj);

end
