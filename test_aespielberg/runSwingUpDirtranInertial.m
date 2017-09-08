%% runs trajectory optimization and animates open-loop playback
megaclear;

p = RigidBodyManipulator('AcrobotWParamsInertial.urdf');

x0 = zeros(p.getNumStates(),1);
xf = x0;
xf(1) = pi;
tf0 = 4;

N = 6; 
prog = DirtranTrajectoryOptimizationTest(p,N,[0 6]);
prog = prog.setRegularizerWeight(1.0);
options.penalty = 2;
prog = prog.setOptimizationType(options);

params = p.getParams();

thresh = 0.000;
prog = prog.addStateConstraint(ConstantConstraint(x0),1);
constraint = constructPositionDiffConstraint( [-thresh; -inf; -thresh; -inf; -inf; -inf]...
    ,[thresh; inf; thresh; inf; inf; inf],p, 'lower_link', [0; 0; 0.0],...
    [0; 0; params(1) + params(2)/2.0; 0; 0; 0]);

x_inds = prog.x_inds(:);
last_x_inds = x_inds(end-3:end);
prog = prog.addConstraint(constraint,  [last_x_inds; prog.param_inds(:)]);

%prog = prog.addStateConstraint(ConstantConstraint(xf),N);

%constraint_upper = constructInertialConstraint(p, 'upper_link',1);
%constraint_lower = constructInertialConstraint(p, 'lower_link',2);

%constraint.grad_method = 'numerical';
%constraint.grad_level = 0;

%x_inds = prog.x_inds(:);
%last_x_inds = x_inds(end-3:end);
%prog = prog.addConstraint(constraint_upper, prog.param_inds(:));
%prog = prog.addConstraint(constraint_lower, prog.param_inds(:));
 

%prog = prog.addRunningCost(@torqueCost);


prog = prog.setSolverOptions('snopt','MajorIterationsLimit',2000);
prog = prog.setSolverOptions('snopt','MinorIterationsLimit',200000);
prog = prog.setSolverOptions('snopt','IterationsLimit',2000000);


%prog = prog.addStateConstraint(ConstantConstraint(xf),N);
%prog = prog.addRunningCost(@cost);
%prog = prog.addFinalCost(@finalCost);

traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));

%rng(6);
traj_init.u = PPTrajectory(foh(linspace(0, tf0, N), randn(size(linspace(0, tf0, N)))));

%traj_init.u = PPTrajectory(foh(0:0.1:tf0, 10*ones(size(0:0.1:tf0))));

tic
[xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
toc

% p = p.setParams(z(end-1:end)); %TODO: make this cleaner
% 
% %%%%%%%%%%%%%%%%%%%%second run%%%%%%%%%%%%%%%%%%%%%%%
% 
% prog = DirtranTrajectoryOptimizationTest(p,N,[0 6]);
% prog = prog.setRegularizerWeight(100.0);
% options.penalty = 2;
% prog = prog.setOptimizationType(options);
% 
% thresh = 0.000;
% prog = prog.addStateConstraint(ConstantConstraint(x0),1);
% constraint = constructPositionDiffConstraint( [-thresh; -inf; -thresh; -inf; -inf; -inf]...
%     ,[thresh; inf; thresh; inf; inf; inf],p, 'lower_link', [0; 0; 0.0],...
%     [0; 0; 10; 0; 0; 0]);
% 
% 
% 
% 
% constraint.grad_method = 'numerical';
% constraint.grad_level = 0;
% 
% x_inds = prog.x_inds(:);
% last_x_inds = x_inds(end-3:end);
% prog = prog.addConstraint(constraint,  [last_x_inds; prog.param_inds(:)]);
% 
%  
% 
% prog = prog.addRunningCost(@torqueCost);
% 
% 
% 
% 
% prog = prog.setSolverOptions('snopt','MajorIterationsLimit',2000);
% prog = prog.setSolverOptions('snopt','MinorIterationsLimit',200000);
% prog = prog.setSolverOptions('snopt','IterationsLimit',2000000);
% 
% 
% %prog = prog.addStateConstraint(ConstantConstraint(xf),N);
% %prog = prog.addRunningCost(@cost);
% %prog = prog.addFinalCost(@finalCost);
% 
% traj_init.x = xtraj;
% traj_init.u = utraj;
% 
% tic
% [xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
% toc

p = p.setParams(z(prog.param_inds)); %TODO: make this cleaner

save('outputInertial.mat','p','xtraj','utraj');
v2 = p.constructVisualizer();
v2.playback(xtraj);
%v.playback(xtraj);

F