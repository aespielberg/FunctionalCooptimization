%% runs trajectory optimization and animates open-loop playback
megaclear;

p = RigidBodyManipulator('LongArm.urdf');
%p = RigidBodyManipulator('AcrobotTest.urdf');

nX = p.getNumOutputs();
x0 = zeros(nX,1);
tf0 = 4;


N = 8; %TODO: bump this back up


options.actuation_weight = 1.0;

prog = DircolTrajectoryOptimizationTest(p,N,[0 6], options);
prog = prog.setRegularizerWeight(0.0);
options.penalty = 2;
prog = prog.setOptimizationType(options);

thresh = 0.000;
prog = prog.addStateConstraint(ConstantConstraint(x0),1);
constraint = constructPositionDiffConstraint( [-thresh; -inf; -thresh; -inf; -inf; -inf]...
    ,[thresh; inf; thresh; inf; inf; inf],p, 'link5', [0; 0; 0.0],...
    [0; 0; 10; 0; 0; 0]);




%constraint.grad_method = 'numerical';
%constraint.grad_level = 0;

x_inds = prog.x_inds(:);
last_x_inds = x_inds(end-3:end);
prog = prog.addConstraint(constraint,  [last_x_inds; prog.param_inds(:)]);

 

%prog = prog.addRunningCost(@torqueCost);




prog = prog.setSolverOptions('snopt','MajorIterationsLimit',2000);
prog = prog.setSolverOptions('snopt','MinorIterationsLimit',200000);
prog = prog.setSolverOptions('snopt','IterationsLimit',2000000);


%prog = prog.addStateConstraint(ConstantConstraint(xf),N);
%prog = prog.addRunningCost(@cost);
%prog = prog.addFinalCost(@finalCost);

traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(x0)]));
%traj_init.u = PPTrajectory(foh(0:0.1:tf0, 0.01*sin(0:0.1:tf0)));

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

p = p.setParams(z(end-2:end-1)); %TODO: make this cleaner


v2 = p.constructVisualizer();
v2.playback(xtraj);
%v.playback(xtraj);

