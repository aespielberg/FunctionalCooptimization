%% runs trajectory optimization and animates open-loop playback
megaclear;

%options.terrain = RigidBodyFlatTerrain();
options.floating = true;
p = RigidBodyManipulator('Quadcopter.urdf',options);
%ob1 = RigidBodyBox([0.5,0.5,0.5],[-1;0;5],[0;0;0]);
%ob1.c = [83,53,10]/255;  % brown
%p = p.addGeometryToBody('world',ob1, 'wall');
%p = compile(p);
%ob2 IS in the plane of the robot
%ob2 = RigidBodyBox([0.5,0.5,0.5],[1;0;2],[0;0;0]);
%ob2.c = [83,53,10]/255;  % brown
%p = p.addGeometryToBody('world',ob2, 'wall');
%p = compile(p);


x0 = zeros(p.getNumStates(),1);
xf = zeros(p.getNumStates(),1);
xf(1) = 1.0;
xf(2) = 1.0;
xf(3) = 1.0;
tf0 = 4;


N = 6; %TODO: bump this back up
prog = DirtranTrajectoryOptimizationTest(p,N,[0 6]);
prog = prog.setRegularizerWeight(0.0);
options.penalty = 2;
options.actuation_weight = 0.0;
prog = prog.setOptimizationType(options);

thresh = 0.000;
prog = prog.addStateConstraint(ConstantConstraint(x0),1);
prog = prog.addStateConstraint(ConstantConstraint(xf),N);

% ob_constraint = constructObstacleConstraint( 0,0,p, 'link2', [0; 0; 0.0],...
%     [1; 0; 2],1);
% 
% prog = prog.addConstraint(ob_constraint,[last_x_inds;prog.param_inds(:)]);

%constraint.grad_method = 'numerical';
%constraint.grad_level = 0;

x_inds = prog.x_inds(:);
numStates = p.getNumStates();
last_x_inds = x_inds(end-numStates+1:end);
%prog = prog.addConstraint(constraint,  [last_x_inds; prog.param_inds(:)]);


%collision_constraint = generateConstraint(MinDistanceConstraint(p,0.2),0);
%prog = prog.addStateConstraint(collision_constraint{1},1:N,1:getNumPositions(p));

%prog = prog.addRunningCost(@torqueCost);
%prog = prog.addRunningCost(@distanceCost);
%prg = prog.addRunningCost(@l1ParamCost);

prog = prog.setSolverOptions('snopt','MajorIterationsLimit',2000);
prog = prog.setSolverOptions('snopt','MinorIterationsLimit',200000);
prog = prog.setSolverOptions('snopt','IterationsLimit',2000000);


%prog = prog.addStateConstraint(ConstantConstraint(xf),N);
%prog = prog.addRunningCost(@cost);
%prog = prog.addFinalCost(@finalCost);
traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));
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

numParams = p.getNumParams();

p = p.setParams(z(end-numParams+1:end)); %TODO: make this cleaner


v2 = p.constructVisualizer();
v2.playback(xtraj);
%v.playback(xtraj);