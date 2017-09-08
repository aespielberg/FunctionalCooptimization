%% runs trajectory optimization and animates open-loop playback
megaclear;

%options.terrain = RigidBodyFlatTerrain();
p = RigidBodyManipulator('RobotArmWParamsSmall.urdf');

%put limits on the inputs
inputVector = ones(p.getNumInputs(),1);
p.setInputLimits(-10*inputVector,10*inputVector);

x0 = zeros(p.getNumStates(),1);
xf = x0;
xf(1) = pi;
tf0 = 4;

use_l1_norm = false; %if false, use normal regularizer
add_obstacles = false;
use_position_diff_constraint = true; %if false, use xf as final constraint
use_torqe_cost = true; %if false, use no cost
cost_function = 'none'; %torque, torquetime, none

N = 10; %TODO: bump this back up

if add_obstacles
    ob1 = RigidBodyBox([0.5,0.5,0.5],[-1;0;2],[0;0;0]);
    ob1.c = [83,53,10]/255;  % brown
    p = p.addGeometryToBody('world',ob1, 'wall');
    p = compile(p);
    %ob2 = RigidBodyBox([0.5,0.5,0.5],[1;0;4],[0;0;0]);
    %ob2.c = [83,53,10]/255;  % brown
    %p = p.addGeometryToBody('world',ob2, 'wall');
    %p = compile(p);
    disp('added obstacles')
end

prog = DirtranTrajectoryOptimizationTest(p,N,[0 6]);

if add_obstacles
    collision_constraint = generateConstraint(MinDistanceConstraint(p,0.2),0);
    prog = prog.addStateConstraint(collision_constraint{1},1:N,1:getNumPositions(p));
end

if ~use_l1_norm
    disp('using regularizer');
    prog = prog.setRegularizerWeight(100.0);
    options.penalty = 2;
else
    disp('using l1 norm');
    prog = prog.setRegularizerWeight(5000.0);
    options.penalty = 3;
end
prog = prog.setOptimizationType(options);

prog = prog.addStateConstraint(ConstantConstraint(x0),1);

x_inds = prog.x_inds(:);
numStates = p.getNumStates();
last_x_inds = x_inds(end-numStates+1:end);

if ~use_position_diff_constraint
    disp('added xf constraint')
    prog = prog.addStateConstraint(ConstantConstraint(xf),N);
else
    disp('added position diff constraint');
    thresh = 0.000;
    constraint = constructPositionDiffConstraint( [-thresh; -inf; -thresh; -inf; -inf; -inf]...
        ,[thresh; inf; thresh; inf; inf; inf],p, 'link3', [0; 0; 0.0],...
        [0; 0; 6; 0; 0; 0]); %xyz rpy
    prog = prog.addConstraint(constraint,  [last_x_inds; prog.param_inds(:)]);
end

%ob_constraint = constructObstacleConstraint( 0,0,p, 'link2', [0; 0; 0.0],...
%    [1; 0; 2],1);

%prog = prog.addConstraint(ob_constraint,[last_x_inds;prog.param_inds(:)]);

%constraint.grad_method = 'numerical';
%constraint.grad_level = 0;

if strcmp(cost_function,'torque') == 1
    disp('using torque cost')
    prog = prog.addRunningCost(@torqueCost);
elseif strcmp(cost_function,'torquetime') == 1
    disp('using torque time cost')
    prog = prog.addRunningCost(@torqueTimeCost);
else
    disp('using no cost')
end
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
traj_init.u = PPTrajectory(foh(0:0.1:tf0, repmat([1;0;0],1,41)));

tic
[xtraj,utraj,z,F,info,xtraj0,utraj0] = prog.solveTraj(tf0,traj_init);
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

%calculate the utraj cost (without regularizer cost)
dt = z(prog.h_inds);
utraj_cost = computeCost(utraj,dt);

xtraj_outputFrame = xtraj.getOutputFrame;

save('outputRobotArm.mat');
%save('outputRobotArm.mat','p','xtraj','utraj','utraj0','xtraj0','F','utraj_cost');
v2 = p.constructVisualizer();
v2.playback(xtraj);
p.getParams().double
F
utraj_cost
disp('data saved to outputRobotArm.mat')
disp('you"re looking for v2.playback(xtraj);')
%v.playback(xtraj);