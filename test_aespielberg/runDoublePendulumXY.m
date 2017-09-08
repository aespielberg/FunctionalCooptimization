function [file_name] = runDoublePendulumXY(use_l1_norm,add_obstacles,use_position_diff_constraint,cost_function,test_name,test_num )

%options.terrain = RigidBodyFlatTerrain();
p = RigidBodyManipulator('DoublePendulumXYSym.urdf');

%put limits on the inputs
inputVector = ones(p.getNumInputs(),1);
limit = 0.345;
%limit = 10.0;
p = p.setInputLimits(-limit*inputVector,limit*inputVector);

x0 = zeros(p.getNumStates(),1);
xf = x0;
xf(1) = pi;
tf0 = 6;



N = 60; %TODO: bump this back up
N4 = N/4;

% sphere1 = RigidBodySphere(1,[-1.5;0;2],[0;0;0]);
% sphere1.c = [83,53,10]/255;  % brown
% p = p.addGeometryToBody('world',sphere1, 'wall');
% p = compile(p);

%sphere2 = RigidBodySphere(1,[1.5;0;6],[0;0;0]);
%sphere2.c = [83,53,10]/255;  % brown
%p = p.addGeometryToBody('world',sphere2, 'wall');
%p = compile(p);





prog = DirtranTrajectoryOptimizationTest(p,N,[0 tf0]);


if ~use_l1_norm
    disp('using regularizer');
    prog = prog.setRegularizerWeight(0.0);
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
    thresh = 0.001;
    constraint = constructPositionDiffConstraint( [-thresh; -thresh; -inf; -inf; -inf; -inf]...
        ,[thresh; thresh; inf; inf; inf; inf],p, 'lower_link', [0; 0; 0.0],...
        [0; 0.2634; 0; 0; 0; 0]); %xyz rpy
    prog = prog.addConstraint(constraint,  [last_x_inds; prog.param_inds(:)]);
end

r = 0.3;
offset = 0.5;
sphere1 = RigidBodySphere(r,[-offset;-r*1.5;0],[0;0;0]);
sphere1.c = [83,53,10]/255;  % brown
p = p.addGeometryToBody('world',sphere1, 'wall');
p = compile(p);
ob_constraint = constructObstacleConstraint(p, 'lower_link', [0; 0; 0], [-offset;-r*1.5;0], r);

for i = 1:1:N
    prog = prog.addConstraint(ob_constraint,[prog.x_inds(:,i); prog.param_inds(:)]);
end

r = 0.3;
offset = 0.3;
sphere1 = RigidBodySphere(r,[-offset;-r*1.5;0],[0;0;0]);
sphere1.c = [83,53,10]/255;  % brown
p = p.addGeometryToBody('world',sphere1, 'wall');
p = compile(p);
ob_constraint = constructObstacleConstraint(p, 'lower_link', [0; 0; 0], [-offset;-r*1.5;0], r);

for i = 1:1:N
    prog = prog.addConstraint(ob_constraint,[prog.x_inds(:,i); prog.param_inds(:)]);
end


offset = 1.2;
r = 1.0;
sphere2 = RigidBodySphere(r,[offset;0;0],[0;0;0]);
sphere2.c = [83,53,10]/255;  % brown
p = p.addGeometryToBody('world',sphere2, 'wall');
p = compile(p);
ob_constraint = constructObstacleConstraint(p, 'lower_link', [0; 0; 0], [offset;0;0], r);

for i = 1:1:N
    prog = prog.addConstraint(ob_constraint,[prog.x_inds(:,i); prog.param_inds(:)]);
end



goal_sphere = RigidBodySphere(0.02,[0;0.2634;0],[0;0;0]);
goal_sphere.c = [255, 0, 0]/255;  % red
p = p.addGeometryToBody('world',goal_sphere, 'goal');
p = compile(p);



pos = [0.3; 0.5; 0];
r = 0.2;
sphere3 = RigidBodySphere(r,pos,[0;0;0]);
sphere3.c = [83,53,10]/255;  % brown
p = p.addGeometryToBody('world',sphere3, 'wall');
p = compile(p);
ob_constraint = constructObstacleConstraint(p, 'lower_link', [0; 0; 0], pos, r);

for i = 1:1:N
    prog = prog.addConstraint(ob_constraint,[prog.x_inds(:,i); prog.param_inds(:)]);
end



if strcmp(cost_function,'torque') == 1
    disp('using torque cost')
    prog = prog.addRunningCost(@torqueCost);
elseif strcmp(cost_function,'torquetime') == 1
    disp('using torque time cost')
    prog = prog.addRunningCost(@torqueTimeCost);
else
    disp('using no cost')
end

prog = prog.setSolverOptions('snopt','MajorIterationsLimit',2000);
prog = prog.setSolverOptions('snopt','MinorIterationsLimit',200000);
prog = prog.setSolverOptions('snopt','IterationsLimit',2000000);



t_init = linspace(0,tf0,N);
traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));
% x1 = x0;
% x1(1) = pi/2;
% x1(2) = -pi/2;
% 
% x2 = x0;
% x2(1) = pi;
% x2(2) = -pi/2;
% 
% x3 = x0;
% x3(1) = pi*3/2;
% x3(2) = -pi/2;
% 
% 
% 
% traj_init.x = PPTrajectory(foh(t_init,[linspacevec(x0,x1,N4), linspacevec(x1,x2,N4), linspacevec(x2,x3,N4), linspacevec(x3,xf,N4)]));

ui = p.getNumInputs();

traj_init.u =  PPTrajectory(foh(t_init,limit*rand(ui,N) - limit/2.0));

tic
[xtraj,utraj,z,F,info,~] = prog.solveTraj(tf0,traj_init);
toc

numParams = p.getNumParams();

if numParams > 0
    p = p.setParams(z(end-numParams+1:end)); %TODO: make this cleaner
end

%calculate the utraj cost (without regularizer cost)
dt = z(prog.h_inds);
utraj_cost = computeCost(utraj,dt);

xtraj_outputFrame = xtraj.getOutputFrame;

file_name = strcat(test_name, num2str(test_num),'.mat');

save(file_name);
disp(strcat('data saved to ', file_name))



end

