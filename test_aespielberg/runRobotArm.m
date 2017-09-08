function [file_name] = runRobotArm(use_l1_norm,add_obstacles,use_position_diff_constraint,cost_function,test_name,test_num )

%options.terrain = RigidBodyFlatTerrain();
p = RigidBodyManipulator('TriplePendulumInertial.urdf');

%put limits on the inputs
inputVector = ones(p.getNumInputs(),1);

limit = 20;
p = p.setInputLimits(-limit*inputVector,limit*inputVector);

x0 = zeros(p.getNumStates(),1);
xf = x0;
xf(1) = pi;
tf0 = 6;



N = 10; %TODO: bump this back up



prog = DirtranTrajectoryOptimizationTest(p,N,[0 tf0]);

if add_obstacles
    collision_constraint = generateConstraint(MinDistanceConstraint(p,0.2),0);
    prog = prog.addStateConstraint(collision_constraint{1},1:N,1:getNumPositions(p));
end

if ~use_l1_norm
    disp('using regularizer');
    prog = prog.setRegularizerWeight(5000.0);
    options.penalty = 2;
else
    disp('using l1 norm');
    prog = prog.setRegularizerWeight(5000.0);
    options.penalty = 1;
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
        ,[thresh; inf; thresh; inf; inf; inf],p, 'bottom_link', [0; 0; 0.0],...
        [0; 0; 5; 0; 0; 0]); %xyz rpy
    prog = prog.addConstraint(constraint,  [last_x_inds; prog.param_inds(:)]);
end




if strcmp(cost_function,'torque') == 1
    disp('using torque cost')
    prog = prog.addRunningCost(@torqueCost);
elseif strcmp(cost_function,'torquetime') == 1
    disp('using torque time cost')
    prog = prog.addRunningCost(@torqueTimeCost);
elseif strcmp(cost_function,'time')
    prog = prog.addFinalCost(@timeCost);
end

prog = prog.setSolverOptions('snopt','MajorIterationsLimit',2000);
prog = prog.setSolverOptions('snopt','MinorIterationsLimit',200000);
prog = prog.setSolverOptions('snopt','IterationsLimit',2000000);

% [lower, upper] = p.getParamLimits();
% bbc = BoundingBoxConstraint(lower, upper);
% prog = prog.addBoundingBoxConstraint(bbc, prog.param_inds);



traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));

ui = p.getNumInputs();
t_init = linspace(0,tf0,N);
traj_init.u =  PPTrajectory(foh(t_init,limit*rand(ui,N)));



tic
[xtraj,utraj,z,F,info,xtraj0,utraj0] = prog.solveTraj(tf0,traj_init);
toc

numParams = p.getNumParams();

p = p.setParams(z(end-numParams+1:end)); %TODO: make this cleaner

%calculate the utraj cost (without regularizer cost)
dt = z(prog.h_inds);
utraj_cost = computeCost(utraj,dt);
tspan = xtraj.tspan;
xtraj_outputFrame = xtraj.getOutputFrame;

file_name = strcat(test_name, num2str(test_num),'.mat');
params = z(prog.param_inds);

save(file_name);
disp(strcat('data saved to ', file_name))

%save('outputRobotArm.mat','p','xtraj','utraj','utraj0','xtraj0','F','utraj_cost');

end

