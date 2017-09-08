function [p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt, midcost, midz, utraj_init] = ...
    bipedTrajOpt(xtraj,utraj,ltraj,ljltraj,scale, costs, regularization_type,...
                regularization_weight, dist, h_goal, clamped_param_inds)
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.terrain = RigidBodyFlatTerrain(); %add basic terrain
options.floating = true; %our robot isn't tethered anywhere in the world
options.ignore_self_collisions = true; %We will enforce this through other constraints
p = RigidBodyManipulator('QuadrupedFull.urdf',options); %our robot urdf.  This still needs to be changed to create parameters

global ns
ns = p.getNumStates;
ui = p.getNumInputs;

torque = 60;
%torque = 100;
%p = p.setInputLimits(-torque*ones(6, 1),torque*ones(6, 1));
p = p.setInputLimits(-torque*ones(8, 1),torque*ones(8, 1));

R_periodic = zeros(p.getNumStates,2*p.getNumStates); %if we want to enforce a periodic motion; not used presently


N = 20; %the number of knot points
T = 4; %The time interval
T0 = T;
%The time runs from 1-4 right now
T_span = [1 T];

%slack to parts of the optimization problem
%scale = 0.001;
to_options.compl_slack = scale*.01;
to_options.lincompl_slack = scale*.001;
to_options.nlcc_mode = 2;
to_options.lincc_mode = 1;
to_options.lambda_mult = p.getMass*9.81*T0/N;



N5 = floor(N/5);

%x0 = [0;zeros(ns - 1,1)];
%xf = [.2;zeros(ns - 1,1)];


rot = pi/16;

%This is for the point contacts, this is a normaly straight upward:
singleLeg = [1; zeros(5, 1)];

vel = zeros(14, 1);

h = 3.0;

if nargin < 2
dist = 3.0; %the distance we wish to travel
h_goal = 3.2;
clamped_param_inds = [2:11];
scale = 0.001;
regularization_weight = 10.0;
regularization_type = 2;
costs = {};
end

to_options.clamped_param_inds = clamped_param_inds;

%seeding sort of intelligently the initial position values of the robot
%TODO: good initial seed?
x0 = [0; 0; h; zeros(3, 1); [0; 0; 0; 0; 0; 0; 0; 0]; vel];
x1 = [0; dist/5; h; zeros(3, 1); [rot; 0; -rot; 0; rot; 0; 0; 0]; vel];
x2 = [0; 2*dist/5; h; zeros(3, 1); [rot; rot; -rot; -rot; rot; rot; 0; 0]; vel];
x3 = [0; 3*dist/5; h; zeros(3, 1);[0; rot; 0; -rot; 0; rot; 0; 0]; vel];
x4 = [0; 4*dist/5; h; zeros(3, 1);[0; 0; 0; 0; 0; 0; 0; 0]; vel];
xf = [0; dist; h; zeros(3, 1); [0; 0; 0; 0; 0; 0; 0; 0]; vel];


first_gait = 1
if isempty(xtraj)
    %Try to come up with a reasonable trajectory
    t_init = linspace(0,T0,N);
    traj_init.x = PPTrajectory(foh(t_init,[linspacevec(x0,x1,N5), linspacevec(x1,x2,N5), linspacevec(x2,x3,N5), linspacevec(x3,x4,N5), linspacevec(x4,xf,N5)]));
    if isempty(utraj)
        traj_init.u = PPTrajectory(foh(t_init,2*torque*rand(ui,N) - torque));
        %u_save = traj_init.u;
        %save('starting_traj/biped_test', 'u_save')
    else
       traj_init.u = utraj; 
    end
    utraj_init = traj_init.u;

    traj_init.l = PPTrajectory(foh(t_init,[ ...
        repmat([singleLeg; singleLeg; singleLeg; singleLeg],1,N5)  ...
        repmat([zeros(6, 1); singleLeg; singleLeg; singleLeg],1,N5) ...
        repmat([singleLeg; zeros(6, 1); singleLeg; singleLeg],1,N5) ...
        repmat([singleLeg; singleLeg; zeros(6, 1); singleLeg],1,N5) ...
        repmat([singleLeg; singleLeg; singleLeg; singleLeg],1,N5)]));

    
    
    
    %scale = 1;
else %This can be used if we wanted to pass in good initial guesses
    t_init = xtraj.pp.breaks;
    traj_init.x = xtraj;
    traj_init.u = utraj;
    traj_init.l = ltraj;
    traj_init.ljl = ljltraj;
end

%The time runs from 1-4 right now
T_span = [1 T];

%slack to parts of the optimization problem
to_options.compl_slack = scale*.01;
to_options.lincompl_slack = scale*.001;

%Things with jl are ignored since we have continuous joints in this
%example; no dimensional limits
%to_options.jlcompl_slack = scale*.01;

%I honestly can't remember what these do but they were good so I'm not
%touching it
to_options.nlcc_mode = 2;
to_options.lincc_mode = 1;
to_options.lambda_mult = p.getMass*9.81*T0/N;
to_options.penalty = regularization_type;
%to_options.lambda_jl_mult = T0/N;

%Here we create traj_opt, an object holding our optimization problem
traj_opt = ContactImplicitTrajectoryOptimizationTest(p,N,T_span,to_options);
%Make sure snopt is running!!!  fmincon takes FOREVER
traj_opt.setSolver('snopt')
traj_opt = traj_opt.setSolverOptions('snopt', 'print', 'snopt_output_1.txt');
traj_opt.actuation_weight = 0.0;


traj_opt = traj_opt.setRegularizerWeight(regularization_weight);




x0_min = x0;
x0_max = x0;
xf_min = xf;
xf_max = xf;

x0_min(3) = 3.5;
x0_max(3) = 4.0;

xf_min(3) = 3.5;
xf_max(3) = 4.0;





 %xf_min(1) = x0_min(1) + 0.01;
 %xf_max(1) = inf;


%Everything above basically says we want to start and end in our initial
%coordinates with slack in the traversed direction.  This makes our gait af
%full loop

if strmatch('torque', costs)
    traj_opt = traj_opt.addRunningCost(@torqueCost);
end

if strmatch('velocity', costs)
    traj_opt = traj_opt.addRunningCost(@velocityCost);
end

if strmatch('foot', costs)
   cost1 = feetHeightCost(p, 'leg_link2');
   cost2 = feetHeightCost(p, 'leg_link3');
   traj_opt = traj_opt.addRunningCost(cost1);
   traj_opt = traj_opt.addRunningCost(cost2);
end

traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(x0_min,x0_max),1);
traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(xf_min,xf_max),N);
%traj_opt = traj_opt.addRunningCost(@torqueCost);
traj_opt = traj_opt.addFinalCost(@timeCost);


constraint1 = constructAboveGroundConstraint( zeros(8, 1), inf(8, 1), p, 'base_link');
constraint2 = constructAboveGroundConstraint( zeros(8, 1), inf(8, 1), p, 'leg_link0');
constraint3 = constructAboveGroundConstraint( zeros(8, 1), inf(8, 1), p, 'leg_link1');
constraint4 = constructAboveGroundConstraint( zeros(8, 1), inf(8, 1), p, 'leg_link4');
constraint5 = constructAboveGroundConstraint( zeros(8, 1), inf(8, 1), p, 'leg_link5');


for i = 1:1:N
    all_x_inds = traj_opt.x_inds;
    x_inds = all_x_inds(:, i);
     traj_opt = traj_opt.addConstraint(constraint1,  [x_inds; traj_opt.param_inds(:)]);
     traj_opt = traj_opt.addConstraint(constraint2,  [x_inds; traj_opt.param_inds(:)]);
     traj_opt = traj_opt.addConstraint(constraint3,  [x_inds; traj_opt.param_inds(:)]);
     traj_opt = traj_opt.addConstraint(constraint4,  [x_inds; traj_opt.param_inds(:)]);
     traj_opt = traj_opt.addConstraint(constraint5,  [x_inds; traj_opt.param_inds(:)]);
end




traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',2000);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',200000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',2000000);



[xtraj,utraj,ltraj,ljltraj,z,F,info, midcost, midz] = traj_opt.solveTraj(t_init,traj_init);






end