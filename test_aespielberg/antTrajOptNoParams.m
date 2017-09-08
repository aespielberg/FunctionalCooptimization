function [p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt, midcost, midz, utraj_init]...
    = antTrajOptNoParams(xtraj,utraj,ltraj,ljltraj,scale, costs, regularization_type,...
                regularization_weight, dist, h_goal, clamped_param_inds)
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.terrain = RigidBodyFlatTerrain(); %add basic terrain
options.floating = true; %our robot isn't tethered anywhere in the world
options.ignore_self_collisions = true; %We will enforce this through other constraints
p = RigidBodyManipulator('AntParams2016.urdf',options); %our robot urdf.  This still needs to be changed to create parameters


global ns
ns = p.getNumStates;
ui = p.getNumInputs;

p = p.setInputLimits(-1.5 * ones(ui, 1),1.5 * ones(ui, 1)); %the input torque limits
R_periodic = zeros(p.getNumStates,2*p.getNumStates); %if we want to enforce a periodic motion; not used presently


N = 16; %the number of knot points
T = 4; %The time interval
T0 = T;

N4 = floor(N/4);

%x0 = [0;zeros(ns - 1,1)];
%xf = [.2;zeros(ns - 1,1)];

%dist = 1.8;
h = 0.05;

pivec = pi*ones(6, 1);

%This is for the point contacts, this is a normaly straight upward:
singleLeg = [1; zeros(5, 1)];

%randomly seed the velocities normally around 0:
%vel = [randn; zeros(5, 1); randn(6, 1)];
vel = zeros(12, 1);
if nargin < 2
    dist = 1.8; %the distance we wish to travel
    h_goal = 0.3;
    clamped_param_inds = [2:7, 9:11];
end

to_options.clamped_param_inds = clamped_param_inds;

%seeding sort of intelligently the initial position values of the robot
x0 = [0.0; 0; h; zeros(3, 1); pivec; zeros(12, 1)];
x1 = [dist/4; 0; h; zeros(3, 1); pivec + 2*pi*[1/2; 0; 0; 1/2; 0; 1/2]; vel];
x2 = [2*dist/4; 0; h; zeros(3, 1); pivec + 2*pi*[1; 0; 0; 1; 0; 1]; vel];
x3 = [3*dist/4; 0; h; zeros(3, 1); pivec + 2*pi*[1; 1/2; 1/2; 1; 1/2; 1/2]; vel];
xf = [dist; 0; h; zeros(3, 1); 3*pi * ones(6, 1); zeros(ns/2, 1)];

%I don't know what the point of this was anymore, looks like a copy of
%above
% x0 = [0.0; 0; h; zeros(3, 1); pivec; zeros(10, 1)];
% x1 = [dist/4; 0; h; zeros(3, 1); pivec + 2*pi*[7/8; 0; 0; 0]; vel];
% x2 = [2*dist/4; 0; h; zeros(3, 1); pivec + 2*pi*[7/8; 0; 7/8; 0]; vel];
% x3 = [3*dist/4; 0; h; zeros(3, 1); pivec + 2*pi*[7/8; 7/8; 7/8; 0]; vel];
% xf = [dist; 0; h; zeros(3, 1); 3*pi * ones(4, 1); zeros(ns/2, 1)];

first_gait = 1
if isempty(xtraj)
    %Try to come up with a reasonable trajectory
    t_init = linspace(0,T0,N);
    traj_init.x = PPTrajectory(foh(t_init,[linspacevec(x0,x1,N4), linspacevec(x1,x2,N4), linspacevec(x2,x3,N4), linspacevec(x3,xf,N4)]));
    if isempty(utraj)
        traj_init.u = PPTrajectory(foh(t_init,randn(ui,N)));
    else
       traj_init.u = utraj; 
    end
    utraj_init = traj_init.u;
    
    if first_gait
        traj_init.l = PPTrajectory(foh(t_init,[repmat([zeros(6, 1); singleLeg; singleLeg; singleLeg; singleLeg; singleLeg],1,N4)  repmat([singleLeg; zeros(6, 1); singleLeg; singleLeg; singleLeg; singleLeg],1,N4) repmat([singleLeg; zeros(6, 1); singleLeg;  singleLeg; singleLeg; singleLeg],1,N4) repmat([singleLeg; singleLeg; singleLeg; zeros(6, 1); singleLeg; singleLeg],1,N4)]));
    else
        traj_init.l = PPTrajectory(foh(t_init,[repmat([zeros(6, 1); singleLeg; singleLeg; singleLeg; singleLeg; singleLeg],1,N4)  repmat([singleLeg; zeros(6, 1); singleLeg; singleLeg; singleLeg; singleLeg],1,N4) repmat([singleLeg; singleLeg; zeros(6, 1); singleLeg; singleLeg; singleLeg],1,N4) repmat([singleLeg; singleLeg; singleLeg; zeros(6, 1); singleLeg; singleLeg],1,N4)]));
    end
    

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
traj_opt.actuation_weight = 1.0;
%Make sure snopt is running!!!  fmincon takes FOREVER
traj_opt.setSolver('snopt')

traj_opt = traj_opt.setRegularizerWeight(regularization_weight);


% power_constraint = constructHexapodMotorConstraint(p.getNumParams(), p.getNumInputs(), N);
% 
% u_inds = traj_opt.u_inds(:);
% x_inds = traj_opt.x_inds;
% 
% joint_motion_inds = x_inds(end-6 + 1:end, :);
% v_inds = joint_motion_inds(:);
% 
% 
% traj_opt = traj_opt.addConstraint(power_constraint,  [traj_opt.param_inds(:); u_inds; v_inds]);
% 
% 



%I made the slack too big, so the current cost function, minimize torque,
%obiously just goes for the shortest distance when possible.  Minimum
%energy per distance is better


%TODO: add this back in
x0_min = x0;
x0_max = x0;


% x0_min(3) = -inf;
% x0_max(3) = inf;

%xf_min = [xf(1:10); -inf(ns/2, 1)]; - 0.05 * ones(1, 20);
%xf_max = [xf(1:10); inf(ns/2, 1)]; + 0.05 * ones(1, 20);
xf_min = xf;
xf_max = xf;

x0_min(3) = h_goal;
x0_max(3) = h_goal;
xf_min(3) = h_goal;
xf_max(3) = h_goal;

x0_min
x0_max
xf_min
xf_max

 %xf_min(1) = x0_min(1) + 0.01;
 %xf_max(1) = inf;


%Everything above basically says we want to start and end in our initial
%coordinates with slack in the traversed direction.  This makes our gait af
%full loop

traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(x0_min,x0_max),1);
traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(xf_min,xf_max),N);

if strmatch('torque', costs)
    traj_opt = traj_opt.addRunningCost(@torqueCost);
end

if strmatch('velocity', costs)
    traj_opt = traj_opt.addRunningCost(@velocityCost);
end

for i = 2:1:N-1
    lb = [-inf(18, 1); -2 * pi * ones(6, 1) ];
    ub =  [inf(18, 1); 2 * pi * ones(6, 1)];
    bb = BoundingBoxConstraint(lb, ub);
    traj_opt = traj_opt.addConstraint(bb, traj_opt.x_inds(:, i));
     
end


% constraint = constructAboveGroundConstraint( zeros(8, 1), inf(8, 1), p, 'base_link');

constraint = constructAboveGroundConstraint( zeros(8, 1), inf(8, 1), p, 'base_link');

for i = 1:1:N
    all_x_inds = traj_opt.x_inds;
    x_inds = all_x_inds(:, i);
    traj_opt = traj_opt.addConstraint(constraint,  [x_inds; traj_opt.param_inds(:)]);
end

% 
% for i = 1:1:N
%     all_x_inds = traj_opt.x_inds;
%     x_inds = all_x_inds(:, i);
%     traj_opt = traj_opt.addConstraint(constraint,  [x_inds; traj_opt.param_inds(:)]);
% end

%This is all code for more exact constraints on the bounding box of the
%robot.  This wasn't efficient and/or was buggy, so it's commented out.
%It's a more exact way of prevening intersection than checking the pitch
%and doesnt' require a priori calculations.

%iteration limits
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',2000);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',200000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',2000000);
traj_opt = traj_opt.setSolverOptions('snopt', 'print', 'snopt_output_1.txt');

%And away we go!!
midcost = 0;
midz = 0;
[xtraj,utraj,ltraj,ljltraj,z,F,info, midz, midcost] = traj_opt.solveTraj(t_init,traj_init);
%[xtraj,utraj,ltraj,ljltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);


end