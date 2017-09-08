function [p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt, midcost, midz, utraj_init]...
    = antSeparateLegsTrajOpt(xtraj,utraj,ltraj,ljltraj,scale, costs, regularization_type,...
                regularization_weight, dist, h_goal, clamped_param_inds)
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.terrain = RigidBodyFlatTerrain(); %add basic terrain
options.floating = true; %our robot isn't tethered anywhere in the world
options.ignore_self_collisions = true; %We will enforce this through other constraints
p = RigidBodyManipulator('AntSeparateLegs.urdf',options); %our robot urdf.  This still needs to be changed to create parameters


global ns
ns = p.getNumStates;
ui = p.getNumInputs;

p = p.setInputLimits(-10 * ones(ui, 1), 10 * ones(ui, 1)); %the input torque limits
R_periodic = zeros(p.getNumStates,2*p.getNumStates); %if we want to enforce a periodic motion; not used presently


N = 16; %the number of knot points
T = 4; %The time interval
T0 = T;

N4 = floor(N/4);

%x0 = [0;zeros(ns - 1,1)];
%xf = [.2;zeros(ns - 1,1)];

%dist = 1.8;
h = 0.3;

pivec = pi*ones(6, 1);

%This is for the point contacts, this is a normaly straight upward:
singleLeg = [1; zeros(5, 1)];

%randomly seed the velocities normally around 0:
%vel = [randn; zeros(5, 1); randn(6, 1)];
vel = zeros(12, 1);
if nargin < 2
    dist = 1.8; %the distance we wish to travel
    h_goal = 0.3;
    clamped_param_inds = [2:7, 14:16];
end

to_options.clamped_param_inds = clamped_param_inds;

%seeding sort of intelligently the initial position values of the robot
x0 = [0.0; 0; h; zeros(3, 1); pivec; zeros(12, 1)];
x1 = [dist/4; 0; h; zeros(3, 1); pivec + 2*pi*[7/8; 0; 0; 0; 0; 0]; vel];
x2 = [2*dist/4; 0; h; zeros(3, 1); pivec + 2*pi*[7/8; 7/8; 0; 0; 0; 0]; vel];
x3 = [3*dist/4; 0; h; zeros(3, 1); pivec + 2*pi*[7/8; 7/8; 7/8; 0; 0; 0]; vel];
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
%Make sure snopt is running!!!  fmincon takes FOREVER
traj_opt.setSolver('snopt')

traj_opt = traj_opt.setRegularizerWeight(regularization_weight);



%traj_opt = traj_opt.addRunningCost(@running_cost_fun); %we will have to edit the cost function


%traj_opt = traj_opt.addRunningCost(@foot_height_fun); TODO: add back



%traj_opt = traj_opt.addFinalCost(@final_cost_fun); %Probably not very important, can get rid of


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

%Turning:
xf_min(2) = xf_max(2) + 1.0;
xf_max(2) = xf_max(2) + 1.0;
xf_min(6) = 0.0;
xf_max(6) = pi/2.0;


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

%here go constraints on the body for each knot point

%TODO: commented this out
% nc = (N-3)*2; %the number of constraints
% 
% A = zeros(nc, traj_opt.num_vars); %Create the constraints as a linear system
% ub = inf(nc, 1);
% lb = -inf(nc, 1);
% 
% %Constrain the pitch at each timestep
% for i = 2:1:(N-2)
%     idx = i-1;
%     A(2*idx - 1, traj_opt.x_inds(3, i)) = 1;
%     A(2*idx, traj_opt.x_inds(5, i)) = 1;
%     
%     lb(2*idx - 1) = 0.1;
%     lb(2*idx) = -pi/16;
%     ub(2*idx) = pi/16;
% end
% 
% traj_opt = traj_opt.addLinearConstraint(LinearConstraint(lb, ub, A));

%TODO: ended commented out

constraint = constructAboveGroundConstraint( zeros(8, 1), inf(8, 1), p, 'base_link');

for i = 1:1:N
    all_x_inds = traj_opt.x_inds;
    x_inds = all_x_inds(:, i);
    traj_opt = traj_opt.addConstraint(constraint,  [x_inds; traj_opt.param_inds(:)]);
end

%This is all code for more exact constraints on the bounding box of the
%robot.  This wasn't efficient and/or was buggy, so it's commented out.
%It's a more exact way of prevening intersection than checking the pitch
%and doesnt' require a priori calculations.

% for i = 1:1:(N-1)
%     traj_opt = traj_opt.addStateConstraint(FunctionHandleConstraint(0, inf, ns, @pos_constraint1, 0), i);
%     traj_opt = traj_opt.addStateConstraint(FunctionHandleConstraint(0, inf, ns, @pos_constraint2, 0), i);
%     traj_opt = traj_opt.addStateConstraint(FunctionHandleConstraint(0, inf, ns, @pos_constraint3, 0), i);
%     traj_opt = traj_opt.addStateConstraint(FunctionHandleConstraint(0, inf, ns, @pos_constraint4, 0), i);
% end

% nc = (N-1);
%
% A = zeros(nc, traj_opt.num_vars);
% ub = inf(nc, 1);
% lb = -inf(nc, 1);
%
% for i = 1:1:(N-1)
%     idx = i;
%     A(idx, traj_opt.x_inds(3, i)) = 1;
%
%     lb(idx) = 0.2;
% end
%
% traj_opt.addLinearConstraint(LinearConstraint(lb, ub, A));
%
% nc = (N-1);
%
% A = zeros(nc, traj_opt.num_vars);
% ub = inf(nc, 1);
% lb = -inf(nc, 1);
%
% for i = 1:1:(N-1)
%     idx = i;
%     A(idx, traj_opt.x_inds(5, i)) = 1;
%
%     lb(idx) = 0 - pi/32;
%     ub(idx) = 0 + pi/32;
% end
%
% traj_opt.addLinearConstraint(LinearConstraint(lb, ub, A));


%Constrain the 4 lower corners to be above the ground




%constrain the pitch to be between -pi/2 and pi/2




%traj_opt = traj_opt.addStateConstraint(periodic_constraint,{[1
%N]}); %TODO: add this back


%[f,df] = foot_height_fun(0,xr,ur); TODO: add back
%[f2,df2] =
%geval(@foot_height_fun,0,xr,ur,struct('grad_method','numerical')); TODO: add back


% l1 = traj_opt.l_inds(5:end,1:5);
% l2 = traj_opt.l_inds(1:4,end-4:end);
% traj_opt = traj_opt.addConstraint(ConstantConstraint(0*l1(:)),l1);
% traj_opt = traj_opt.addConstraint(ConstantConstraint(0*l2(:)),l2);


% traj_opt = traj_opt.setCheckGrad(true);
%snprint('snopt.out');

%iteration limits
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',2000);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',200000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',2000000);

%And away we go!!
[xtraj,utraj,ltraj,ljltraj,z,F,info, midcost, midz] = traj_opt.solveTraj(t_init,traj_init);

%Below are just a bunch of current cost functions used
    function [f,df] = running_cost_fun(h,x,u)
        f = u'*u;
        disp(f)
        df = [0 zeros(1,ns) 2*u'];
    end

    function [f,df] = foot_height_fun(h,x,u)
        q = x(1:(ns/2));
        K = 100;
        [phi,~,~,~,~,~,~,~,n] = p.contactConstraints(q,false,struct('terrain_only',false));
        phi0 = [.1;.1];
        f = K*(phi - phi0)'*(phi - phi0);
        % phi: 2x1
        % n: 2xnq
        df = [0 2*K*(phi-phi0)'*n zeros(1,9)];
    end

    function [f,df] = final_cost_fun(T,x)
        K = 1000000;
        f = K*T;
        df = [K zeros(1,ns)];
    end



end