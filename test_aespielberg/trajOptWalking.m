function [times, statistics, p] = trajOptWalking(num_runs, options, input_utraj)


if nargin < 1
    num_runs = 1;
    visualize = true;
    
elseif nargin < 2
    visualize = false;
else
    visualize = options.visualize;
    bot = options.bot;
    costs = options.costs;
    regularization_type = options.regularization_type;
    regularization_weight = options.regularization_weight;
    dist = options.dist;
    h_goal = options.h_goal;
    clamped_param_inds = options.clamped_param_inds;
end

if nargin < 2
    bot = 'Ant';
    costs = {};
    regularization_type = 2;
    regularization_weight = 100.0;
    
end

if nargin < 3
   for i = 1:1:num_runs
      input_utraj{i} = []; 
   end
end



megaclear
times = zeros(num_runs, 1);
cost = {};
for i = 1:1:num_runs
    
    disp('this is run number')
    disp(i)
    tic
    clear xtraj;
    clear utraj;
    clear ltraj;
    clear ljltraj;
    try %catches a failure in the run if the second one fails
        try %determines how we run it...TODO: edit if we do this using the first one
            xtraj = options.xtraj;
            utraj = options.utraj;
            ltraj = options.ltraj;
            ljltraj = options.ljltraj;
            if strcmp(bot, 'Ant')
                [p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt, midcost, midz, utraj_init] = antTrajOpt(xtraj, utraj, ltraj, ljltraj, 0.001, costs, regularization_type, regularization_weight, dist, h_goal, clamped_param_inds);
            elseif strcmp(bot, 'Biped')
                [p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt, midcost, midz, utraj_init] = bipedTrajOpt(xtraj, utraj, ltraj, ljltraj, 0.001, costs, regularization_type, regularization_weight, dist, h_goal, clamped_param_inds);
            elseif strcomp(bot, 'Quadruped')
                [p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt, midcost, midz, utraj_init] = quadrupedTrajOpt(xtraj, utraj, ltraj, ljltraj, 0.001, costs, regularization_type, regularization_weight, dist, h_goal, clamped_param_inds);
            elseif strcomp(bot, 'AntSeparateLegs')
                [p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt, midcost, midz, utraj_init] = antSeprateLegsTrajOpt(xtraj, utraj, ltraj, ljltraj, 0.001, costs, regularization_type, regularization_weight, dist, h_goal, clamped_param_inds);
            end
        catch
            if strcmp(bot, 'Ant')
                [p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt, midcost, midz, utraj_init] = antTrajOpt([], input_utraj{i}, [], [], 0.001, costs, regularization_type, regularization_weight, dist, h_goal, clamped_param_inds);
            elseif strcmp(bot, 'Biped')
                % load('starting_traj/biped_test')
                % input_utraj{i} = u_save;
                [p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt, midcost, midz, utraj_init] = bipedTrajOpt([], input_utraj{i}, [], [], 0.000001, costs, regularization_type, regularization_weight, dist, h_goal, clamped_param_inds);
            elseif strcmp(bot, 'Quadruped')
                [p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt, midcost, midz, utraj_init] = quadrupedTrajOpt([], input_utraj{i}, [], [], 0.001, costs, regularization_type, regularization_weight, dist, h_goal, clamped_param_inds);
            elseif strcmp(bot, 'AntSeparateLegs')
                [p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt, midcost, midz, utraj_init] = antSeparateLegsTrajOpt([], input_utraj{i}, [], [], 0.001, costs, regularization_type, regularization_weight, dist, h_goal, clamped_param_inds);
            end
        end
        recordResults(p, z(traj_opt.param_inds, 1), xtraj); %needed for viz
%         xtraj_compare = xtraj;
%         load('starting_traj/xtraj_result');
%         diff = (xtraj - xtraj_compare);
%         norm(diff.eval(0.1:0.1:diff.tspan(2)), 2)
        times(i) = toc;
        dt = z(traj_opt.h_inds);
        cost{i} = {midcost, F, z(traj_opt.param_inds, 1) midz(traj_opt.param_inds, 1), xtraj, utraj, utraj_init};
        if ~isempty(traj_opt.param_inds)
            p = p.setParams(z(traj_opt.param_inds, 1));
        end
        
        if visualize
            v = p.constructVisualizer; v.playback(xtraj);
        end
        
        catch
            disp('failed run')
            
       end
    

    
end





statistics = cost;



end