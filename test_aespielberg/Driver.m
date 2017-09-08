options = struct();
%biped
%options.costs = {'torque', 'foot'};

%ant
%options.costs = {'torque'};

%test
options.costs = {};
options.visualize = true;

options.regularization_type = 2;
options.regularization_weight = 0.0;

 %options.regularization_type = 2;
 %options.regularization_weight = 10.0;
 %options.actuation_weight = 1.0;
 options.actuation_weight = 0.0;
 
 %Biped
% options.bot = 'Biped';
%  options.h_goal = 3.0;
%  options.clamped_param_inds = 3:6; %Biped 8 params
%   options.dist = 3.0;

  options.bot = 'Quadruped';
 options.h_goal = 3.0;
 options.clamped_param_inds = 3:9; %Quadruped 5 params
  options.dist = 2.5;
 
 %ANT
%  options.bot = 'Ant';
%  options.h_goal = 1.5;
% options.clamped_param_inds= [4:7, 9:11]; %Ant 4 Params
% %options.clamped_param_inds = 1:11;
% options.dist = 4.5;

% options.bot = 'AntSeparateLegs';
% options.h_goal = 0.5;
% options.clamped_param_inds= [4:7, 14:16]; %Ant 4 Params
% options.dist = 3.0;



%Biped baseline
% options.clamped_param_inds = 1:12;
% load('Biped5Params');

num_runs = 3;


for i = 1:1:num_runs
    if exist('costs', 'var')
        if ~isempty(costs{i})
            input_utraj{i} = costs{i}{7};
        else
            input_utraj{i} = []; %TODO: not ideal...should have saved inputs upfront.
        end
    else
        input_utraj{i} = [];
    end
end

%



[times, costs, p] = trajOptWalking(num_runs, options, input_utraj);