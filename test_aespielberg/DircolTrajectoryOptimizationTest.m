classdef DircolTrajectoryOptimizationTest < DirectTrajectoryOptimization
    % Direct colocation approach
    % Over each interval, f(x(k),u(k)) and f(x(k+1),u(k+1)) are evaluated to
    % determine d/dt x(k) and d/dt x(k+1). A cubic spline is fit over the
    % interval x and d/dt x at the end points.
    % x(k+.5) and d/dt x(k+.5) are determined based on this spline.
    % Then, the dynamics constraint is:
    % d/dt x(k+.5) = f(x(k+.5),.5*u(k) + .5*u(k+1))
    %
    %  integrated cost is: .5*h(1)*g(x(1),u(1)) + .5*h(N-1)*g(x(N),u(N)) +
    %                   sum((.5*h(i)+.5*h(i-1))*g(x(i),u(i))
    %  more simply stated, integrated as a zoh with half of each time
    %  interval on either side of the knot point
    % this might be the wrong thing for the cost function...
    properties (SetAccess = protected)
        param_inds %the indices of the parameters
        suppl_inds %indices of supplementary variable for actuation
        regularizer_weight
        penalty
        initial_param_values
        costs_to_add
        function_name
        actuation_weight
    end
    
    properties (Constant)
        generate_dynamics = false;
        zeroth_run = false;
        first_run  = true;
    end
    
    methods
        function obj = DircolTrajectoryOptimizationTest(plant,N,duration,options)
            
            if nargin < 4
                options = struct();
            end
            if ~isfield(options,'integration_method')
                options.integration_method = DirtranTrajectoryOptimization.MIDPOINT;
            end
            
            if ~isfield(options,'actuation_weight')
                options.actuation_weight = 0.0;
            end
            
            
            

            
            %TODO: testing!!
            
            all_xyz = [];
            all_axis = [];
            for i = 1:1:plant.getNumBodies()
                parent = plant.getBody(i);
                if ~parent.parent %TODO: really not the guard, I think, for floating plants
                    continue; %skip over root which is grounded
                end
                %rpy = sym(zeros(3, 1));
                %xyz = sym(strcat('xyz', num2str(i)), [3, 1], 'real');
                rpy = zeros(3, 1);
                xyz = zeros(3, 1);
                all_xyz = [all_xyz; xyz];
                
                
                parentIdx = plant.findLinkId(parent.linkname);
                fr = RigidBodyFrame(parentIdx,xyz,rpy,[parent.linkname,'_force_frame']);
                [plant,frame_id] = addFrame(plant,fr);
                axis = [0; 0; 1];
%                 axis = sym(strcat('axis', num2str(i)), [3, 1]);
%                 assume(axis, 'real');
%                 assume(axis, 'positive');
                all_axis = [all_axis; axis];
                fe = RigidBodyThrust(frame_id,axis, 1, [-inf, inf]);
                %fe.direct_feedthrough_flag = false;
                fe.name = [parent.linkname, 'thrust'];
                plant = plant.setForce(length(plant.force) + 1, fe);
            end
            
            num_new_acts = length(plant.force);
            plant = compile(plant);
            
            
            
            if ~isfield(options,'actuation_pattern')
                options.actuation_pattern = zeros(N, num_new_acts);
            end
           
            
            obj = obj@DirectTrajectoryOptimization(plant,N,duration,options);
            
            obj.plant = plant;
            
            %set external force constants:
            lb = [];
            ub = [];
            for i=1:1:N
                %construct lb and ub
               num_old_act = obj.plant.getNumInputs() - num_new_acts;
               lb = [lb; [-inf * ones(num_old_act, 1); options.actuation_pattern(i, :)']];
               ub = [ub; [inf * ones(num_old_act, 1); options.actuation_pattern(i, :)']];
            end
            
            %TODO: break this up into smaller constraints
            obj = obj.addConstraint(BoundingBoxConstraint(lb, ub), obj.u_inds(:));
            
            
            %plant = plant.setNumInputs(1);
            
            %TODO: testing!!
            
%             all_xyz = [];
%             all_axis = [];
%             for i = 1:1:obj.plant.getNumBodies()
%                 parent = obj.plant.getBody(i);
%                 if ~parent.parent %TODO: really not the guard, I think, for floating plants
%                     continue; %skip over root which is grounded
%                 end
%                 rpy = sym(zeros(3, 1));
%                 xyz = sym(strcat('xyz', num2str(i)), [3, 1], 'real');
%                 all_xyz = [all_xyz; xyz];
%                 
%                 
%                 parentIdx = obj.plant.findLinkId(parent.linkname);
%                 fr = RigidBodyFrame(parentIdx,xyz,rpy,[parent.linkname,'_force_frame']);
%                 [obj.plant,frame_id] = addFrame(obj.plant,fr);
%                 axis = sym(strcat('axis', num2str(i)), [3, 1]);
%                 assume(axis, 'real');
%                 assume(axis, 'positive');
%                 all_axis = [all_axis; axis];
%                 fe = RigidBodyThrust(frame_id,axis, 1, [-inf, inf]);
%                 %fe.direct_feedthrough_flag = false;
%                 fe.name = [parent.linkname, 'thrust'];
%                 obj.plant = obj.plant.setForce(length(obj.plant.force) + 1, fe);
%             end
%             
%             %obj.plant = obj.plant.setNumInputs(obj.plant.getNumInputs() + obj.plant.getNumBodies());
%             obj.plant = compile(obj.plant);
            
            
            obj.initial_param_values = obj.plant.getParams();
            obj.costs_to_add = {};
            
            obj.actuation_weight = options.actuation_weight;
            
            paramsSym = sym(obj.plant.getParamFrame().getCoordinateNames(), 'real');
            

            obj.function_name = strcat(['testfunctioncolloc',obj.plant.name{1}]);
            
            nU = obj.plant.getNumInputs();
                nX = obj.plant.getNumOutputs();
                h = sym('h%d', [1, 1], 'real');
                u0 = sym('u0%d', [nU, 1], 'real');
                x0 = sym('x0%d', [nX, 1], 'real');
                u1 = sym('u1%d', [nU, 1], 'real');
                x1 = sym('x1%d', [nX, 1], 'real');
                xd0 = sym('xd0%d', [nX, 1], 'real');
                xd1 = sym('xd1%d', [nX, 1], 'real');
                dxd0 = sym('dxd0%d%d', [nX, 1 + nX + nU], 'real');
                dxd1 = sym('dxd1%d%d', [nX, 1 + nX + nU], 'real');
                
                data0.xdot = xd0;
                data0.dxdot = dxd0;
                data1.xdot = xd1;
                data1.dxdot = dxd1;
            
            if obj.generate_dynamics  && (exist(obj.function_name, 'file') ~= 3)
                
                h = sym('h%d', [1, 1], 'real');
                u0 = sym('u0%d', [plant.getNumInputs(), 1], 'real');
                u1 = sym('u1%d', [plant.getNumInputs(), 1], 'real');
                x0 = sym('x0%d', [plant.getNumOutputs(), 1], 'real');
                x1 = sym('x1%d', [plant.getNumOutputs(), 1], 'real');
                xdot0 = sym('xd0%d', [plant.getNumOutputs(), 1], 'real');
                xdot1 = sym('xd1%d', [plant.getNumOutputs(), 1], 'real');
                data0.xdot = xdot0;
                data1.xdot = xdot1;
                
                dynamicsEquation = constraint_fun(obj,h,x0,x1,u0,u1, paramsSym, data0, data1);
                fToMColloc(dynamicsEquation, plant);
                
%                 file_in = strcat(['dynamics_functioncolloc',obj.plant.name{1},'.c']);
%                 file_out = strcat(['testfunctioncolloc',obj.plant.name{1},'.c']);
%                 file_out_no_c = strcat(['testfunctioncolloc',obj.plant.name{1}]);
%                 python_call = sprintf('python edit_dynamics_stack_overflow.py %s %s', file_in, file_out);
%                 system(python_call)

                %new version
                file_out_no_c = strcat(['testfunctioncolloc',obj.plant.name{1}]);
                compile_call = strcat(['sh compile_mex.sh ', file_out_no_c]);
                system(compile_call);
                
                
                
                
                %              dynamicsEquation = tempPlant.dynamics(h, x0, u0);
                %              dynamicsJacobian = jacobian(dynamicsEquation, [h; x0; u0; paramsSym]);
                % %
                %              matlabFunction(dynamicsEquation, 'vars', {h, x0, u0, paramsSym}, 'file', strcat('dynamics_function', obj.plant.name{1}), 'Optimize',false);
                %              matlabFunction(dynamicsJacobian, 'vars', {h, x0, u0, paramsSym}, 'file', strcat('dynamics_jacobian', obj.plant.name{1}), 'Optimize',false);
                disp('Done writing files')
                
            elseif ~obj.generate_dynamics
              
                
                
                
                [dynamicsEquation, dynamicsJacobian] = constraint_fun(obj,h,x0,x1,u0,u1,paramsSym,data0,data1);
               
                
%                 matlabFunction(dynamicsEquation, 'vars', {h, x0, x1, u0, u1, paramsSym, xd0, xd1, dxd0, dxd1, all_xyz, all_axis}, 'file', strcat('dynamics_function_colloc', obj.plant.name{1}), 'Optimize',false);
%                 matlabFunction(dynamicsJacobian, 'vars', {h, x0, x1, u0, u1, paramsSym, xd0, xd1, dxd0, dxd1, all_xyz, all_axis}, 'file', strcat('dynamics_jacobian_colloc', obj.plant.name{1}), 'Optimize',false);
           
                  matlabFunction(dynamicsEquation, 'vars', {h, x0, x1, u0, u1, paramsSym, xd0, xd1, dxd0, dxd1}, 'file', strcat('dynamics_function_colloc', obj.plant.name{1}), 'Optimize',false);
                  matlabFunction(dynamicsJacobian, 'vars', {h, x0, x1, u0, u1, paramsSym, xd0, xd1, dxd0, dxd1}, 'file', strcat('dynamics_jacobian_colloc', obj.plant.name{1}), 'Optimize',false);

            end
        end
        
        
        
        function obj = setupVariables(obj, N)
            
            obj = setupVariables@DirectTrajectoryOptimization(obj,N);
            num_vars = obj.num_vars;
            params = obj.plant.getParams();
            param_names = params.getFrame().getCoordinateNames();
            obj = obj.addDecisionVariable(length(param_names), param_names);
            obj.param_inds =  (num_vars+1):1:(num_vars+length(param_names));%setup indices
            
            %ANDY NEW CODE
            nS = obj.getNumSuppl();
            obj = obj.addDecisionVariable(nS, {'suppl'});
            obj.suppl_inds = (obj.param_inds(end) + 1):(obj.param_inds(end) + nS);
            
        end
        
        function z0 = getInitialVars(obj,t_init,traj_init)
            %ANDYCODE
            z0 = getInitialVars@DirectTrajectoryOptimization(obj,t_init,traj_init);
            z0(obj.param_inds) = obj.plant.getParams().double;
            
            z0(obj.suppl_inds) = max(abs([-10000; 10000])); %TODO: fix this later to the real value
            
        end
        
        function [xtraj,utraj,z,F,info,xtraj0,utraj0] = solveTraj(obj,t_init,traj_init)
            
            if obj.zeroth_run
                param_vals = obj.initial_param_values.double;
                
                bounds_constraint = BoundingBoxConstraint(param_vals, param_vals);
                
                [obj, cnstr_idx] = obj.addConstraint(bounds_constraint, obj.param_inds);
                
                [xtraj,utraj,z,F,info] = solveTraj@DirectTrajectoryOptimization(obj,t_init,traj_init);
                obj = obj.deleteBoundingBoxConstraint(cnstr_idx); %Remove last bounding box constraint
                
                t_init = utraj.getBreaks();
                traj_init.x = xtraj;
                traj_init.u = utraj;
            end
            
            
            
            %Add lower bound constraints
            [lower_bounds, upper_bounds] = obj.plant.getParamLimits();
            % upper_bounds = inf(length(lower_bounds), 1);
            
            %             lower_bounds = obj.plant.getParams().double;
            %             upper_bounds = obj.plant.getParams().double;
            bounds_constraint = BoundingBoxConstraint(lower_bounds, upper_bounds);
            obj = obj.addConstraint(bounds_constraint, obj.param_inds);
            
            if obj.first_run
                [xtraj,utraj,z,F,info, ~] = solveTraj@DirectTrajectoryOptimization(obj,t_init,traj_init);
                
                t_init = utraj.getBreaks();
                traj_init.x = xtraj;
                traj_init.u = utraj;
                
                if ~isempty(obj.param_inds)
                    obj.plant = obj.plant.setParams(z(obj.param_inds(:)));
                end
                
            end
            disp('first run done')
            
            obj = obj.addActuation();
            %obj = obj.addFinalCost(@(T, x, params)obj.parameter_regularizer(T, x, params));
            
            for i = 1:1:length(obj.costs_to_add)
                cost_object = obj.costs_to_add{i};
                obj = obj.addCost(cost_object{1}, cost_object{2});
            end
            
            
            
            
            
            
            
            [xtraj,utraj,z,F,info] = solveTraj@DirectTrajectoryOptimization(obj,t_init,traj_init);
            
            
        end
        
        
        function obj = setRegularizerWeight(obj, regularizer_weight)
            obj.regularizer_weight = regularizer_weight;
        end
        
        function obj = setOptimizationType(obj, options)
            obj.penalty = options.penalty;
        end
        
        
        function num = getNumSuppl(obj)
            num = 1; %pretty dumb right now I admit
        end
        
        function [f, df] = actuation_minimizer(obj, suppl)
            %TODO: let's just make a new decision variable function,
            %operating on u and suppl only
            f = suppl * obj.actuation_weight;
            df = [obj.actuation_weight];
        end
        
        function [f, df] = parameter_regularizer(T, x, params)
            
            switch obj.penalty
                
                case 1
                    param_values = obj.initial_param_values;
                    w = obj.regularizer_weight;
                    e = 0.00000000000000001;
                    param_diff = (params - double(param_values));
                    
                    
                    f = w*sqrt(param_diff'*param_diff + e);
                    df = [0, zeros(length(x), 1)', w*param_diff' / sqrt(param_diff' * param_diff + e)];
                case 2
                    param_values = obj.initial_param_values;
                    if ~isempty(param_values.double)
                        w = obj.regularizer_weight;
                        f = w*norm(double(param_values) - params, 2)^2;
                        df = [0, zeros(length(x), 1)', 2*w*(params - double(param_values))'];
                    else
                        f = 0;
                        df = [0, zeros(length(x), 1)'];
                    end
                case 3
                    w = obj.regularizer_weight;
                    e = 0.00000000000000001;
                    
                    f = w*sqrt(params'*params + e);
                    df = [0, zeros(length(x), 1)', w*params' / sqrt(params' * params + e)];
                    
            end
        end
        
        
        function obj = addDynamicConstraints(obj)
            N = obj.N;
            nX = obj.plant.getNumStates();
            nU = obj.plant.getNumInputs();
            nP = obj.plant.getNumParams();
            constraints = cell(N-1,1);
            dyn_inds = cell(N-1,1);
            
            
            n_vars = 2*nX + 2*nU + 1 + nP;
            
            if obj.generate_dynamics
                cnstr = FunctionHandleConstraint(zeros(nX,1),zeros(nX,1),n_vars,@obj.constraint_fun_param);
            else
                cnstr = FunctionHandleConstraint(zeros(nX,1),zeros(nX,1),n_vars,@obj.constraint_fun_old);
            end
            cnstr = setName(cnstr,'collocation');
            
            if obj.generate_dynamics
                cnstr.grad_method = 'numerical';
                cnstr.grad_level = 0;
            end
            
            % create shared data functions to calculate dynamics at the knot
            % points
            shared_data_index = obj.getNumSharedDataFunctions;
            for i=1:obj.N,
                obj = obj.addSharedDataFunction(@obj.dynamics_data,{obj.x_inds(:,i);obj.u_inds(:,i)});
            end
            
            for i=1:obj.N-1,
                dyn_inds{i} = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.u_inds(:,i+1); obj.param_inds(:)};
                constraints{i} = cnstr;
                
                obj = obj.addConstraint(constraints{i}, dyn_inds{i},[shared_data_index+i;shared_data_index+i+1]);
            end
        end
        
        function [f,df] = constraint_fun(obj,h,x0,x1,u0,u1,params,data0,data1)
            % calculate xdot at knot points
            %  [xdot0,dxdot0] = obj.plant.dynamics(0,x0,u0);
            %  [xdot1,dxdot1] = obj.plant.dynamics(0,x1,u1);
            
            pObj = obj.plant.setParams(params);
            
            nX = pObj.getNumStates();
            nU = pObj.getNumInputs();
            
            % use the shared data objects for the dynamics at the knot points
            xdot0 = data0.xdot;
            dxdot0 = data0.dxdot;
            xdot1 = data1.xdot;
            dxdot1 = data1.dxdot;
            
            % cubic interpolation to get xcol and xdotcol, as well as
            % derivatives
            xcol = .5*(x0+x1) + h/8*(xdot0-xdot1);
            dxcol = [1/8*(xdot0-xdot1) (.5*eye(nX) + h/8*dxdot0(:,2:1+nX)) ...
                (.5*eye(nX) - h/8*dxdot1(:,2:1+nX)) h/8*dxdot0(:,nX+2:1+nX+nU) -h/8*dxdot1(:,nX+2:1+nX+nU)];
            xdotcol = -1.5*(x0-x1)/h - .25*(xdot0+xdot1);
            dxdotcol = [1.5*(x0-x1)/h^2 (-1.5*eye(nX)/h - .25*dxdot0(:,2:1+nX)) ...
                (1.5*eye(nX)/h - .25*dxdot1(:,2:1+nX)) -.25*dxdot0(:,nX+2:1+nX+nU) -.25*dxdot1(:,nX+2:1+nX+nU)];
            
            % evaluate xdot at xcol, using foh on control input
            
            %Some temporary variables just for calculating the dynamics
            %expression
            u_temp = sym('utemp', [pObj.getNumInputs(), 1], 'real');
            x_temp = sym('xtemp', [pObj.getNumOutputs(), 1], 'real');
            %[g] = pObj.dynamics(0,xcol,.5*(u0+u1));
            
            %Calculate the general dynamics expression
            [g] = pObj.dynamics(h, x_temp,u_temp);
            dgdxcol = jacobian(g, [h; x_temp; u_temp]);
            
            %and now substitute in:
            g = subs(g, x_temp, xcol);
            g = subs(g, u_temp, .5*(u0+u1));
            dgdxcol = subs(dgdxcol, x_temp, xcol);
            dgdxcol = subs(dgdxcol, u_temp, .5*(u0+u1));

            
            
            
            dg = dgdxcol(:,2:1+nX)*dxcol + [zeros(nX,1+2*nX) .5*dgdxcol(:,2+nX:1+nX+nU) .5*dgdxcol(:,2+nX:1+nX+nU)];
            
            % constrait is the difference between the two
            f = xdotcol - g;
            df = dxdotcol - dg;
            dfdp = jacobian(f, [params]);
            df = [df, dfdp];
        end
        
        %TODO: get rid of this eventually
        function [f,df] = constraint_fun_orig(obj,h,x0,x1,u0,u1,data0,data1)
            % calculate xdot at knot points
            %  [xdot0,dxdot0] = obj.plant.dynamics(0,x0,u0);
            %  [xdot1,dxdot1] = obj.plant.dynamics(0,x1,u1);
            
            nX = obj.plant.getNumStates();
            nU = obj.plant.getNumInputs();
            
            % use the shared data objects for the dynamics at the knot points
            xdot0 = data0.xdot;
            dxdot0 = data0.dxdot;
            xdot1 = data1.xdot;
            dxdot1 = data1.dxdot;
            
            % cubic interpolation to get xcol and xdotcol, as well as
            % derivatives
            xcol = .5*(x0+x1) + h/8*(xdot0-xdot1);
            dxcol = [1/8*(xdot0-xdot1) (.5*eye(nX) + h/8*dxdot0(:,2:1+nX)) ...
                (.5*eye(nX) - h/8*dxdot1(:,2:1+nX)) h/8*dxdot0(:,nX+2:1+nX+nU) -h/8*dxdot1(:,nX+2:1+nX+nU)];
            xdotcol = -1.5*(x0-x1)/h - .25*(xdot0+xdot1);
            dxdotcol = [1.5*(x0-x1)/h^2 (-1.5*eye(nX)/h - .25*dxdot0(:,2:1+nX)) ...
                (1.5*eye(nX)/h - .25*dxdot1(:,2:1+nX)) -.25*dxdot0(:,nX+2:1+nX+nU) -.25*dxdot1(:,nX+2:1+nX+nU)];
            
            % evaluate xdot at xcol, using foh on control input
            [g,dgdxcol] = obj.plant.dynamics(0,xcol,.5*(u0+u1));
            dg = dgdxcol(:,2:1+nX)*dxcol + [zeros(nX,1+2*nX) .5*dgdxcol(:,2+nX:1+nX+nU) .5*dgdxcol(:,2+nX:1+nX+nU)];
            
            % constrait is the difference between the two
            f = xdotcol - g;
            df = dxdotcol - dg;
        end
        
        
        function [f] = constraint_fun_param(obj,h,x0,x1,u0,u1, params, data0,data1)
            %TODO: this doesn't work . I will need to write my own numeric
            %derivatives and then compose it from a few basic functions
            f = feval(strcat(['testfunctioncolloc',obj.plant.name{1}]), h, x0, x1, u0, u1, data0.xdot, data1.xdot, data0.dxdot, data1.dxdot, params);
        end
        
        function [f,df] = constraint_fun_old(obj,h,x0,x1,u0,u1, params, data0,data1)
            dyn_fun_name = strcat('dynamics_function_colloc', obj.plant.name{1});
            dyn_jac_name = strcat('dynamics_jacobian_colloc', obj.plant.name{1});
            nU = obj.plant.getNumInputs();
            f = feval(dyn_fun_name, h, x0, x1, u0, u1, params, data0.xdot, data1.xdot, data0.dxdot, data1.dxdot);
            df = feval(dyn_jac_name, h, x0, x1, u0, u1, params, data0.xdot, data1.xdot, data0.dxdot, data1.dxdot);
            
         %   [f_t, df_t] = obj.constraint_fun(h,x0,x1,u0,u1, params, data0,data1);
        end
        
        
        
        
        function data = dynamics_data(obj,x,u)
            [data.xdot,data.dxdot] = obj.plant.dynamics(0,x,u);
        end
        
        %ANDYCODE
        function obj = addActuationCost(obj)
            nS = obj.getNumSuppl();
            actuation_cost = FunctionHandleObjective(nS, @(suppl)obj.actuation_minimizer(suppl));
            actuation_cost.setName('actuation1');
            obj.costs_to_add{length(obj.costs_to_add) + 1} = {actuation_cost,{obj.suppl_inds(:)}};
        end
        
        
        
        function obj = addActuationConstraints(obj)
            all_u_inds = obj.u_inds;
            for i = 1:1:obj.N
                u_inds = all_u_inds(:, i);
                constraint = constructActuationConstraint(obj.plant.getNumInputs(), obj.getNumSuppl());
                obj = obj.addConstraint(constraint,  [u_inds(:); obj.suppl_inds(:)]);
            end
            
        end
        
        function obj = addActuation(obj)
            obj = obj.addActuationCost();
            obj = obj.addActuationConstraints(); %these constraints should be trivial to fulfill in the firs tround
        end
        
        function obj = addRunningCost(obj,running_cost_function)
            % Adds an integrated cost to all time steps, which is
            % numerical implementation specific (thus abstract)
            % this cost is assumed to be time-invariant
            % @param running_cost_function a function handle
            %  of the form running_cost_function(dt,x,u)
            % This implementation assumes a ZOH, but where the values of
            % x(i),u(i) are held over an interval spanned by .5(dt(i-1) + dt(i))
            
            nX = obj.plant.getNumStates();
            nU = obj.plant.getNumInputs();
            nP = obj.plant.getNumParams();
            
            running_cost_end = FunctionHandleObjective(1+nX+nU+nP,@(h,x,u) obj.running_fun_end(running_cost_function,h,x,u));
            running_cost_mid = FunctionHandleObjective(2+nX+nU+nP,@(h0,h1,x,u) obj.running_fun_mid(running_cost_function,h0,h1,x,u));
            
            %Costs to add:
            
            inds_1 = {obj.h_inds(1);obj.x_inds(:,1);obj.u_inds(:,1); obj.param_inds(:)};
            
            %obj = obj.addCost(running_cost_end,inds_1);
            obj.costs_to_add{length(obj.costs_to_add) + 1} = {running_cost_end,inds_1};
            
            for i=2:obj.N-1,
                inds_i = {obj.h_inds(i);obj.x_inds(:,i);obj.u_inds(:,i); obj.param_inds(:)};
                %obj = obj.addCost(running_cost_mid,{obj.h_inds(i-1);obj.h_inds(i);obj.x_inds(:,i);obj.u_inds(:,i)});
                obj.costs_to_add{length(obj.costs_to_add) + 1} = {running_cost_mid,inds_i};
            end
            
            inds_end = {obj.h_inds(end);obj.x_inds(:,end);obj.u_inds(:,end); obj.param_inds(:)};
            %obj = obj.addCost(running_cost_end,{obj.h_inds(end);obj.x_inds(:,end);obj.u_inds(:,end)});
            obj.costs_to_add{length(obj.costs_to_add) + 1} = {running_cost_end,inds_end};
        end
        
        %ANDYCODE
        function obj = addFinalCost(obj,final_cost_function)
            % adds a cost to the final state and total time
            % @param final_cost_function a function handle f(T,xf)
            nX = obj.plant.getNumStates();
            nH = obj.N-1;
            nP = obj.plant.getNumParams();
            cost = FunctionHandleObjective(nH+nX+nP,@(h,x, params) obj.final_cost(final_cost_function,h,x, params));
            
            cost = cost.setName(strcat('regularizer', 1));
            obj.costs_to_add{length(obj.costs_to_add) + 1} = {cost,{obj.h_inds;obj.x_inds(:,end); obj.param_inds(:)}};
            obj = obj.addCost(cost,{obj.h_inds;obj.x_inds(:,end); obj.param_inds(:)});
        end
        
        function [utraj,xtraj] = reconstructInputTrajectory(obj,z)
            % Interpolate between knot points to reconstruct a trajectory using
            % the hermite spline
            t = [0; cumsum(z(obj.h_inds))];
            u = reshape(z(obj.u_inds),[],obj.N);
            utraj = PPTrajectory(foh(t,u));
            utraj = utraj.setOutputFrame(obj.plant.getInputFrame);
        end
        
        function xtraj = reconstructStateTrajectory(obj,z)
            % Interpolate between knot points to reconstruct a trajectory using
            % the hermite spline
            t = [0; cumsum(z(obj.h_inds))];
            u = reshape(z(obj.u_inds),[],obj.N);
            
            x = reshape(z(obj.x_inds),[],obj.N);
            xdot = zeros(size(x,1),obj.N);
            for i=1:obj.N,
                xdot(:,i) = obj.plant.dynamics(t(i),x(:,i),u(:,i));
            end
            xtraj = PPTrajectory(pchipDeriv(t,x,xdot));
            xtraj = xtraj.setOutputFrame(obj.plant.getStateFrame);
        end
    end
    
    methods (Access = protected)
        function [f,df] = running_fun_end(obj,running_handle,h,x,u)
            [f,dg] = running_handle(.5*h,x,u);
            
            df = [.5*dg(:,1) dg(:,2:end)];
        end
        
        function [f,df] = running_fun_mid(obj,running_handle,h0,h1,x,u)
            [f,dg] = running_handle(.5*(h0+h1),x,u);
            
            df = [.5*dg(:,1) .5*dg(:,1) dg(:,2:end)];
        end
        
    end
end