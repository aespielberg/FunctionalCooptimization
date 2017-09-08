classdef DirtranTrajectoryOptimizationTest < DirectTrajectoryOptimization
    % Direct transcription trajectory optimization
    %  implements multiple possible integration schemes for the dynamics
    %  constraints xdot = f(x,u) and for for integrating the running cost
    %  term.
    %
    %  For forward euler integratino:
    %    dynamics constraints are: x(k+1) = x(k) + h(k)*f(x(k),u(k))
    %    integrated cost is sum of g(h(k),x(k),u(k))
    %  For backward euler integration:
    %    dynamics constraints are: x(k+1) = x(k) + h(k)*f(x(k+1),u(k))
    %    integrated cost is sum of g(h(k),x(k+1),u(k))
    %  For midpoint integration:
    %    dynamics constraints are: x(k+1) = x(k) + h(k)*f(.5*x(k)+.5*x(k+1),.5*u(k)+.5*u(k+1))
    %    integrated cost is sum of g(h(k),.5*x(k)+.5*x(k+1),.5*u(k)+.5*u(k+1))
    properties (Constant)
        FORWARD_EULER = 1;
        BACKWARD_EULER = 2;
        MIDPOINT = 3;  % DEFAULT
        
    end
    
    properties
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
        function obj = DirtranTrajectoryOptimizationTest(plant,N,duration,options)
            if nargin < 4
                options = struct();
            end
            if ~isfield(options,'integration_method')
                options.integration_method = DirtranTrajectoryOptimization.MIDPOINT;
            end
            
            if ~isfield(options,'actuation_weight')
                options.actuation_weight = 0.0;
            end
            
            obj = obj@DirectTrajectoryOptimization(plant,N,duration,options);
            
            obj.initial_param_values = plant.getParams();
            obj.costs_to_add = {};
            
            obj.actuation_weight = options.actuation_weight;
            
            paramsSym = sym(plant.getParamFrame().getCoordinateNames(), 'real');
            
            if ~isempty(paramsSym)
                tempPlant = plant.setParams( paramsSym );
                
            else
                tempPlant = plant;
            end
            
            obj.function_name = strcat(['testfunction',obj.plant.name{1}]);
            
            dynamics_function_name = strcat('dynamics_function', obj.plant.name{1});
            
            if obj.generate_dynamics  && (exist(obj.function_name, 'file') ~= 3)
                
                h = sym('h%d', [1, 1], 'real');
                u0 = sym('u0%d', [plant.getNumInputs(), 1], 'real');
                u1 = sym('u1%d', [plant.getNumInputs(), 1], 'real');
                x0 = sym('x0%d', [plant.getNumOutputs(), 1], 'real');
                x1 = sym('x1%d', [plant.getNumOutputs(), 1], 'real');
                
                dynamicsEquation = midpoint_constraint_fun(obj,h,x0,x1,u0,u1, paramsSym);
                fToMNoContacts(dynamicsEquation, plant, 0);
                
                file_in = strcat(['dynamics_function',obj.plant.name{1},'.c']);
                file_out = strcat(['testfunction',obj.plant.name{1},'.c']);
                file_out_no_c = strcat(['testfunction',obj.plant.name{1}]);
                %python_call = sprintf('python edit_dynamics_stack_overflow.py %s %s', file_in, file_out);
                %system(python_call)
                %compile_call = strcat(['sh compile_mex.sh ', file_out_no_c]);
                %system(compile_call);
                
                
                
                
                %              dynamicsEquation = tempPlant.dynamics(h, x0, u0);
                %              dynamicsJacobian = jacobian(dynamicsEquation, [h; x0; u0; paramsSym]);
                % %
                %              matlabFunction(dynamicsEquation, 'vars', {h, x0, u0, paramsSym}, 'file', strcat('dynamics_function', obj.plant.name{1}), 'Optimize',false);
                %              matlabFunction(dynamicsJacobian, 'vars', {h, x0, u0, paramsSym}, 'file', strcat('dynamics_jacobian', obj.plant.name{1}), 'Optimize',false);
                disp('Done writing files')
                
            elseif ~obj.generate_dynamics && exist(dynamics_function_name, 'file') ~= 2
                h = sym('h%d', [1, 1], 'real');
                u0 = sym('u0%d', [plant.getNumInputs(), 1], 'real');
                x0 = sym('x0%d', [plant.getNumOutputs(), 1], 'real');
                
                dynamicsEquation = tempPlant.dynamics(h, x0, u0);
                dynamicsJacobian = jacobian(dynamicsEquation, [h; x0; u0; paramsSym]);
                %
                matlabFunction(dynamicsEquation, 'vars', {h, x0, u0, paramsSym}, 'file', dynamics_function_name, 'Optimize',false);
                matlabFunction(dynamicsJacobian, 'vars', {h, x0, u0, paramsSym}, 'file', strcat('dynamics_jacobian', obj.plant.name{1}), 'Optimize',false);
            end
            
        end
        
        function z0 = getInitialVars(obj,t_init,traj_init)
            %ANDYCODE
            z0 = getInitialVars@DirectTrajectoryOptimization(obj,t_init,traj_init);
            z0(obj.param_inds) = obj.plant.getParams().double;
            
            z0(obj.suppl_inds) = max(abs([-10000; 10000])); %TODO: fix this later to the real value
            
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
            
            obj = obj.addActuation();
           % obj = obj.addFinalCost(@(T, x, params)obj.parameter_regularizer(T, x, params));
            
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
        
        function [f, df] = parameter_regularizer(obj, T, x, params)
            
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
            nX = obj.plant.getNumStates();
            nU = obj.plant.getNumInputs();
            nP = obj.plant.getNumParams();
            N = obj.N;
            
            constraints = cell(N-1,1);
            dyn_inds = cell(N-1,1);
            
            switch obj.options.integration_method
                case DirtranTrajectoryOptimization.FORWARD_EULER
                    n_vars = 2*nX + nU + 1 + nP;
                    cnstr = FunctionHandleConstraint(zeros(nX,1),zeros(nX,1),n_vars,@obj.forward_constraint_fun);
                case DirtranTrajectoryOptimization.BACKWARD_EULER
                    n_vars = 2*nX + nU + 1 + nP;
                    cnstr = FunctionHandleConstraint(zeros(nX,1),zeros(nX,1),n_vars,@obj.backward_constraint_fun);
                case DirtranTrajectoryOptimization.MIDPOINT
                    n_vars = 2*nX + 2*nU + 1 + nP;
                    if obj.generate_dynamics
                        cnstr = FunctionHandleConstraint(zeros(nX,1),zeros(nX,1),n_vars,@obj.midpoint_constraint_fun_param);
                    else
                        cnstr = FunctionHandleConstraint(zeros(nX,1),zeros(nX,1),n_vars,@obj.midpoint_constraint_fun_old);
                    end
                otherwise
                    error('Drake:DirtranTrajectoryOptimization:InvalidArgument','Unknown integration method');
            end
            if obj.generate_dynamics
                cnstr.grad_method = 'numerical';
                cnstr.grad_level = 0;
            end
            
            for i=1:obj.N-1,
                switch obj.options.integration_method
                    case DirtranTrajectoryOptimization.FORWARD_EULER
                        dyn_inds{i} = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i); obj.param_inds(:)};
                    case DirtranTrajectoryOptimization.BACKWARD_EULER
                        dyn_inds{i} = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i); obj.param_inds(:)};
                    case DirtranTrajectoryOptimization.MIDPOINT
                        dyn_inds{i} = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.u_inds(:,i+1); obj.param_inds(:)};
                    otherwise
                        error('Drake:DirtranTrajectoryOptimization:InvalidArgument','Unknown integration method');
                end
                constraints{i} = cnstr;
                
                obj = obj.addConstraint(constraints{i}, dyn_inds{i});
            end
        end
        
        %         function obj = addRunningCost(obj,running_cost_function)
        %             % Adds an integrated cost to all time steps, which is
        %             % numerical implementation specific (thus abstract)
        %             % this cost is assumed to be time-invariant
        %             % @param running_cost_function a function handle
        %             %  of the form running_cost_function(dt,x,u)
        %
        %             nX = obj.plant.getNumStates();
        %             nU = obj.plant.getNumInputs();
        %
        %
        %             for i=1:obj.N-1,
        %                 switch obj.options.integration_method
        %                     case DirtranTrajectoryOptimization.FORWARD_EULER
        %                         running_cost = FunctionHandleObjective(1+nX+nU, running_cost_function);
        %                         inds_i = {obj.h_inds(i);obj.x_inds(:,i);obj.u_inds(:,i)};
        %                     case DirtranTrajectoryOptimization.BACKWARD_EULER
        %                         running_cost = FunctionHandleObjective(1+nX+nU, running_cost_function);
        %                         inds_i = {obj.h_inds(i);obj.x_inds(:,i+1);obj.u_inds(:,i)};
        %                     case DirtranTrajectoryOptimization.MIDPOINT
        %                         running_cost = FunctionHandleObjective(1+2*nX+2*nU,...
        %                             @(h,x0,x1,u0,u1) obj.midpoint_running_fun(running_cost_function,h,x0,x1,u0,u1));
        %                         inds_i = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.u_inds(:,i+1)};
        %                     otherwise
        %                         error('Drake:DirtranTrajectoryOptimization:InvalidArgument','Unknown integration method');
        %                 end
        %
        %                 obj = obj.addCost(running_cost,inds_i);
        %             end
        %         end
        %     end
        
        %ANDYCODE
        function obj = addActuationCost(obj)
            nS = obj.getNumSuppl();
            actuation_cost = FunctionHandleObjective(nS, @(suppl)obj.actuation_minimizer(suppl));
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
            
            nX = obj.plant.getNumStates();
            nU = obj.plant.getNumInputs();
            nP = obj.plant.getNumParams();
            
            for i=1:obj.N-1,
                switch obj.options.integration_method
                    case DirtranTrajectoryOptimization.FORWARD_EULER
                        running_cost = FunctionHandleObjective(1+nX+nU+nP, running_cost_function);
                        inds_i = {obj.h_inds(i);obj.x_inds(:,i);obj.u_inds(:,i); obj.param_inds(:)};
                    case DirtranTrajectoryOptimization.BACKWARD_EULER
                        running_cost = FunctionHandleObjective(1+nX+nU+nP, running_cost_function);
                        inds_i = {obj.h_inds(i);obj.x_inds(:,i+1);obj.u_inds(:,i); obj.param_inds(:)};
                    case DirtranTrajectoryOptimization.MIDPOINT
                        running_cost = FunctionHandleObjective(1+2*nX+2*nU+nP,...
                            @(h,x0,x1,u0,u1,params) obj.midpoint_running_fun(running_cost_function,h,x0,x1,u0,u1,params));
                        inds_i = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.u_inds(:,i+1); obj.param_inds(:)};
                    otherwise
                        error('Drake:DirtranTrajectoryOptimization:InvalidArgument','Unknown integration method');
                end
                running_cost = running_cost.setName(strcat('objective', num2str(i)));
                obj.costs_to_add{length(obj.costs_to_add) + 1} = {running_cost,inds_i};
                %obj = obj.addCost(running_cost,inds_i);
            end
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
            %obj = obj.addCost(cost,{obj.h_inds;obj.x_inds(:,end); obj.param_inds(:)});
        end
        
        
    end
    
    
    methods (Access=protected)
        function [f,df] = forward_constraint_fun(obj,h,x0,x1,u, params)
            params_poly = obj.plant.getParamFrame().getPoly;
            pObj = obj.plant.setParams(params_poly);
            nX = obj.plant.getNumStates();
            [xdot,dxdot] = pObj.dynamics(0,x0,u);
            
            
            
            f = x1 - x0 - h*xdot;
            
            df = [-xdot (-eye(nX) - h*dxdot(:,2:1+nX)) eye(nX) -h*dxdot(:,nX+2:end) zeros(nX, length(params))];
        end
        
        function [f,df] = backward_constraint_fun(obj,h,x0,x1,u, params)
            params_poly = obj.plant.getParamFrame().getPoly;
            pObj = obj.plant.setParams(params_poly);
            nX = obj.plant.getNumStates();
            [xdot,dxdot] = pObj.dynamics(0,x1,u);
            f = x1 - x0 - h*xdot;
            df = [-xdot -eye(nX) (eye(nX) - h*dxdot(:,2:1+nX)) -h*dxdot(:,nX+2:end) zeros(nX, length(params))];
        end
        
        function [f,df] = midpoint_constraint_fun_old(obj,h,x0,x1,u0,u1, params)
            
            %pObj = obj.plant.setParams(params_poly);
            %pObj = obj.plant.setParams(params);
            nX = obj.plant.getNumStates();
            nP = obj.plant.getNumParams();
            
            dfunction = str2func(strcat('dynamics_function', obj.plant.name{1}));
            djacobian = str2func(strcat('dynamics_jacobian', obj.plant.name{1}));
            
            xdot = dfunction(0,.5*(x0+x1),.5*(u0+u1), params);
            dxdot = djacobian(0,.5*(x0+x1),.5*(u0+u1), params);
            %[xdot,dxdot] = pObj.dynamics(0,.5*(x0+x1),.5*(u0+u1));
            f = x1 - x0 - h*xdot;
            dfdparams = -h * dxdot(:, end-nP + 1:end);
            
            df = [-xdot (-eye(nX) - .5*h*dxdot(:,2:1+nX)) (eye(nX)- .5*h*dxdot(:,2:1+nX)) -.5*h*dxdot(:,nX+2:end-nP) -.5*h*dxdot(:,nX+2:end-nP) dfdparams];
        end
        
        function [f] = midpoint_constraint_fun_param(obj,h,x0,x1,u0,u1, params)
            f = feval(strcat(['testfunction',obj.plant.name{1}]), h, x0, x1, u0, u1, params);
        end
        
        function [f,df] = midpoint_constraint_fun(obj,h,x0,x1,u0,u1, params)
            pObj = obj.plant.setParams(params);
            nX = pObj.getNumStates();
            [xdot] = pObj.dynamics(0,.5*(x0+x1),.5*(u0+u1));
            f = x1 - x0 - h*xdot;
            %df = [-xdot (-eye(nX) - .5*h*dxdot(:,2:1+nX)) (eye(nX)- .5*h*dxdot(:,2:1+nX)) -.5*h*dxdot(:,nX+2:end) -.5*h*dxdot(:,nX+2:end)];
        end
        
        %         function [f,df] = midpoint_running_fun(obj,running_handle,h,x0,x1,u0,u1)
        %             nX = obj.plant.getNumStates();
        %             nU = obj.plant.getNumInputs();
        %             [f,dg] = running_handle(h,.5*(x0+x1),.5*(u0+u1));
        %
        %             df = [dg(:,1) .5*dg(:,2:1+nX) .5*dg(:,2:1+nX) .5*dg(:,2+nX:1+nX+nU) .5*dg(:,2+nX:1+nX+nU)];
        %         end
        
        %TODO: also add the others in here
        function [f,df] = midpoint_running_fun(obj,running_handle,h,x0,x1,u0,u1, params)
            nX = obj.plant.getNumStates();
            nU = obj.plant.getNumInputs();
            nP = obj.plant.getNumParams();
            [f,dg] = running_handle(h,.5*(x0+x1),.5*(u0+u1), params);
            
            df = [dg(:,1) .5*dg(:,2:1+nX) .5*dg(:,2:1+nX) .5*dg(:,2+nX:1+nX+nU) .5*dg(:,2+nX:1+nX+nU) dg(:,2+nX+nU:1+nX+nU+nP)];
        end
        
        function [f,df] = final_cost(obj,final_cost_function,h,x, params)
            T = sum(h);
            
            %Hopefully the function is either defined here or is passed in
            try
                [f,dg] = final_cost_function(T,x, params);
            catch
                [f,dg] = final_cost_function(T,x, params); %TODO: why did I need this?
            end
            df = [repmat(dg(:,1),1,length(h)) dg(:,2:end)];
        end
    end
end