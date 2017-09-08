classdef ContactImplicitTrajectoryOptimizationTest < DirectTrajectoryOptimization
    % phi, lambda
    properties
        nC
        nD % number of friction elements per contact
        
        l_inds % orderered [lambda_N;lambda_f1;lambda_f2;...;gamma] for each contact sequentially
        lfi_inds % nD x nC indexes into lambda for each time step
        lambda_mult
        ljl_inds  % joint limit forces
        jl_lb_ind  % joint indices where the lower bound is finite
        jl_ub_ind % joint indices where the lower bound is finite
        nJL % number of joint limits = length([jl_lb_ind;jl_ub_ind])
        
        nonlincompl_constraints
        nonlincompl_slack_inds
        
        %Andy properties:
        param_inds %the indices of the parameters
        regularizer_weight
        penalty
        initial_param_values
        costs_to_add
        
        
        
        
    end
    
    properties (Constant)
        FORWARD_EULER = 1;
        BACKWARD_EULER = 2;
        MIDPOINT = 3;  % DEFAULT
        MIXED = 4;   % matched to TimeSteppingRigidBodyManipulator. Forward on qd, backward on q
    end
    
    methods
        function obj = ContactImplicitTrajectoryOptimizationTest(plant,N,duration,options)
            if nargin<4, options=struct(); end
            
            if ~isfield(options,'nlcc_mode')
                options.nlcc_mode = 2;
            end
            if ~isfield(options,'lincc_mode')
                options.lincc_mode = 1;
            end
            if ~isfield(options,'compl_slack')
                options.compl_slack = 0;
            end
            if ~isfield(options,'lincompl_slack')
                options.lincompl_slack = 0;
            end
            if ~isfield(options,'jlcompl_slack')
                options.jlcompl_slack = 0;
            end
            if ~isfield(options,'lambda_mult')
                options.lambda_mult = 1;
            end
            if ~isfield(options,'lambda_jl_mult')
                options.lambda_jl_mult = 1;
            end
            if ~isfield(options,'active_collision_options')
                options.active_collision_options.terrain_only = true;
            end
            if ~isfield(options,'integration_method')
                options.integration_method = ContactImplicitTrajectoryOptimizationTest.MIDPOINT;
            end
            
            obj = obj@DirectTrajectoryOptimization(plant,N,duration,options);
            
            
            %Andy additions:
            obj.initial_param_values = plant.getParams();
            obj.costs_to_add = {};
            obj = obj.setOptimizationType(options);
            
        end
        
       
        
        %ANDY CODE - regularization stuff:
        function obj = setRegularizerWeight(obj, regularizer_weight)
            obj.regularizer_weight = regularizer_weight;
        end
        
        function obj = setOptimizationType(obj, options)
            obj.penalty = options.penalty;
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
                    w = obj.regularizer_weight;
                    f = w*norm(double(param_values) - params, 2)^2;
                    df = [0, zeros(length(x), 1)', 2*w*(params' - double(param_values)')];
                    
            end
        end
        
        function [f, df] = velocity(obj, T, x, params)
            
            f = x(1) / T;
            disp('f is')
            f
            
            df = [-x(1)/T^2, 1/T, zeros(1, length(x) - 1), zeros(1, length(params))];
            
            
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
                    w = obj.regularizer_weight;
                    disp('other f is ')
                    f = w*norm(double(param_values) - params, 2)^2
                    df = [0, zeros(length(x), 1)', 2*w*(params' - double(param_values)')];
                    
            end
        end
        
        function obj = addDynamicConstraints(obj)
            nX = obj.plant.getNumStates();
            nU = obj.plant.getNumInputs();
            nq = obj.plant.getNumPositions();
            nP = obj.plant.getNumParams();
            N = obj.N;
            
            constraints = cell(N-1,1);
            lincompl_constraints = cell(N-1,1);
            obj.nonlincompl_constraints = cell(N-1,1);
            obj.nonlincompl_slack_inds = cell(N-1,1);
            jlcompl_constraints = cell(N-1,1);
            dyn_inds = cell(N-1,1);
            
            %Andy edited this all to add in the nP
            n_vars = 2*nX + nU + 1 + obj.nC*(2+obj.nD) + obj.nJL + nP;
            cnstr = FunctionHandleConstraint(zeros(nX,1),zeros(nX,1),n_vars,@obj.dynamics_constraint_fun);
            
            %ANDY edit: everything numerical
            cnstr.grad_method = 'numerical';
            cnstr.grad_level = 0;
            
            %Andy TODO: I think I need to set the params here or something first
            [~,~,~,~,~,~,~,mu] = obj.plant.contactConstraints(zeros(nq,1),false,obj.options.active_collision_options);
            
            %Andy added in param_inds here
            for i=1:obj.N-1,
                %         dyn_inds{i} = [obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.l_inds(:,i);obj.ljl_inds(:,i)];
                dyn_inds{i} = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.l_inds(:,i);obj.ljl_inds(:,i); ; obj.param_inds(:)};
                constraints{i} = cnstr;
                
                obj = obj.addConstraint(constraints{i}, dyn_inds{i});
                
                if obj.nC > 0
                    % indices for (i) gamma
                    gamma_inds = obj.l_inds(obj.nD+2:obj.nD+2:end,i);
                    % awkward way to pull out these indices, for (i) lambda_N and
                    % lambda_f
                    lambda_inds = obj.l_inds(repmat((1:1+obj.nD)',obj.nC,1) + kron((0:obj.nC-1)',(2+obj.nD)*ones(obj.nD+1,1)),i);
                    
                    %Andy TODO: need to modify this
                    
                    nonlincompl_constraints2{i} = NonlinearComplementarityConstraint(@nonlincompl_fun,nX + obj.nC,obj.nC*(1+obj.nD), obj.options.nlcc_mode,obj.options.compl_slack);
                    obj.nonlincompl_constraints{i} = ParametricNonlinearComplementarityConstraint(obj.plant, nX + obj.nC,obj.nC*(1+obj.nD), length(obj.param_inds), obj.options.compl_slack, obj.options.active_collision_options, nonlincompl_constraints2{i});
                    
                    %obj.nonlincompl_constraints{i} = NonlinearComplementarityConstraint(@nonlincompl_fun,nX + obj.nC,obj.nC*(1+obj.nD), obj.options.nlcc_mode,obj.options.compl_slack);
                    
                    
                    obj.nonlincompl_slack_inds{i} = obj.num_vars+1:obj.num_vars + obj.nonlincompl_constraints{i}.n_slack;

                    obj = obj.addConstraint(obj.nonlincompl_constraints{i},[obj.x_inds(:,i+1);gamma_inds;lambda_inds;obj.param_inds']);
                    %obj = obj.addConstraint(obj.nonlincompl_constraints{i},[obj.x_inds(:,i+1);gamma_inds;lambda_inds]);
                    
                    
                    % linear complementarity constraint
                    %   gamma /perp mu*lambda_N - sum(lambda_fi)
                    %
                    %  Generate terms W,r,M,gamma_inds so that
                    %  gamma = y(gamma_inds)
                    %  Wz+Mx+r = mu*lambda_N - sum(lambda_fi)
                    r = zeros(obj.nC,1);
                    W = zeros(obj.nC,obj.nC);
                    M = zeros(obj.nC,obj.nC*(1+obj.nD));
                    for k=1:obj.nC,
                        M(k,1 + (k-1)*(1+obj.nD)) = mu(k);
                        M(k,(2:obj.nD+1) + (k-1)*(1+obj.nD)) = -ones(obj.nD,1);
                    end
                    
                    %Andy TODO: need to modify this
                    
                    %TODO: really remove this addition
                    
                    lincompl_constraints2{i} =  LinearComplementarityConstraint(W,r,M,obj.options.lincc_mode,obj.options.lincompl_slack);
                    lincompl_constraints{i} = ParametricLinearComplementarityConstraint(obj.plant,obj.options.lincc_mode,obj.options.lincompl_slack, lincompl_constraints2{i});

                    %lincompl_constraints{i} =  LinearComplementarityConstraint(W,r,M,obj.options.lincc_mode,obj.options.lincompl_slack);
                    
                    %obj = obj.addConstraint(lincompl_constraints{i},[lambda_inds;gamma_inds]);
                    
                    obj = obj.addConstraint(lincompl_constraints{i},[lambda_inds;gamma_inds; obj.param_inds']);

                end
                
                if obj.nJL > 0
                    % joint limit linear complementarity constraint
                    % lambda_jl /perp [q - lb_jl; -q + ub_jl]
                    W_jl = zeros(obj.nJL);
                    
                    %Andy TODO: might need to edit this as well
                    [r_jl,M_jl] = jointLimitConstraints(obj.plant,zeros(nq,1));
                    jlcompl_constraints{i} = LinearComplementarityConstraint(W_jl,r_jl,M_jl,obj.options.lincc_mode,obj.options.jlcompl_slack);
                    
                    %jlcompl_constraints{i}.grad_method = 'numerical';
                    %jlcompl_constraints{i}.grad_level = 0;
                    
                    obj = obj.addConstraint(jlcompl_constraints{i},[obj.x_inds(1:nq,i+1);obj.ljl_inds(:,i)]);
                end
            end
            
            % nonlinear complementarity constraints:
            %   lambda_N /perp phi(q)
            %   lambda_fi /perp gamma + Di*psi(q,v)
            % x = [q;v;gamma]
            % z = [lambda_N;lambda_F1;lambda_f2] (each contact sequentially)
            %ANDY TODO: will need to edit this as well
            function [f,df] = nonlincompl_fun(y)
                nq = obj.plant.getNumPositions;
                nv = obj.plant.getNumVelocities;
                x = y(1:nq+nv+obj.nC);
                z = y(nq+nv+obj.nC+1:end);
                gamma = x(nq+nv+1:end);
                
                q = x(1:nq);
                v = x(nq+1:nq+nv);
                
                [phi,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = obj.plant.contactConstraints(q,false,obj.options.active_collision_options);
                
                f = zeros(obj.nC*(1+obj.nD),1);
                df = zeros(obj.nC*(1+obj.nD),nq+nv+obj.nC*(2+obj.nD));
                
                f(1:1+obj.nD:end) = phi;
                df(1:1+obj.nD:end,1:nq) = n;
                for j=1:obj.nD,
                    f(1+j:1+obj.nD:end) = gamma+D{j}*v;
                    df(1+j:1+obj.nD:end,nq+nv+(1:obj.nC)) = eye(size(D{j},1));  %d/dgamma
                    df(1+j:1+obj.nD:end,nq+(1:nv)) = D{j};%d/dv
                    df(1+j:1+obj.nD:end,1:nq) = matGradMult(dD{j},v);%d/dq
                end
                
            end
        end
        
        function [f] = dynamics_constraint_fun(obj,h,x0,x1,u,lambda,lambda_jl, params)
            
            pObj = obj.plant.setParams(params);
            
            nq = obj.plant.getNumPositions;
            nv = obj.plant.getNumVelocities;
            nu = obj.plant.getNumInputs;
            nl = length(lambda);
            njl = length(lambda_jl);
            
            lambda = lambda*obj.options.lambda_mult;
            lambda_jl = lambda_jl*obj.options.lambda_jl_mult;
            
            assert(nq == nv) % not quite ready for the alternative
            
            q0 = x0(1:nq);
            v0 = x0(nq+1:nq+nv);
            q1 = x1(1:nq);
            v1 = x1(nq+1:nq+nv);
            
            %TODO: removed all gradients
            switch obj.options.integration_method
                case ContactImplicitTrajectoryOptimizationTest.MIDPOINT
                    [H,C,B,~, ~, ~] = pObj.manipulatorDynamics((q0+q1)/2,(v0+v1)/2);
%                     dH0 = dH/2;
%                     dC0 = dC/2;
%                     dB0 = dB/2;
%                     dH1 = dH/2;
%                     dC1 = dC/2;
%                     dB1 = dB/2;
                case ContactImplicitTrajectoryOptimizationTest.FORWARD_EULER
                    [H,C,B,~, ~, ~] = pObj.manipulatorDynamics(q0,v0);
%                     dH1 = zeros(nq^2,2*nq);
%                     dC1 = zeros(nq,2*nq);
%                     dB1 = zeros(nq*nu,2*nq);
                case ContactImplicitTrajectoryOptimizationTest.BACKWARD_EULER
                    [H,C,B,~, ~, ~] = pObj.manipulatorDynamics(q1,v1);
%                     dH0 = zeros(nq^2,2*nq);
%                     dC0 = zeros(nq,2*nq);
%                     dB0 = zeros(nq*nu,2*nq);
                case ContactImplicitTrajectoryOptimizationTest.MIXED
                    [H,C,B,~,~,~] = pObj.manipulatorDynamics(q0,v0);
%                     dH1 = zeros(nq^2,2*nq);
%                     dC1 = zeros(nq,2*nq);
%                     dB1 = zeros(nq*nu,2*nq);
            end
            
            BuminusC = B*u-C;
            
            %TODO: commented this out
%             if nu>0,
%                 dBuminusC0 = matGradMult(dB0,u) - dC0;
%                 dBuminusC1 = matGradMult(dB1,u) - dC1;
%             else
%                 dBuminusC0 = -dC0;
%                 dBuminusC1 = -dC1;
%             end
            
            %TODO: comment out dfq
            switch obj.options.integration_method
                case ContactImplicitTrajectoryOptimizationTest.MIDPOINT
                    % q1 = q0 + h*v1
                    fq = q1 - q0 - h*(v0 + v1)/2;
                    %dfq = [-(v1+v0)/2, -eye(nq), -h/2*eye(nq), eye(nq), -h/2*eye(nq) zeros(nq,nu+nl+njl)];
                case ContactImplicitTrajectoryOptimizationTest.FORWARD_EULER
                    % q1 = q0 + h*v1
                    fq = q1 - q0 - h*v0;
                    %dfq = [-v0, -eye(nq), -h*eye(nq), eye(nq), zeros(nq,nv) zeros(nq,nu+nl+njl)];
                case ContactImplicitTrajectoryOptimizationTest.BACKWARD_EULER
                    % q1 = q0 + h*v1
                    fq = q1 - q0 - h*v1;
                    %dfq = [-v1, -eye(nq), zeros(nq,nv), eye(nq), -h*eye(nq) zeros(nq,nu+nl+njl)];
                case ContactImplicitTrajectoryOptimizationTest.MIXED
                    fq = q1 - q0 - h*v1;
                    %dfq = [-v1, -eye(nq), zeros(nq,nv), eye(nq), -h*eye(nq) zeros(nq,nu+nl+njl)];
            end
            
            
            % H*v1 = H*v0 + h*(B*u - C) + n^T lambda_N + d^T * lambda_f
            fv = H*(v1 - v0) - h*BuminusC;
            % [h q0 v0 q1 v1 u l ljl]
            
            %TODO: commented out
            %dfv = [-BuminusC, zeros(nv,nq), -H, zeros(nv,nq), H,-h*B, zeros(nv,nl+njl)] + ...
            %    [zeros(nv,1) matGradMult(dH0,v1-v0)-h*dBuminusC0 matGradMult(dH1,v1-v0)-h*dBuminusC1 zeros(nv,nu+nl+njl)];
            
            if nl>0
                [phi,normal,~,~,~,~,~,~,n,D,dn,dD] = pObj.contactConstraints(q1,false,obj.options.active_collision_options);
                % construct J and dJ from n,D,dn, and dD so they relate to the
                % lambda vector
                J = zeros(nl,nq);
                J(1:2+obj.nD:end,:) = n;
                
                %TODO: commented these out
%                 dJ = zeros(nl*nq,nq);
%                 dJ(1:2+obj.nD:end,:) = dn;
                
                for j=1:length(D),
                    J(1+j:2+obj.nD:end,:) = D{j};
                    
                    %TODO: commented this out
%                    dJ(1+j:2+obj.nD:end,:) = dD{j};
                end
                
                fv = fv - J'*lambda;
                
                %TODO: commented this out
%                 dfv(:,2+nq+nv:1+2*nq+nv) = dfv(:,2+nq+nv:1+2*nq+nv) - matGradMult(dJ,lambda,true);
%                 dfv(:,2+2*nq+2*nv+nu:1+2*nq+2*nv+nu+nl) = -J'*obj.options.lambda_mult;
            end
            
            if njl>0
                [~,J_jl] = jointLimitConstraints(pObj,q1);
                
                fv = fv - J_jl'*lambda_jl;
                
                %TOOD: commented this out
                %dfv(:,2+2*nq+2*nv+nu+nl:1+2*nq+2*nv+nu+nl+njl) = -J_jl'*obj.options.lambda_jl_mult;
            end
            
            f = [fq;fv];
            f
            
            %TOOD: ocmmented this out
            %df = [dfq;dfv];
        end
        
        function [xtraj,utraj,ltraj,ljltraj,z,F,info] = solveTraj(obj,t_init,traj_init)
            
            [lb, ub] = obj.plant.getParamLimits();
            lower_bounds = obj.plant.getParams().double;
            upper_bounds = obj.plant.getParams().double;
            bounds_constraint = BoundingBoxConstraint(lb, ub);
            
            constant_constraint = ConstantConstraint(lower_bounds);
            
            obj = obj.addConstraint(bounds_constraint, obj.param_inds);
            
            
            obj = obj.addFinalCost(@(T, x, params)obj.parameter_regularizer(T, x, params));
            
            obj = obj.addFinalCost(@(T, x, params)obj.velocity(T, x, params));
            
%             for i = 1:1:length(obj.costs_to_add)
%                 cost_object = obj.costs_to_add{i};
%                obj.addCost(cost_object{1}, cost_object{2});
%             end
            
            
            [xtraj,utraj,z,F,info] = solveTraj@DirectTrajectoryOptimization(obj,t_init,traj_init);
            t = [0; cumsum(z(obj.h_inds))];
            if obj.nC>0
                ltraj = PPTrajectory(foh(t,reshape(z(obj.l_inds),[],obj.N)));
            else
                ltraj = [];
            end
            if obj.nJL>0
                ljltraj = PPTrajectory(foh(t,reshape(z(obj.ljl_inds),[],obj.N)));
            else
                ljltraj = [];
            end
        end
        
        function obj = setupVariables(obj,N)
            obj = setupVariables@DirectTrajectoryOptimization(obj,N);
            obj.nC = obj.plant.getNumContactPairs;
            [~,normal,d] = obj.plant.contactConstraints(zeros(obj.plant.getNumPositions,1));
            obj.nD = 2*length(d);
            assert(size(normal,2) == obj.nC); % just a double check
            
            nContactForces = obj.nC*(2 + obj.nD);
            
            obj.l_inds = reshape(obj.num_vars + (1:N * nContactForces),nContactForces,N);
            obj = obj.addDecisionVariable(N * nContactForces);
            
            obj.lfi_inds = zeros(obj.nD,obj.nC);
            for i=1:obj.nC,
                obj.lfi_inds(:,i) = (2:1+obj.nD)' + (i-1)*(2+obj.nD)*ones(obj.nD,1);
            end
            
            obj.nJL = obj.plant.getNumJointLimitConstraints();
            obj.ljl_inds = reshape(obj.num_vars + (1:N * obj.nJL),obj.nJL,N);
            
            % joint limit constraints
            [jl_lb,jl_ub] = obj.plant.getJointLimits();
            obj.jl_lb_ind = find(jl_lb ~= -inf);
            obj.jl_ub_ind = find(jl_ub ~= inf);
            
            obj = obj.addDecisionVariable(N * obj.nJL);
            
            %Andy code: parameter stuff
            num_vars = obj.num_vars;
            params = obj.plant.getParams();
            param_names = params.getFrame().getCoordinateNames();
            obj = obj.addDecisionVariable(length(param_names), param_names);
            obj.param_inds =  (num_vars+1):1:(num_vars+length(param_names));%setup indices
            
            
        end
        
        % evaluates the initial trajectories at the sampled times and
        % constructs the nominal z0. Overwrite to implement in a different
        % manner
        function z0 = getInitialVars(obj,t_init,traj_init)
            if isscalar(t_init)
                t_init = linspace(0,t_init,obj.N);
            elseif length(t_init) ~= obj.N
                error('The initial sample times must have the same length as property N')
            end
            z0 = zeros(obj.num_vars,1);
            z0(obj.h_inds) = diff(t_init);
            
            if isfield(traj_init,'u')
                z0(obj.u_inds) = traj_init.u.eval(t_init);
            else
                nU = getNumInputs(obj.plant);
                z0(obj.u_inds) = 0.01*randn(nU,obj.N);
            end
            
            if isfield(traj_init,'x')
                z0(obj.x_inds) = traj_init.x.eval(t_init);
            else
                if ~isfield(traj_init,'u')
                    traj_init.u = setOutputFrame(PPTrajectory(foh(t_init,reshape(z0(obj.u_inds),nU,obj.N))),getInputFrame(obj.plant));
                end
                
                % todo: if x0 and xf are equality constrained, then initialize with
                % a straight line from x0 to xf (this was the previous behavior)
                
                %simulate
                sys_ol = cascade(traj_init.u,obj.plant);
                [~,x_sim] = sys_ol.simulate([t_init(1) t_init(end)]);
                z0(obj.x_inds) = x_sim.eval(t_init);
            end
            
            if obj.nC > 0
                if isfield(traj_init,'l')
                    z0(obj.l_inds) = traj_init.l.eval(t_init);
                else
                    z0(obj.l_inds) = 0;
                end
            end
            if obj.nJL > 0
                if isfield(traj_init,'ljl')
                    z0(obj.ljl_inds) = traj_init.ljl.eval(t_init);
                else
                    z0(obj.ljl_inds) = 0;
                end
            end
            
            if obj.nC > 0
                for i=1:obj.N-1,
                    gamma_inds = obj.l_inds(obj.nD+2:obj.nD+2:end,i);
                    lambda_inds = obj.l_inds(repmat((1:1+obj.nD)',obj.nC,1) + kron((0:obj.nC-1)',(2+obj.nD)*ones(obj.nD+1,1)),i);
                    if ~isempty(obj.nonlincompl_slack_inds{i})
                        z0(obj.nonlincompl_slack_inds{i}) = obj.nonlincompl_constraints{i}.slack_fun(z0([obj.x_inds(:,i+1);gamma_inds;lambda_inds]));
                    end
                end
            end
            
            %ANDY adding: set initial vars for parameters
            z0(obj.param_inds) = obj.plant.getParams().double;
            
            
        end
        

        
        %ANDY COde:
        function obj = addFinalCost(obj,final_cost_function)
            % adds a cost to the final state and total time
            % @param final_cost_function a function handle f(T,xf)
            nX = obj.plant.getNumStates();
            nH = obj.N-1;
            nP = obj.plant.getNumParams();
            cost = FunctionHandleObjective(nH+nX+nP,@(h,x, params) obj.final_cost(final_cost_function,h,x, params));
            
            %             cost.grad_method = 'numerical';
            %             cost.grad_level = 0;
            %obj.costs_to_add{length(obj.costs_to_add) + 1} = {cost,{obj.h_inds;obj.x_inds(:,end); obj.param_inds(:)}};
            obj = obj.addCost(cost,{obj.h_inds;obj.x_inds(:,end); obj.param_inds(:)});
        end
        
        function obj = addRunningCost(obj,running_cost_function)
            nX = obj.plant.getNumStates();
            nU = obj.plant.getNumInputs();
            nP = obj.plant.getNumParams();
            
            running_cost = FunctionHandleObjective(1+nX+nU+nP,running_cost_function);
            for i=1:obj.N-1,
                
                %Andy TODO: make sure addCost is added elsewhere
                obj.costs_to_add{length(obj.costs_to_add) + 1} = {running_cost,{obj.h_inds(i);obj.x_inds(:,i);obj.u_inds(:,i);obj.param_inds(:)}};
                
                %obj = obj.addCost(running_cost,{obj.h_inds(i);obj.x_inds(:,i);obj.u_inds(:,i)});
            end
        end
        
    end
    
   methods (Access=protected)
            %ANDY CODE:
        
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