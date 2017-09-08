classdef ContactImplicitTrajectoryOptimizationTest < DirectTrajectoryOptimization
    % phi, lambda
    properties (Constant)
        debug = false;
        generate_dynamics = false;
        generate_complementarity = false;
        zeroth_run = false;
        
    end
    
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
        input_limits
        z_solve
        
        nonlincompl_constraints
        nonlincompl_slack_inds
        
    end
    
    properties (Constant)
        FORWARD_EULER = 1;
        BACKWARD_EULER = 2;
        MIDPOINT = 3;  % DEFAULT
        MIXED = 4;   % matched to TimeSteppingRigidBodyManipulator. Forward on qd, backward on q
    end
    
    properties
        param_inds %the indices of the parameters
        suppl_inds
        regularizer_weight
        penalty
        initial_param_values
        costs_to_add
        function_name
        nlncon_name
        clamped_param_inds
        actuation_weight
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
                options.integration_method = ContactImplicitTrajectoryOptimization.MIDPOINT;
            end
            if ~isfield(options,'actuation_weight')
               options.actuation_weight = 0.0; 
            end
            
            sympref('HeavisideAtOrigin',0);
            
            paramsSym = sym(plant.getParamFrame().getCoordinateNames(), 'real');
            
            global tempPlant;
            
            if ~isempty(paramsSym)
                tempPlant = plant.setParams( paramsSym );
            else
                tempPlant = plant;
                tempPlant.mex_model_ptr = 0; %don't use mex
                
            end
            
            obj = obj@DirectTrajectoryOptimization(plant,N,duration,options);
            
            
            obj.clamped_param_inds = options.clamped_param_inds;
            obj.initial_param_values = plant.getParams();
            obj.costs_to_add = {};
            
            obj.actuation_weight = options.actuation_weight;
            
            
            
            obj.plant = plant;
            if ~obj.debug;
                obj.plant = tempPlant; %TODO: why did we do this earlier?
                
                
                obj.function_name = strcat(['testfunction',obj.plant.name{1}]);
                obj.nlncon_name = strcat(['nonlinear_compl_combined',obj.plant.name{1},'.m']);
                
                %TODO: set up the right functions here
                if obj.generate_dynamics  || (exist(obj.function_name, 'file') ~= 3)
                    h = sym('h%d', [1, 1], 'real');
                    u = sym('u%d', [plant.getNumInputs(), 1], 'real');
                    x0 = sym('x0%d', [plant.getNumOutputs(), 1], 'real');
                    x1 = sym('x1%d', [plant.getNumOutputs(), 1], 'real');
                    nContactForces = obj.nC*(2 + obj.nD);
                    l = sym('l%d', [nContactForces, 1], 'real');
                    jL = sym('nJL%d', [obj.nJL, 1], 'real');
                    
                    f = obj.dynamics_constraint_fun(h,x0,x1,u,l,jL);
                    fToM(f, plant, nContactForces);
                    file_in = strcat(['dynamics_function',obj.plant.name{1},'.c']);
                    file_out = strcat(['testfunction',obj.plant.name{1},'.c']);
                    file_out_no_c = strcat(['testfunction',obj.plant.name{1}]);
                    python_call = sprintf('python edit_dynamics_stack_overflow.py %s %s', file_in, file_out);
                    system(python_call)
                    compile_call = strcat(['sh compile_mex.sh ', file_out_no_c]);
                    system(compile_call);
                    
%                 elseif ~obj.generate_dynamics
%                      h = sym('h%d', [1, 1], 'real');
%                     u = sym('u%d', [plant.getNumInputs(), 1], 'real');
%                     x0 = sym('x0%d', [plant.getNumOutputs(), 1], 'real');
%                     x1 = sym('x1%d', [plant.getNumOutputs(), 1], 'real');
%                     nContactForces = obj.nC*(2 + obj.nD);
%                     l = sym('l%d', [nContactForces, 1], 'real');
%                     jL = sym('nJL%d', [obj.nJL, 1], 'real');
%                     
%                     f = obj.dynamics_constraint_fun(h,x0,x1,u,l,jL);
%                     
%                     
                end
                if obj.generate_complementarity || (exist(obj.nlncon_name, 'file') ~= 2)
                    nX = obj.plant.getNumStates();
                    xdim = nX + obj.nC;
                    zdim = obj.nC*(1+obj.nD);
                    
                    y1 = sym('y1', [xdim, 1], 'real');
                    y2 = sym('y2', [zdim, 1], 'real');
                    
                    y = [y1; y2; paramsSym];
                    [f, df] = obj.nonlincompl_fun(y);
                    f = simplify(f);
                    df = simplify(df);
                    
                    matlabFunction(df, 'vars', {y}, 'file', strcat('nonlinear_compl_jacobian', obj.plant.name{1}), 'Optimize',false);
                    matlabFunction(f, 'vars', {y}, 'file', strcat('nonlinear_compl', obj.plant.name{1}), 'Optimize',false);
                    
                    fid = fopen(obj.nlncon_name);
                    fprintf(fid, 'function [f, df] = nonlinear_compl_combined%s(y)\n', obj.plant.name{1});
                    fprintf(fid, 'f = nonlinear_compl%s(y);\n', obj.plant.name{1});
                    fprintf(fid, 'f = nonlinear_compl%s(y);\n', obj.plant.name{1});
                    fprintf(fid, 'end');
                    
                    
                    fclose(fid);
                    
                end
                
                
            end
            
            %f2 = matlabFunction(df, 'vars', {h, x, u, l, jL, paramsSym}, 'file', strcat('dynamics_jacobian', obj.plant.name{1}), 'Optimize',false);
            %disp('Done writing files
            obj.penalty = 2; %TODO: move this elsewhere
            obj = obj.setSolverOptions('snopt','print',sprintf('debug_%s.out', obj.plant.name{1}));
            %obj.constraint_err_tol = 1e-6;
            obj.z_solve = [];
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
            
            
            
            if obj.debug
                n_vars = 2*nX + nU + 1 + obj.nC*(2+obj.nD) + obj.nJL;
                cnstr = FunctionHandleConstraint(zeros(nX,1),zeros(nX,1),n_vars,@obj.dynamics_constraint_fun);
                %TODO: bad
                
                cnstr.grad_method = 'numerical';
                cnstr.grad_level = 0;
            else
                n_vars = 2*nX + nU + 1 + obj.nC*(2+obj.nD) + obj.nJL + nP;
                cnstr = FunctionHandleConstraint(zeros(nX,1),zeros(nX,1),n_vars,@obj.dynamics_constraint_fun_params);
                cnstr.grad_method = 'numerical';
                cnstr.grad_level = 0;
                
            end
            
            %ANDYTODO: not a big fan of using globals here
            global tempPlant;
            [~,~,~,~,~,~,~,mu] = tempPlant.contactConstraints(zeros(nq,1),false,obj.options.active_collision_options);
            
            for i=1:obj.N-1,
                %         dyn_inds{i} = [obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.l_inds(:,i);obj.ljl_inds(:,i)];
                if obj.debug
                    dyn_inds{i} = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.l_inds(:,i);obj.ljl_inds(:,i)};
                else
                    dyn_inds{i} = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.l_inds(:,i);obj.ljl_inds(:,i); obj.param_inds(:)};
                end
                constraints{i} = cnstr;
                
                obj = obj.addConstraint(constraints{i}, dyn_inds{i});
                
                if obj.nC > 0
                    % indices for (i) gamma
                    gamma_inds = obj.l_inds(obj.nD+2:obj.nD+2:end,i);
                    % awkward way to pull out these indices, for (i) lambda_N and
                    % lambda_f
                    lambda_inds = obj.l_inds(repmat((1:1+obj.nD)',obj.nC,1) + kron((0:obj.nC-1)',(2+obj.nD)*ones(obj.nD+1,1)),i);
                    
                    
                    if obj.debug
                        obj.nonlincompl_constraints{i} = NonlinearComplementarityConstraint(@obj.nonlincompl_fun,nX + obj.nC,obj.nC*(1+obj.nD),obj.options.nlcc_mode,obj.options.compl_slack);
                        obj.nonlincompl_slack_inds{i} = obj.num_vars+1:obj.num_vars + obj.nonlincompl_constraints{i}.n_slack;
                        obj = obj.addConstraint(obj.nonlincompl_constraints{i},[obj.x_inds(:,i+1);gamma_inds;lambda_inds]);
                    else
                        
                        %DEBUG PURPOSES:
                        test_cnstr = NonlinearComplementarityConstraint(@obj.nonlincompl_fun,nX + obj.nC,obj.nC*(1+obj.nD),obj.options.nlcc_mode,obj.options.compl_slack);
                        
                        nonlin_constraint_handle = str2func(strcat('nonlinear_compl_combined', obj.plant.name{1}));
                        obj.nonlincompl_constraints{i} = NonlinearComplementarityConstraintTest(nonlin_constraint_handle,nX + obj.nC,obj.nC*(1+obj.nD),nP, obj.options.compl_slack, test_cnstr);
                        %obj.nonlincompl_constraints{i} = NonlinearComplementarityConstraintTest2(@obj.nonlincompl_fun,nX + obj.nC,obj.nC*(1+obj.nD),nP, obj.options.nlcc_mode,obj.options.compl_slack);
                        obj.nonlincompl_slack_inds{i} = obj.num_vars+1:obj.num_vars + obj.nonlincompl_constraints{i}.n_slack;
                        obj = obj.addConstraint(obj.nonlincompl_constraints{i},[obj.x_inds(:,i+1);gamma_inds;lambda_inds; obj.param_inds']);
                        %                         cnstr1 = obj.nonlincompl_constraints{i}.constraints{1};
                        %                           cnstr2 = obj.nonlincompl_constraints{i}.constraints{2};
                        %                           cnstr3 = obj.nonlincompl_constraints{i}.constraints{3};
                        %                           a = testfun(@cnstr1.eval, 'compl_vec');
                        %                           b = testfun(@cnstr2.eval, 'compl_vec');
                        %                           c = testfun(@cnstr3.eval, 'compl_vec');
                        %                           save('complex_result', 'a', 'b', 'c');
                    end
                    
                    %Now let's make sure we don't go airborne
                    
                    
                   n_vars = size(obj.l_inds, 1);
                   non_airborne_cnstr = FunctionHandleConstraint(1.0,inf,n_vars,@nonAirborneConstraint);
                   obj = obj.addConstraint(non_airborne_cnstr, obj.l_inds(:, i));
                   
                   small_slip_cnstr = FunctionHandleConstraint(zeros(n_vars / 6, 1), 1.0*ones(n_vars / 6, 1),n_vars,@smallSlipConstraint);
                   obj = obj.addConstraint(small_slip_cnstr, obj.l_inds(:, i));
                    
                   
                    
                    
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
                    
                    if obj.debug
                        lincompl_constraints{i} = LinearComplementarityConstraint(W,r,M,obj.options.lincc_mode,obj.options.lincompl_slack);
                    else
                        lincompl_constraints{i} = LinearComplementarityConstraintTest(obj.nC,obj.nD,obj.options.lincc_mode,obj.options.lincompl_slack);
                    end
                    
                    obj = obj.addConstraint(lincompl_constraints{i},[lambda_inds;gamma_inds]);
                end
                
                if obj.nJL > 0
                    %ANDYTODO: we haven't touched this yet, so don't worry
                    %about it
                    % joint limit linear complementarity constraint
                    % lambda_jl /perp [q - lb_jl; -q + ub_jl]
                    W_jl = zeros(obj.nJL);
                    [r_jl,M_jl] = jointLimitConstraints(obj.plant,zeros(nq,1));
                    jlcompl_constraints{i} = LinearComplementarityConstraint(W_jl,r_jl,M_jl,obj.options.lincc_mode,obj.options.jlcompl_slack);
                    
                    obj = obj.addConstraint(jlcompl_constraints{i},[obj.x_inds(1:nq,i+1);obj.ljl_inds(:,i)]);
                end
            end
            
            % nonlinear complementarity constraints:
            %   lambda_N /perp phi(q)
            %   lambda_fi /perp gamma + Di*psi(q,v)
            % x = [q;v;gamma]
            % z = [lambda_N;lambda_F1;lambda_f2] (each contact sequentially)
            
        end
        
        %ANDYTODO: this was taken out of being a private method function to
        %being a class method function
        %Make sure that it's matlab function'd appropriately so this
        %doesn't need to be turned back to a function for optimization
        %purposes
        function [f,df] = nonlincompl_fun(obj, y)
            nq = obj.plant.getNumPositions;
            nv = obj.plant.getNumVelocities;
            np = obj.plant.getNumParams;
            x = y(1:nq+nv+obj.nC);
            z = y(nq+nv+obj.nC+1:end-1);
            gamma = x(nq+nv+1:end);
            
            q = x(1:nq);
            v = x(nq+1:nq+nv);
            
            [phi,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = obj.plant.contactConstraints(q,false,obj.options.active_collision_options);
            
            %ANDYCODE
%             y_sym = q(2);
%             %step = 0.5*(heaviside(y_sym - 0.5) + heaviside(y_sym - 1.5) + heaviside(y_sym - 2.5));
%             step = 1.0*(heaviside(y_sym - 1.5) + (y_sym - 0.5) * (heaviside(y_sym - 0.5) - heaviside(y_sym - 1.5)));
%             phi = (phi-step)';
            
            
            f = zeros(obj.nC*(1+obj.nD),1);
            df = zeros(obj.nC*(1+obj.nD),nq+nv+obj.nC*(2+obj.nD));
            if isa(D{1}, 'sym') || isa(v, 'sym') || isa(gamma, 'sym')
                f = sym(f);
                df = sym(df);
            end
            
            
            
            
            f(1:1+obj.nD:end) = phi;
            df(1:1+obj.nD:end,1:nq) = n;
            for j=1:obj.nD,
                f(1+j:1+obj.nD:end) = gamma+D{j}*v;
                df(1+j:1+obj.nD:end,nq+nv+(1:obj.nC)) = eye(size(D{j},1));  %d/dgamma
                df(1+j:1+obj.nD:end,nq+(1:nv)) = D{j};%d/dv
                df(1+j:1+obj.nD:end,1:nq) = matGradMult(dD{j},v);%d/dq
            end
            
            if isa(f, 'sym')
                dfparams = jacobian(f, y(end - np + 1:end));
                df = [df, dfparams];
            end
            
        end
        
        function [f, df] = dynamics_constraint_fun_params(obj, h, x0, x1, u, lambda, lambda_jl, params)
            f = feval(strcat(['testfunction',obj.plant.name{1}]), h, x0, x1, u, lambda, lambda_jl, params);
            %f_test = obj.dynamics_constraint_fun(h, x0, x1, u, lambda, lambda_jl);
        end
        
        function [f] = dynamics_constraint_fun(obj,h,x0,x1,u,lambda,lambda_jl)
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
            
            switch obj.options.integration_method
                case ContactImplicitTrajectoryOptimization.MIDPOINT
                    %ANDYCODE CHNAGE
                    if obj.debug
                        [H,C,B,dH,dC,dB] = obj.plant.manipulatorDynamics((q0+q1)/2,(v0+v1)/2);
                        dH0 = dH/2;
                        dC0 = dC/2;
                        dB0 = dB/2;
                        dH1 = dH/2;
                        dC1 = dC/2;
                        dB1 = dB/2;
                    else
                        [H,C,B] = obj.plant.manipulatorDynamics((q0+q1)/2,(v0+v1)/2);
                    end
                    
                case ContactImplicitTrajectoryOptimization.FORWARD_EULER
                    [H,C,B,dH0,dC0,dB0] = obj.plant.manipulatorDynamics(q0,v0);
                    dH1 = zeros(nq^2,2*nq);
                    dC1 = zeros(nq,2*nq);
                    dB1 = zeros(nq*nu,2*nq);
                case ContactImplicitTrajectoryOptimization.BACKWARD_EULER
                    [H,C,B,dH1,dC1,dB1] = obj.plant.manipulatorDynamics(q1,v1);
                    dH0 = zeros(nq^2,2*nq);
                    dC0 = zeros(nq,2*nq);
                    dB0 = zeros(nq*nu,2*nq);
                case ContactImplicitTrajectoryOptimization.MIXED
                    [H,C,B,dH0,dC0,dB0] = obj.plant.manipulatorDynamics(q0,v0);
                    dH1 = zeros(nq^2,2*nq);
                    dC1 = zeros(nq,2*nq);
                    dB1 = zeros(nq*nu,2*nq);
            end
            
            BuminusC = B*u-C;
            
            %ANDYCODE Change
            if obj.debug
                if nu>0,
                    dBuminusC0 = matGradMult(dB0,u) - dC0;
                    dBuminusC1 = matGradMult(dB1,u) - dC1;
                else
                    dBuminusC0 = -dC0;
                    dBuminusC1 = -dC1;
                end
            end
            
            switch obj.options.integration_method
                case ContactImplicitTrajectoryOptimization.MIDPOINT
                    % q1 = q0 + h*v1
                    fq = q1 - q0 - h*(v0 + v1)/2;
                    
                    %ANDYCODE CHANGE
                    if obj.debug
                        dfq = [-(v1+v0)/2, -eye(nq), -h/2*eye(nq), eye(nq), -h/2*eye(nq) zeros(nq,nu+nl+njl)];
                    end
                case ContactImplicitTrajectoryOptimization.FORWARD_EULER
                    % q1 = q0 + h*v1
                    fq = q1 - q0 - h*v0;
                    if obj.debug
                        dfq = [-v0, -eye(nq), -h*eye(nq), eye(nq), zeros(nq,nv) zeros(nq,nu+nl+njl)];
                    end
                case ContactImplicitTrajectoryOptimization.BACKWARD_EULER
                    % q1 = q0 + h*v1
                    fq = q1 - q0 - h*v1;
                    if obj.debug
                        dfq = [-v1, -eye(nq), zeros(nq,nv), eye(nq), -h*eye(nq) zeros(nq,nu+nl+njl)];
                    end
                case ContactImplicitTrajectoryOptimization.MIXED
                    fq = q1 - q0 - h*v1;
                    if obj.debug
                        dfq = [-v1, -eye(nq), zeros(nq,nv), eye(nq), -h*eye(nq) zeros(nq,nu+nl+njl)];
                    end
            end
            
            
            % H*v1 = H*v0 + h*(B*u - C) + n^T lambda_N + d^T * lambda_f
            fv = H*(v1 - v0) - h*BuminusC;
            % [h q0 v0 q1 v1 u l ljl]
            
            %ANDYCODE CHANGE
            if obj.debug
                dfv = [-BuminusC, zeros(nv,nq), -H, zeros(nv,nq), H,-h*B, zeros(nv,nl+njl)] + ...
                    [zeros(nv,1) matGradMult(dH0,v1-v0)-h*dBuminusC0 matGradMult(dH1,v1-v0)-h*dBuminusC1 zeros(nv,nu+nl+njl)];
            end
            
            if nl>0
                [phi,normal,~,~,~,~,~,~,n,D,dn,dD] = obj.plant.contactConstraints(q1,false,obj.options.active_collision_options);
                % construct J and dJ from n,D,dn, and dD so they relate to the
                % lambda vector
                
                
                J = zeros(nl,nq);
                
                if isa(n, 'sym') %TODO: is this a sufficient check?
                    J = sym(J);
                end
                
                J(1:2+obj.nD:end,:) = n;
                
                %ANDYCODE commented out
                if obj.debug
                    dJ = zeros(nl*nq,nq);
                    dJ(1:2+obj.nD:end,:) = dn;
                end
                
                for j=1:length(D),
                    J(1+j:2+obj.nD:end,:) = D{j};
                    
                    %ANDYCODE commented out
                    if obj.debug
                        dJ(1+j:2+obj.nD:end,:) = dD{j};
                    end
                end
                
                fv = fv - J'*lambda;
                
                %ANDYCODE commented out
                if obj.debug
                    dfv(:,2+nq+nv:1+2*nq+nv) = dfv(:,2+nq+nv:1+2*nq+nv) - matGradMult(dJ,lambda,true);
                    dfv(:,2+2*nq+2*nv+nu:1+2*nq+2*nv+nu+nl) = -J'*obj.options.lambda_mult;
                end
            end
            
            if njl>0
                d%ANDYTODO: this might have to change later
                [~,J_jl] = jointLimitConstraints(obj.plant,q1);
                
                fv = fv - J_jl'*lambda_jl;
                
                %ANDYCODE commented out
                if obj.debug
                    dfv(:,2+2*nq+2*nv+nu+nl:1+2*nq+2*nv+nu+nl+njl) = -J_jl'*obj.options.lambda_jl_mult;
                end
            end
            
            f = [fq;fv];
            if obj.debug
                df = [dfq;dfv];
            end
        end
        
        function [xtraj,utraj,ltraj,ljltraj,z,F,info, subCost, midz] = solveTraj(obj,t_init,traj_init)
            %ANDYCODE: make sure to add new parameter constraining cost
            
            
            if obj.zeroth_run
                %0th run...? try to get it working without parameters?
                param_vals = obj.initial_param_values.double;
                %             %lower_bounds = obj.initial_param_values.double + 0.1;
                %             %upper_bounds = lower_bounds;
                %
                %             %             lower_bounds = obj.plant.getParams().double;
                %             %             upper_bounds = obj.plant.getParams().double;
                if ~obj.debug
                    bounds_constraint = BoundingBoxConstraint(param_vals, param_vals);
                    
                    [obj, cnstr_idx] = obj.addConstraint(bounds_constraint, obj.param_inds);
                end
                
                
                %ANDYTODO: restructure this to mirror other function
                [xtraj,utraj,z,F,info] = solveTraj@DirectTrajectoryOptimization(obj,t_init,traj_init);
                obj = obj.deleteBoundingBoxConstraint(cnstr_idx); %Remove last bounding box constraint
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
                
                traj_init.x = xtraj;
                traj_init.u = utraj;
                traj_init.l = ltraj;
                traj_init.ljl = ljltraj;
                
            end
            
            %Add lower bound constraints
            [lower_bounds, upper_bounds] = obj.plant.getParamLimits();
            init_params = obj.initial_param_values.double;
            lower_bounds(obj.clamped_param_inds) = init_params(obj.clamped_param_inds );
            upper_bounds(obj.clamped_param_inds ) = init_params(obj.clamped_param_inds );
            %             upper_bounds = init_params;
            %             lower_bounds = init_params;
            
            if ~obj.debug
                lower_bounds
                upper_bounds
                bounds_constraint = BoundingBoxConstraint(lower_bounds, upper_bounds);
                
                obj = obj.addConstraint(bounds_constraint, obj.param_inds);
            end
            
            
            
            
            
            %ANDYTODO: restructure this to mirror other function
            [xtraj,utraj,z,F,info] = solveTraj@DirectTrajectoryOptimization(obj,t_init,traj_init);
            obj.z_solve = z;
            disp(z(obj.param_inds))
            midz = z;
            us = z(obj.u_inds);
            subCost = min(us(:));
            
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
            
            %SECOND RUN
            if ~isempty(obj.param_inds)
                obj = obj.addFinalCost(@(T, x, params)obj.parameter_regularizer(T, x, params));
            end
            
            obj = obj.addActuation();
            
            for i = 1:1:length(obj.costs_to_add)
                cost_object = obj.costs_to_add{i};
                obj = obj.addCost(cost_object{1}, cost_object{2});
                
                
            end
            
            if ~isempty(obj.param_inds(:))
                obj.plant = obj.plant.setParams(z(obj.param_inds(:)));
            end
            traj_init.x = xtraj;
            traj_init.u = utraj;
            traj_init.l = ltraj;
            traj_init.ljl = ljltraj;
            t_init = utraj.getBreaks();
            
            [xtraj,utraj,z,F,info] = solveTraj@DirectTrajectoryOptimization(obj,t_init,traj_init);
            
            
            
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
            
            
            %ANDY CODE
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
        
        % evaluates the initial trajectories at the sampled times and
        % constructs the nominal z0. Overwrite to implement in a different
        % manner
        function z0 = getInitialVars(obj,t_init,traj_init)
            if ~isempty(obj.z_solve)
                z0 = obj.z_solve;
                return;
                
            end
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
                        if obj.debug
                            z0(obj.nonlincompl_slack_inds{i}) = obj.nonlincompl_constraints{i}.slack_fun(z0([obj.x_inds(:,i+1);gamma_inds;lambda_inds]));
                        else
                            z0(obj.nonlincompl_slack_inds{i}) = obj.nonlincompl_constraints{i}.slack_fun(z0([obj.x_inds(:,i+1);gamma_inds;lambda_inds;(obj.param_inds')]));
                        end
                    end
                end
            end
            
            
            %ANDYCODE
            %z0(obj.param_inds) = obj.plant.getParams().double;
            z0(obj.param_inds) = obj.initial_param_values.double;
            
            %Andy new code
            %TODO NEW: eventually want to get this working for multiple
            %limits
            z0(obj.suppl_inds) = max(abs([-100; 100])); %TODO: fix this later to the real value
            
%                 load('starting_traj/z_test');
%                 z0 = z;
            
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
            %TODO: add the supplementary variable to the other function
            %stuff
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
                    w = obj.regularizer_weight;
                    f = w*norm(double(param_values) - params, 2)^2;
                    df = [0, zeros(length(x), 1)', 2*w*(params - double(param_values))'];
                    
            end
        end
        
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
            nX = obj.plant.getNumStates();
            nU = obj.plant.getNumInputs();
            nP = obj.plant.getNumParams();
            running_cost = FunctionHandleObjective(1+nX+nU+nP,running_cost_function);
            for i=1:obj.N-1,
                %obj = obj.addCost(running_cost,{obj.h_inds(i);obj.x_inds(:,i);obj.u_inds(:,i); obj.param_inds(:,i)});
                obj.costs_to_add{length(obj.costs_to_add) + 1} = {running_cost,{obj.h_inds(i);obj.x_inds(:,i);obj.u_inds(:,i); obj.param_inds(:)}};
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
            
            %             cost.grad_method = 'numerical';
            %             cost.grad_level = 0;
            obj.costs_to_add{length(obj.costs_to_add) + 1} = {cost,{obj.h_inds;obj.x_inds(:,end); obj.param_inds(:)}};
            %obj = obj.addCost(cost,{obj.h_inds;obj.x_inds(:,end); obj.param_inds(:)});
        end
        
    end
    
    
    %ANDYTODO: what about midpoint running fun?
    
    methods(Access=protected)
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