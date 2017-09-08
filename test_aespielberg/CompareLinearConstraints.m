% %%%Just a test script to see if the parametric linear constraint stuff is
% %%%working right
% warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
% warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
% warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
% options.terrain = RigidBodyFlatTerrain(); %add basic terrain
% options.floating = true; %our robot isn't tethered anywhere in the world
% options.ignore_self_collisions = true; %We will enforce this through other constraints
% p = RigidBodyManipulator('Ant.urdf',options);
% 
% c1 = ParametricLinearComplementarityConstraint(obj.plant,obj.options.lincc_mode,obj.options.lincompl_slack);
% 
% %TODO: is this really worth it...?
% %%GET ALL THE OTHER STUFF
% nX = obj.plant.getNumStates();
%       nU = obj.plant.getNumInputs();
%       nq = obj.plant.getNumPositions();
% 
%       
%       [~,~,~,~,~,~,~,mu] = obj.plant.contactConstraints(zeros(nq,1),false,obj.options.active_collision_options);
%       
%       for i=1:obj.N-1,
% %         dyn_inds{i} = [obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.l_inds(:,i);obj.ljl_inds(:,i)];
%         dyn_inds{i} = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.l_inds(:,i);obj.ljl_inds(:,i)};
%         constraints{i} = cnstr;
%         
%         obj = obj.addConstraint(constraints{i}, dyn_inds{i});
%         
%         if obj.nC > 0
%           % indices for (i) gamma
%           gamma_inds = obj.l_inds(obj.nD+2:obj.nD+2:end,i);
%           % awkward way to pull out these indices, for (i) lambda_N and
%           % lambda_f
%           lambda_inds = obj.l_inds(repmat((1:1+obj.nD)',obj.nC,1) + kron((0:obj.nC-1)',(2+obj.nD)*ones(obj.nD+1,1)),i);
%           
%           
%           obj.nonlincompl_constraints{i} = NonlinearComplementarityConstraint(@nonlincompl_fun,nX + obj.nC,obj.nC*(1+obj.nD),obj.options.nlcc_mode,obj.options.compl_slack);
%           obj.nonlincompl_slack_inds{i} = obj.num_vars+1:obj.num_vars + obj.nonlincompl_constraints{i}.n_slack;          
%           obj = obj.addConstraint(obj.nonlincompl_constraints{i},[obj.x_inds(:,i+1);gamma_inds;lambda_inds]);
%           
%           % linear complementarity constraint
%           %   gamma /perp mu*lambda_N - sum(lambda_fi)
%           %
%           %  Generate terms W,r,M,gamma_inds so that
%           %  gamma = y(gamma_inds)
%           %  Wz+Mx+r = mu*lambda_N - sum(lambda_fi)
%           r = zeros(obj.nC,1);
%           W = zeros(obj.nC,obj.nC);
%           M = zeros(obj.nC,obj.nC*(1+obj.nD));
%           for k=1:obj.nC,
%             M(k,1 + (k-1)*(1+obj.nD)) = mu(k);
%             M(k,(2:obj.nD+1) + (k-1)*(1+obj.nD)) = -ones(obj.nD,1);
%           end
% 
% 
% 
% c2 = 

function CompareLinearConstraints(cnstr1, cnstr2)

for i = 1:1:100
rdm = rand(36, 1);
val1 = cnstr1.constraints{2}.eval(rdm);
val2 = cnstr2.constraints{2}.eval([rdm; [1.0; 1.0; 1.8; 0.2; 0.2; 0.1; 0.1; 0.3; 0.0; 0.0; 0.0]]);
norm(val1 - val2, 2)
disp('and')
val1 = cnstr1.constraints{3}.eval(rdm);
val2 = cnstr2.constraints{3}.eval([rdm; [1.0; 1.0; 1.8; 0.2; 0.2; 0.1; 0.1; 0.3; 0.0; 0.0; 0.0]]);
norm(val1 - val2, 2)
end
