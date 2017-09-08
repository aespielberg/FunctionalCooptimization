classdef ParametricNonlinearComplementarityConstraint < CompositeConstraint
  % NonlinearComplementarityConstraint
  % A constraint of the form z >= 0, f(x,z) >= 0, <z,f(x,z)> = 0
  %
  % Constraints are applied to the stacked vector [x;z;gamma]
  %   wherever there are slack variables gamma
  %
  % mode 1: (default)
  %         z >= 0 (bb),
  %         f(x,z) >= 0 (nl),
  %         <z,f(x,z)> = 0 (nl) (elementwise)
  %
  % mode 2: (slack variable for nonlinear function)
  %         z >= 0 (bb)
  %         gamma >= 0 (bb, slack var)
  %         f(x,z) - gamma = 0 (nl)
  %         <z,gamma> = 0 (nl)  (elementwise)
  %
  % mode 3: (Fischer-Burmeister)
  %         z + f(x,z) - sqrt(z^2 + f(x,z)^2) (elementwise)
  % mode 4: (prox)
  %         z - max(0,z - r*f(x,z)) for some r
  properties
    slack_fun = [] % function to solve for slack variables,  where slack = slack_fun(x,z)
  end
  
  methods
    

    function obj = ParametricNonlinearComplementarityConstraint(plant, xdim,zdim, paramdim, slack, active_collision_options, verify_cnstr)
      
      



      constraints{1} = BoundingBoxConstraint([-inf(xdim,1);zeros(2*zdim,1); -inf(paramdim, 1)],inf(2*zdim+xdim + paramdim,1));
      constraints{2} = FunctionHandleConstraint(zeros(zdim,1),zeros(zdim,1),xdim+2*zdim + paramdim,@slackeq);

      constraints{2}.grad_method = 'numerical';
      constraints{2}.grad_level = 0;


      constraints{3} = FunctionHandleConstraint(zeros(zdim,1),zeros(zdim,1)+slack,xdim+2*zdim + paramdim,@slackprod);

      constraints{3}.grad_method = 'numerical';
      constraints{3}.grad_level = 0;


            function [f] = nonlinfun_orig(y)
                nC = plant.getNumContactPairs;
                [~,~,d] = plant.contactConstraints(zeros(plant.getNumPositions,1));
                nD = 2*length(d);
          


                
                %TODO: is this the original?
                nq = plant.getNumPositions;
                nv = plant.getNumVelocities;
                
                
                x = y(1:nq+nv+nC);
                z = y(nq+nv+nC+1:end);
                gamma = x(nq+nv+1:end);
                
                q = x(1:nq);
                v = x(nq+1:nq+nv);
                
                [phi,normal,~,xA,xB,idxA,idxB,mu,n,D,dn,dD] = plant.contactConstraints(q,false,active_collision_options);
                
                f = zeros(nC*(1+nD),1);
                %df = zeros(nC*(1+nD),nq+nv+nC*(2+nD));
                
                f(1:1+nD:end) = phi;
%                df(1:1+nD:end,1:nq) = n;
                for j=1:nD,
                    f(1+j:1+nD:end) = gamma+D{j}*v;
%                     df(1+j:1+nD:end,nq+nv+(1:nC)) = eye(size(D{j},1));  %d/dgamma
%                     df(1+j:1+nD:end,nq+(1:nv)) = D{j};%d/dv
%                     df(1+j:1+nD:end,1:nq) = matGradMult(dD{j},v);%d/dq
                end
                
            end      
      

      function [f] = nonlinfun(y)
                nC = plant.getNumContactPairs;
                [~,~,d] = plant.contactConstraints(zeros(plant.getNumPositions,1));
                nD = 2*length(d);
          

                params =  y(end - paramdim + 1 : end);
                
                plantCopy = plant.setParams(params);
                
                
                nq = plantCopy.getNumPositions;
                nv = plantCopy.getNumVelocities;
                
                x = y(1:nq+nv+nC);
                z = y(nq+nv+nC+1:end);
                gamma = x(nq+nv+1:end);
                
                q = x(1:nq);
                v = x(nq+1:nq+nv);
                
                [phi,normal,~,xA,xB,idxA,idxB,mu,n,D,dn,dD] = plantCopy.contactConstraints(q,false,active_collision_options);
                
                %TODO: check that this is okay to do
%                 thresh = 1e-6;
%                 phi(abs(phi)<thresh)=0;
                
                
                f = zeros(nC*(1+nD),1);
%                df = zeros(nC*(1+nD),nq+nv+nC*(2+nD));
                
                f(1:1+nD:end) = phi;
%                df(1:1+nD:end,1:nq) = n;
                for j=1:nD,
                    f(1+j:1+nD:end) = gamma+D{j}*v;
%                     df(1+j:1+nD:end,nq+nv+(1:nC)) = eye(size(D{j},1));  %d/dgamma
%                     df(1+j:1+nD:end,nq+(1:nv)) = D{j};%d/dv
%                     df(1+j:1+nD:end,1:nq) = matGradMult(dD{j},v);%d/dq
                end
                
            end
      

      
      function [f,df] = slackeq(y)
        x = y(1:xdim);
        z = y(xdim+1:xdim+zdim);
        params = y(xdim+zdim+1:xdim+zdim+paramdim);
        gamma = y(xdim+zdim+paramdim + 1: end);
        
        
        [f] = nonlinfun([x;z;params]);
        f = f - gamma;
        
        f_ = verify_cnstr.constraints{2}.eval([x; z; gamma]);
                

%TODO: is it possible to refactor this like the other one so I can just
%pass in params?
%         f
%         f_
%         x
%         z
%            disp('break') 
%         end
        
        
        %df = [df zeros(zdim)] - [zeros(zdim,zdim+xdim) eye(zdim)];
      end
      
      function [f] = slackprod(y)
        x = y(1:xdim);
        z = y(xdim+1:xdim+zdim);
        gamma = y(xdim+zdim+paramdim+1: end);
        
        f = z.*gamma;
        
        f_ = verify_cnstr.constraints{3}.eval([x; z; gamma]);
                

%         f
%         f_
%         x
%         z
%         if norm(f - f_, 2) > 0
%            disp('break') 
%         end
        
        
        
        %df = [zeros(zdim,xdim) diag(gamma) diag(z)];
      end
      

      

      
      obj = obj@CompositeConstraint(constraints, zdim);
      


      obj.slack_fun = @nonlinfun_orig;

      
    end
  end
end