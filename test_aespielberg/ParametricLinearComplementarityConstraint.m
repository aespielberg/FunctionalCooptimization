classdef ParametricLinearComplementarityConstraint < CompositeConstraint
    % LinearComplementarityConstraint
    % A constraint of the form z >= 0, Wz + Mx + q >= 0, <z,Wz + q> = 0
    %  for given W,q
    % Constraints are applied to the stacked vector [x;z;gamma]
    %   wherever there are slack variables gamma
    %
    % mode 1: (default)
    %         z >= 0 (bb),
    %         W*z + M*x + q >= 0 (lin),
    %         <z,W*z+M*x+q)> = 0 (nl) (elementwise)
    %
    % mode 3: (Fischer-Burmeister)
    %         z + W*z+M*x+q - sqrt(z^2 + (W*z+M*x+q)^2) (nl) (elementwise)
    methods
        
        function obj = ParametricLinearComplementarityConstraint(plant,mode,slack, verify_cnstr)
            %from the plant we can create the M, W, q
            nC = plant.getNumContactPairs;
            [~,~,d] = plant.contactConstraints(zeros(plant.getNumPositions,1));
            nD = 2*length(d);
            
            
            nP = plant.getNumParams();
            
            
            zdim = nC;
            xdim = nC*(1+nD);
            n = 0;
            
            bcon = BoundingBoxConstraint([-inf(xdim,1);zeros(zdim,1); -inf(nP, 1)],inf(zdim+xdim + nP,1));
            
            %TOOD: change this
            lincon = FunctionHandleConstraint(zeros(nC,1),inf(zdim,1), xdim+zdim+nP, @(y)linfun(y, plant));
            lincon.grad_method = 'numerical';
            lincon.grad_level = 0;
            
            
            nlcon = FunctionHandleConstraint(zeros(zdim,1),zeros(zdim,1)+slack,xdim+zdim+nP,@(y)prodfun(y, plant));
            nlcon.grad_method = 'numerical';
            nlcon.grad_level = 0;
            
            
            constraints = {bcon;lincon;nlcon};
            
            function [f] = linfun(y, plantCopy)
                
                x = y(1:xdim);
                z = y(xdim+1:xdim+zdim);
                params = y(zdim+xdim+1:end);
                
                plantCopy = plantCopy.setParams(params);
                
                options.active_collision_options.terrain_only = true;
                nq = plantCopy.getNumPositions();
                [~,~,~,~,~,~,~,mu] = plantCopy.contactConstraints(zeros(nq,1),false,options.active_collision_options);
                
                q = zeros(nC,1);
                W = zeros(nC, nC);
                M = zeros(nC,nC*(1+nD));
                for k=1:nC,
                    M(k,1 + (k-1)*(1+nD)) = mu(k);
                    M(k,(2:nD+1) + (k-1)*(1+nD)) = -ones(nD,1);
                end
                
                assert(zdim == size(W,1));
                assert(zdim == size(M,1));
                
                
                f = [M W] * [x; z];
                
                f_ = verify_cnstr.constraints{2}.eval([x; z]);
                
                if (norm(f, 2) < 1 && norm(f, 2) > 0) || (norm(f_, 2) < 1 && norm(f_, 2) > 0)
                   disp('break') 
                end
%                 f
%                 f_
%                 x
%                 z
                if norm(f - f_, 2) > 1e-10
                   disp('break') 
                end
            end
            
            function [f] = prodfun(y, plantCopy)
                x = y(1:xdim);
                z = y(xdim+1:xdim+zdim);
                params = y(zdim+xdim+1:end);
                %and now set object
                
                plantCopy = plantCopy.setParams(params);
                
                options.active_collision_options.terrain_only = true;
                nq = plantCopy.getNumPositions();
                [~,~,~,~,~,~,~,mu] = plantCopy.contactConstraints(zeros(nq,1),false,options.active_collision_options);
                
                q = zeros(nC,1);
                W = zeros(nC, nC);
                M = zeros(nC,nC*(1+nD));
                for k=1:nC,
                    M(k,1 + (k-1)*(1+nD)) = mu(k);
                    M(k,(2:nD+1) + (k-1)*(1+nD)) = -ones(nD,1);
                end
                
                assert(zdim == size(W,1));
                assert(zdim == size(M,1));
                
                
                %This gets the mu.  Now we can construct M and W
                
                %now from this, get mu
                
                
                g = W*z + M*x + q;
                %dg = [M W];
                
                f = z.*g;
                
                f_ = verify_cnstr.constraints{3}.eval([x; z]);
                
%                 f
%                 f_
%                 x
%                 z
                
                if (norm(f, 2) < 1 && norm(f, 2) > 0) || (norm(f_, 2) < 1 && norm(f_, 2) > 0)
                   disp('break') 
                end
                
                if norm(f - f_, 2) > 1e-10
                   disp('break') 
                end
                
                
                %df = diag(z)*dg + [zeros(zdim,xdim) diag(g)];
            end
            
            
            
            obj = obj@CompositeConstraint(constraints, n);
        end
    end
end