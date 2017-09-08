classdef LinearComplementarityConstraintTest < CompositeConstraint
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
  
  %ANDYEDIT :Only mode 1, since that's all we need
  methods
    
    function obj = LinearComplementarityConstraintTest(nC, nD, mode,slack)
      %Generate W, q, M here:
       
      q = zeros(nC,1);
      W = zeros(nC,nC);
      M = zeros(nC,nC*(1+nD));
      %ANDYTODO: for now, like in contactConstraints, mu is just all
      %1s - might change later
      %ANDYTODO: I guess there are no parameters needed?
      %ANDYTODO: a bit of this will have to change if we need parameters,
      %including moving W, M creation to their respective functions.  This
      %means explicitly coding up the linear constraint.
      
      mu = ones(nC, 1);
      for k=1:nC,
          M(k,1 + (k-1)*(1+nD)) = mu(k);
          M(k,(2:nD+1) + (k-1)*(1+nD)) = -ones(nD,1);
      end
        
        
      zdim = size(W,2);
      
      
      xdim = size(M,2);
      
      assert(zdim == size(W,1));
      assert(zdim == size(M,1));
      
      constraints = {};
      n = 0;

      bcon = BoundingBoxConstraint([-inf(xdim,1);zeros(zdim,1)],inf(zdim+xdim,1));
      lincon = LinearConstraint(-q,inf(zdim,1),[M W]);
      nlcon = FunctionHandleConstraint(zeros(zdim,1),zeros(zdim,1)+slack,xdim+zdim,@prodfun);
      constraints = {bcon;lincon;nlcon};

      function [f,df] = prodfun(y)
        x = y(1:xdim);
        z = y(xdim+1:end);
        
        g = W*z + M*x + q;
        dg = [M W];
        
        f = z.*g;
        df = diag(z)*dg + [zeros(zdim,xdim) diag(g)];
      end


      
      obj = obj@CompositeConstraint(constraints, n);
    end
  end
end