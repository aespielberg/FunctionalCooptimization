classdef NonlinearComplementarityConstraintTest < CompositeConstraint
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
    test_cnstr;
  end
  
  methods
    
    function obj = NonlinearComplementarityConstraintTest(fun, xdim,zdim,paramdim,slack, test_cnstr)

      %ONLY EVER IN mode 2
      n = 0;
      constraints{1} = BoundingBoxConstraint([-inf(xdim,1);zeros(zdim,1); -inf(paramdim, 1); zeros(zdim, 1)],inf(2*zdim+xdim + paramdim,1));
      constraints{2} = FunctionHandleConstraint(zeros(zdim,1),zeros(zdim,1),xdim+2*zdim + paramdim,@slackeq);
      constraints{3} = FunctionHandleConstraint(zeros(zdim,1),zeros(zdim,1)+slack,xdim+2*zdim + paramdim,@slackprod);
% %       constraints{1}.grad_level = 0;
%        constraints{2}.grad_level = 0;
%        constraints{3}.grad_level = 0;
% %       constraints{1}.grad_method = 'numerical';
%        constraints{2}.grad_method = 'numerical';
%        constraints{3}.grad_method = 'numerical';
      n = zdim;

      
      function [f,df] = slackeq(y)
        x = y(1:xdim);
        z = y(xdim+1:xdim+zdim);
        params = y(xdim+zdim+1:xdim+zdim+paramdim);
        gamma = y(xdim+zdim+paramdim+1:end);
        [f,df] = fun([x;z;params]);
        f = f - gamma;
        %[test_f, test_df] = test_cnstr.constraints{2}.eval([x;z;gamma]);
        
%         if norm((test_f - f)) > 1e-6
%                 disp('break')
%         end
        %f = test_f;
        df = [df zeros(zdim)] ...
            - [zeros(zdim,zdim+xdim) zeros(zdim, paramdim) eye(zdim) ];
        
        %f = test_f;
        %df = [df(:, 1:(zdim + xdim)), zeros(zdim, paramdim), df(:, (zdim + xdim + paramdim + 1):end)];
        %df = test_df;

        %df = [test_df df(:, end-paramdim+1:end)];
      end
      
      function [f,df] = slackprod(y)
        x = y(1:xdim);
        z = y(xdim+1:xdim+zdim);
        params = y(xdim+zdim+1:xdim+zdim+paramdim);
        gamma = y(xdim+zdim+paramdim+1:end);
        
        f = z.*gamma;
        [test_f, test_df] = test_cnstr.constraints{3}.eval([x;z;gamma]);
        
%         if norm((test_f - f)) > 1e-6
%                 disp('break')
%         end
        %f = test_f;
        
        df = [zeros(zdim,xdim) diag(gamma) zeros(zdim, paramdim) diag(z) ];
        
        %f = test_f;
        %df = test_df;
        %df = [test_df df(:, end-paramdim+1:end)];
      end
      
     
      
      obj = obj@CompositeConstraint(constraints, n);
      obj.test_cnstr = test_cnstr;
      

     obj.slack_fun = fun;

      
    end
  end
end