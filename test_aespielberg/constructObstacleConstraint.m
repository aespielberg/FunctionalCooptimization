function constraint = constructObstacleConstraint(plant, frame_name, point_in_frame, obstacle_center,obstacle_radius)

%run positiondiff once

paramsSym = sym(plant.getParamFrame().getCoordinateNames(), 'real');

if ~isempty(paramsSym)
    plant = plant.setParams(paramsSym);
end

try
    frameId = plant.findLinkId(frame_name);
    com = plant.getBody(frameId).com;
catch
    frameId = plant.findFrameId(frame_name);
    T = plant.getFrame(frameId).T;
    com = T(1:3, end);
end

state_length = plant.getNumOutputs();

%first half of state is position, second half is velocity;
q = sym('q%d', [state_length / 2, 1], 'real');
qd = sym('qd%d', [state_length / 2, 1], 'real');

kinsol = plant.doKinematics(q, qd); %TODO: consid%constraint = FunctionHandleConstraint(lb, ub, (plant.getNumOutputs() + plant.getNumParams()), @(state)positionDiff(plant, frame_name, point_in_frame, target_point, state), 0);er options in the future

options.rotation_type = 1;

samples = 5;
for i = 1:1:samples+1
% [x1, J] = plant.forwardKin(kinsol, frameId, point_in_frame + com, options); %TODO: what does final argument, dj actually do?
% [x2, J] = plant.forwardKin(kinsol, frameId, point_in_frame + [0; 0.5; 0], options); %the visual length
% [x3, J] = plant.forwardKin(kinsol, frameId, point_in_frame, options);
x = plant.forwardKin(kinsol, frameId, point_in_frame - [0; paramsSym(2); 0] * (i - 1)/samples, options);
point = x(1:3);
f(i) = norm(obstacle_center - point) - obstacle_radius;
end


%you want to take x and make sure it is not within a certain radius of a
%obstacle_center
% 
% point1 = x1(1:3);
% point2 = x2(1:3);
% point3 = x3(1:3);
% 
% f1 = norm(obstacle_center - point1) - obstacle_radius;
% f2 = norm(obstacle_center - point2) - obstacle_radius;
% f3 = norm(obstacle_center - point3) - obstacle_radius;
% 
% f = [f1; f2; f3];



df = jacobian(f, [q; qd; paramsSym]);



constraintHandle = matlabFunction(f, df, 'vars', {[q; qd; paramsSym]});

disp(['Finished making matlabFunction'])
%%%%edit positionDiff to be parameterized
%get x and J
%create matlab functions for each
%wrap them into a new function that computes x and J together
%make function handle constraint just operate on x and J


constraint = FunctionHandleConstraint(zeros(samples+1, 1), inf(samples+1, 1), (plant.getNumOutputs() + plant.getNumParams()), constraintHandle);
end