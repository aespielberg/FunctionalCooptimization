function constraint = constructAboveGroundConstraint( lb, ub, plant, frame_name)

%run positiondiff once

paramsSym = sym(plant.getParamFrame().getCoordinateNames(), 'real');
plant = plant.setParams(paramsSym);

try
    frameId = plant.findLinkId(frame_name);
    com = plant.getBody(frameId).com;
    body = plant.getBody(frameId);
    geo = body.visual_geometry;
    bounding_geometry = geo{1}.getBoundingBoxPoints();
catch
    baseException = MException('NotSupportingConstraintException','Cannot handle frames');
    throw(baseException);
end

state_length = plant.getNumOutputs();

%first half of state is position, second half is velocity;
q = sym('q%d', [state_length / 2, 1], 'real');
qd = sym('qd%d', [state_length / 2, 1], 'real');

kinsol = plant.doKinematics(q, qd); %TODO: consider options in the future

options.rotation_type = 0;

num_pts = size(bounding_geometry, 2);

world_pts = sym(zeros(num_pts, 1));

for i = 1:1:num_pts
    point_in_frame = bounding_geometry(:, i);
    [x, J] = plant.forwardKin(kinsol, frameId, point_in_frame, options); %TODO: what does final argument, dj actually do?
    world_pts(i) = x(3); %z coordinate
end
f = world_pts;
df = jacobian(f, [q; qd; paramsSym]);



constraintHandle = matlabFunction(f, df, 'vars', {[q; qd; paramsSym]});


disp(['Finished making matlabFunction'])
%%%%edit positionDiff to be parameterized
%get x and J
%create matlab functions for each
%wrap them into a new function that computes x and J together
%make function handle constraint just operate on x and J


%constraint = FunctionHandleConstraint(lb, ub, (plant.getNumOutputs() + plant.getNumParams()), @(state)positionDiff(plant, frame_name, point_in_frame, target_point, state), 0);
constraint = FunctionHandleConstraint(lb, ub, (plant.getNumOutputs() + plant.getNumParams()), constraintHandle);
end