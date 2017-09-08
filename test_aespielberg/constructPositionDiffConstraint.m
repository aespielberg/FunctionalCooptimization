function constraint = constructPositionDiffConstraint( lb, ub, plant, frame_name, point_in_frame, target_point)

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

kinsol = plant.doKinematics(q, qd); %TODO: consider options in the future

options.rotation_type = 1;


[x, J] = plant.forwardKin(kinsol, frameId, point_in_frame + com, options); %TODO: what does final argument, dj actually do?

f = target_point - x;
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