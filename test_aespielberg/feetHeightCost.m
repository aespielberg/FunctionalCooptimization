function cost = feetHeightCost(plant, frame_name)

%frame should be the leg name

paramsSym = sym(plant.getParamFrame().getCoordinateNames(), 'real');
plant = plant.setParams(paramsSym);

h = sym('h%d', [1, 1], 'real');
u = sym('u%d', [plant.getNumInputs(), 1], 'real');

try
    frameId = plant.findLinkId(frame_name);
    body = plant.getBody(frameId);
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





point_in_frame = [0; 0; -paramsSym(1)/2.0];
[x, J] = plant.forwardKin(kinsol, frameId, point_in_frame, options); %TODO: what does final argument, dj actually do?
height = x(3); %z coordinate

f = 1000*height^2; %constrain positive
df = jacobian(f, [h; q; qd; u; paramsSym]);


x = [q; qd];
cost = matlabFunction(f, df, 'vars', [{h}, {x}, {u}, {paramsSym}]);


disp(['Finished making matlabCost'])

end