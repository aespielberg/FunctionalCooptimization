function [f] = positionDiff( plant, frame_name, point_in_frame, target_point, inp)
%plant: the plant
%state: the current state of the plant
%params: the plant's current parameters
%Create a constraint function from inputs:
%frame_name: the name of the frame in which our constrained point lies
%point_in_frame: the location of the point within that frame
%target_point: the target point where we want this to go
%outputs: the difference, f [Del_x, del_y, del_z, del_r, del_p, del_y] and its Jacobian as df



%findFrameId: it's one of these, so either the try or the catch should work

%TODO: is this working??
params_len = plant.getNumParams();
params = inp(end-params_len+1 : end);
x_dim = plant.getNumOutputs();
state = inp(1:x_dim); %TODO: remove hardcode
%double(params)
plant = plant.setParams(double(params));

try
    frameId = plant.findLinkId(frame_name);
    com = plant.getBody(frameId).com;
catch
    frameId = plant.findFrameId(frame_name);
    T = plant.getFrame(frameId).T;
    com = T(1:3, end);
end



state_length = length(state);
%first half of state is position, second half is velocity;
q = state(1:state_length/2);
qd = state((state_length/2 + 1):end);

kinsol = plant.doKinematics(q, qd); %TODO: consider options in the future

options.rotation_type = 1;


[x, J] = plant.forwardKin(kinsol, frameId, point_in_frame + com, options); %TODO: what does final argument, dj actually do?

f = target_point - x;
%df = -J; %df/dx = d(target_point)/dq - df/dq = 0 - J = -J
df = zeros(6, x_dim + length(params));
f
end

