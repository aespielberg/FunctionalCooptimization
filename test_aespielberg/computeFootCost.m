function [ cost ] = computeFootCost(xtraj, dt, plant )

params = plant.getParams();
N = length(dt)+1; %dt doesn't include time 0
x = zeros(N,xtraj.getNumOutputs());
t = 1:N;
t(1) = 0;
x(1,:) = xtraj.eval(0)';

frames = {'leg_link2', 'leg_link3'};

for i=2:N
    for j = 1:2
        t(i) = t(i-1) + dt(i-1);
        x(i,:) = xtraj.eval(t(i))';
        frame_name = frames{j};
        
        
        try
            frameId = plant.findLinkId(frame_name);
        catch
            frameId = plant.findFrameId(frame_name);
            T = plant.getFrame(frameId).T;
        end
        
        state_length = plant.getNumOutputs();
        
        %first half of state is position, second half is velocity;
        
        
        kinsol = plant.doKinematics(x(i,1:10), x(i,11:20));
        
        options.rotation_type = 1;
        
        pos = plant.forwardKin(kinsol, frameId, [0; 0; -params(1)] , options);
        
        z(i*j) = pos(3);
    end
end

dt = [dt; dt];

cost = sum(dt(1:30)'.*z(1:30).*z(1:30));
