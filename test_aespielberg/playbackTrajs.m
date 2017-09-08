function playbackTrajs(costs, plant_name)

for i = 1:1:length(costs)
    if ~isempty(costs{i})
        i
        options.terrain = RigidBodyFlatTerrain(); %add basic terrain
        options.floating = true; %our robot isn't tethered anywhere in the world
        options.ignore_self_collisions = true; %We will enforce this through other constraints
        p = RigidBodyManipulator(plant_name,options); %our robot urdf.  This still needs to be changed to create parameters
        xtraj = costs{i}{5}
        xtraj = xtraj.setOutputFrame(p.getOutputFrame());
        p = p.setParams(costs{i}{3})
        v = p.constructVisualizer(); v.playback(xtraj)
        pause
    end




end