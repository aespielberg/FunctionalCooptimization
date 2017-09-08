function constraint = constructInertialConstraint(plant, frame_name,link_num)

%run positiondiff once

paramsSym = sym(plant.getParamFrame().getCoordinateNames(), 'real');
plant = plant.setParams(paramsSym);

try
    frameId = plant.findLinkId(frame_name);
    l = 2*abs(plant.getBody(frameId).com(3));
catch
    frameId = plant.findFrameId(frame_name);
    T = plant.getFrame(frameId).T;
    l = 2*T(3, end);
end

m = paramsSym(2*link_num-1);
Ic = paramsSym(2*link_num);
f = (Ic - m*(l^2)/12);
df = jacobian(f,paramsSym);

constraintHandle = matlabFunction(f, df, 'vars', {paramsSym});

disp(['Finished making matlabFunction'])

constraint = FunctionHandleConstraint(0, 0, plant.getNumParams(), constraintHandle,1);
end