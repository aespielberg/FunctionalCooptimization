function recordResults(p, params, xtraj)
%first get filename
logname = [char(datetime), 'quadruped'];
logname_vels = [char(datetime), 'quadruped','vels'];
logname_ar = [char(datetime), 'quadruped', 'ar'];
folder_name = 'animations';

filename_ar = [folder_name '/' logname_ar '.txt.'];
filename_vels = [folder_name '/' logname_vels '.txt'];
filename = [folder_name '/' logname '.txt'];
fid_ar = fopen(filename_ar, 'wt');
fid_vels = fopen(filename_vels, 'wt');
fid = fopen(filename, 'wt');

%TODO: default params somehow
default_params = double(p.getParams());
fprintf(fid, 'params\n');
for k = 1:1:length(params)
    fprintf(fid, '%s ' , num2str(params(k)));
end
fprintf(fid, '\n');
fprintf(fid, 'default_params\n');
for k = 1:1:length(default_params)
    fprintf(fid, '%s ' , num2str(default_params(k)));
end
fprintf(fid, '\n');


frames = 60;

%joints:
joints = p.body.jointname;


deltat = xtraj.tspan(2) - xtraj.tspan(1);
fprintf(fid, 'waypoints\n');


xtraj_at_points = xtraj.eval(xtraj.pp.breaks);

%sample_vels
for i = 1:1:size(xtraj_at_points, 2)
    for j = size(xtraj_at_points, 1)/2 + 7:1:length(xtraj_at_points(:, i))
       fprintf(fid_vels, '%s ' , num2str(xtraj_at_points(j, i))); 
        
    end
    fprintf(fid_vels, '\n');
end


%sample points
for i = 0:1:deltat
    for j = 1:1:frames
        val = xtraj.eval(i + j / frames);
        len_val = length(val);
        val = val(1:len_val/2); %get rid of velocities.
        for k = 1:1:length(val)
            fprintf(fid, '%s ' , num2str(val(k)));
        end
        fprintf(fid, '\n');
    end
end

%sample points w/ anchor points
p = p.setParams(params);
for i = 0:1:deltat
    for j = 1:1:frames
        val = xtraj.eval(i + j / frames);
        len_val = length(val);
        vel = val(len_val/2 + 1:end);
        val = val(1:len_val/2); %get rid of velocities.
        
        for k = 3:1:p.getNumBodies() %one of them is the world
%           body = p.getBody(k);
%             geo = body.visual_geometry;
%             bounding_geometry = geo{1}.getBoundingBoxPoints();
%             point_in_frame = bounding_geometry(:, 1);
            kinsol = p.doKinematics(val, vel);
            [x] = p.forwardKin(kinsol, k, zeros(3, 1));
            for l = 1:1:3
                fprintf(fid_ar, '%s ' , num2str(x(l)));
            end
        end
        
        
        for k = 1:1:length(val)
            fprintf(fid_ar, '%s ' , num2str(val(k)));
        end
        fprintf(fid_ar, '\n');
    end
end

fclose(fid);
fclose(fid_vels);

end