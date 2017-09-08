function [f, df] = hexapodMotorConstraint(z)

params = z(1:11);


u = z(11 + 1:end);
u_size = length(u);


% 
power = u;

powers = [15.3, 28.3, 60];
lengths = [0.32, 0.356, 0.42] * 3;
heights = [0.5, 0.506, 0.611] * 2;
widths = [0.4, 0.4, 0.41] * 2;
masses = [0.55, 0.72, 1.26] * 6;

function [min_length, min_mass, min_width, min_height, df_length, df_width, df_height, df_mass] = powerToDimensions(power)

    min_length = zeros(size(power));
    min_mass = zeros(size(power));
    min_width = zeros(size(power));
    min_height = zeros(size(power));
    
    df_length = zeros(size(power));
    df_mass = zeros(size(power));
    df_width = zeros(size(power));
    df_height = zeros(size(power));
    

    for i = 1:1:size(power(:))
        if power(i) < powers(1)
            power_diff = (powers(2) - powers(1));
            slope_length = (lengths(2) - lengths(1))/power_diff;
            slope_mass = (masses(2) - masses(1))/power_diff;
            slope_width = (widths(2) - widths(1))/power_diff;
            slope_height = (heights(2) - heights(1))/power_diff;
            min_length(i) = (power(i) - 0.0) * slope_length + lengths(1);
            min_mass(i) = (power(i) - 0.0) * slope_mass + masses(1);
            min_width(i) = (power(i) - 0.0) * slope_width + widths(1);
            min_height(i) = (power(i) - 0.0) * slope_height + heights(1);
            
        elseif power(i) < powers(2)
            
            power_diff = (powers(3) - powers(2));
            slope_length = (lengths(3) - lengths(2))/power_diff;
            slope_mass = (masses(3) - masses(2))/power_diff;
            slope_width = (widths(3) - widths(2))/power_diff;
            slope_height = (heights(3) - heights(2))/power_diff;
            min_length(i) = (power(i) - powers(1)) * slope_length + lengths(2);
            min_mass(i) = (power(i) - powers(1)) * slope_mass + masses(2);
            min_width(i) = (power(i) - powers(1)) * slope_width + widths(2);
            min_height(i) = (power(i) - powers(1)) * slope_height + heights(2);
        else
            %TODO: same as above, is that okay?
            power_diff = (powers(3) - powers(2));
            slope_length = (lengths(3) - lengths(2))/power_diff;
            slope_mass = (masses(3) - masses(2))/power_diff;
            slope_width = (widths(3) - widths(2))/power_diff;
            slope_height = (heights(3) - heights(2))/power_diff;
            min_length(i) = (power(i) - powers(1)) * slope_length + lengths(2);
            min_mass(i) = (power(i) - powers(1)) * slope_mass + masses(2);
            min_width(i) = (power(i) - powers(1)) * slope_width + widths(2);
            min_height(i) = (power(i) - powers(1)) * slope_height + heights(2);
        end
        
        df_height(i) = slope_height;
        df_width(i) = slope_width;
        df_length(i) = slope_length;
        df_mass(i) = slope_mass;
        
    end
    
    
end
    
    [min_length, min_mass, min_width, min_height, df_length, df_width, df_height, df_mass] = powerToDimensions(power);
    grad_mass = [1, zeros(1, 10), -df_mass'];
    grad_length = [0, 0, 1, zeros(1, 8), -df_length'];
    grad_width = [0, 0, 0, 1, zeros(1, 7), -df_width'];
    grad_height = [0, 0, 0, 0, 1, zeros(1, 6), -df_height'];
    
    
    f = [params(1) - min_mass(:); params(3) - min_length(:); params(4) - min_width(:); params(5) - min_height(:)];
    %grad_length = [1, zeros(1, 10), -v(:)' * df_length, -u(:)' * df_length];
    %grad_mass = [0, 0, 1, zeros(1, 8), -v(:)' * df_mass, -u(:)' * df_mass];
    %min(f)
    df = [repmat(grad_mass, u_size, 1); repmat(grad_length, u_size, 1); repmat(grad_width, u_size, 1); repmat(grad_height, u_size, 1)];

    
end