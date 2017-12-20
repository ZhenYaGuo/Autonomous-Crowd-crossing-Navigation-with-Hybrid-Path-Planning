function data = initEscapeRoutes(data)
%INITESCAPEROUTES Summary of this function goes here
%   Detailed explanation goes here

for i=1:data.floor_count

    boundary_data = zeros(size(data.floor(i).img_wall));
    boundary_data(data.floor(i).img_wall) =  1;
    %if (i == 1)
        boundary_data(data.floor(i).img_exit) = -1;
    %else
        boundary_data(data.floor(i).img_stairs_down) = -1;
    %end
    
    exit_dist = fastSweeping(boundary_data) * data.meter_per_pixel;
    [data.floor(i).img_dir_x, data.floor(i).img_dir_y] = ...
        getNormalizedGradient(boundary_data, -exit_dist);
end

