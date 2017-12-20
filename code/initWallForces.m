function data = initWallForces(data)
%INITWALLFORCES init wall distance maps and gradient maps for each floor

for i=1:data.floor_count

    % init boundary data for fast sweeping method
    boundary_data = zeros(size(data.floor(i).img_wall));
    boundary_data(data.floor(i).img_wall) = -1;
    
    % get wall distance
    wall_dist = fastSweeping(boundary_data) * data.meter_per_pixel;
    data.floor(i).img_wall_dist = wall_dist;
    
    % get normalized wall distance gradient
    [data.floor(i).img_wall_dist_grad_x, ...
     data.floor(i).img_wall_dist_grad_y] = ...
     getNormalizedGradient(boundary_data, wall_dist-data.meter_per_pixel);
end

