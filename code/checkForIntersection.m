function val = checkForIntersection(data, floor_idx, agent_idx)
% check an agent for an intersection with another agent or a wall
% the check is kept as simple as possible
%
%  arguments:
%   data            global data structure
%   floor_idx       which floor to check
%   agent_idx       which agent on that floor
%   agent_new_pos   vector: [x,y], desired agent position to check
%
%  return:
%   0               for no intersection
%   1               has an intersection with wall
%   2                                   with another agent

val = 0;

p = data.floor(floor_idx).agents(agent_idx).p;
r = data.floor(floor_idx).agents(agent_idx).r;

% check for agent intersection
for i=1:length(data.floor(floor_idx).agents)
    if i~=agent_idx
        if norm(data.floor(floor_idx).agents(i).p-p)*data.meter_per_pixel ...
                <= r +  data.floor(floor_idx).agents(i).r
            val=2;
            return;
        end
    end
end


% check for wall intersection
if lerp2(data.floor(floor_idx).img_wall_dist, p(1), p(2)) < r
    val = 1;
end

