function data = initAgents(data)

% place agents randomly in desired spots, without overlapping



function radius = getAgentRadius()
    %radius of an agent in meters
    radius = data.r_min + (data.r_max-data.r_min)*rand();
end

data.agents_exited = 0; %how many agents have reached the exit
data.total_agent_count = 0;

floors_with_agents = 0;
agent_count = data.agents_per_floor;
for i=1:data.floor_count
    data.floor(i).agents = [];
    [y,x] = find(data.floor(i).img_spawn);
    
    if ~isempty(x)
        floors_with_agents = floors_with_agents + 1;
        for j=1:agent_count
            cur_agent = length(data.floor(i).agents) + 1;
            
            % init agent
            data.floor(i).agents(cur_agent).r = getAgentRadius();
            data.floor(i).agents(cur_agent).v = [0, 0];
            data.floor(i).agents(cur_agent).f = [0, 0];
            data.floor(i).agents(cur_agent).m = data.m;
            data.floor(i).agents(cur_agent).v0 = data.v0;
            
            tries = 10;
            while tries > 0
                % randomly pick a spot and check if it's free
                idx = randi(length(x));
                data.floor(i).agents(cur_agent).p = [y(idx), x(idx)];
                if checkForIntersection(data, i, cur_agent) == 0
                    tries = -1; % leave the loop
                end
                tries = tries - 1;
            end
            if tries > -1
                %remove the last agent
                data.floor(i).agents = data.floor(i).agents(1:end-1);
            end
        end
        data.total_agent_count = data.total_agent_count + length(data.floor(i).agents);
    
        if length(data.floor(i).agents) ~= agent_count
            fprintf(['WARNING: could only place %d agents on floor %d ' ...
                'instead of the desired %d.\n'], ...
                length(data.floor(i).agents), i, agent_count);
        end
    end
end
if floors_with_agents==0
    error('no spots to place agents!');
end

end
