function data = addWallForce(data)
%ADDWALLFORCE adds wall's force contribution to each agent

for fi = 1:data.floor_count

    for ai=1:length(data.floor(fi).agents)
        % get agents data
        p = data.floor(fi).agents(ai).p;
        ri = data.floor(fi).agents(ai).r;
        vi = data.floor(fi).agents(ai).v;
        
        % get direction from nearest wall to agent
        nx = lerp2(data.floor(fi).img_wall_dist_grad_x, p(1), p(2));
        ny = lerp2(data.floor(fi).img_wall_dist_grad_y, p(1), p(2));
        
        % get distance to nearest wall
        diW = lerp2(data.floor(fi).img_wall_dist, p(1), p(2));
        
        % get perpendicular and tangential unit vectors
        niW = [ nx ny];
        tiW = [-ny nx];
        
        
        % calculate force
        if diW < ri
            T1 = data.k * (ri - diW);
            T2 = data.kappa * (ri - diW) * dot(vi, tiW) * tiW;
        else
            T1 = 0;
            T2 = 0;
        end
        Fi = (data.A * exp((ri-diW)/data.B) + T1)*niW - T2;
        
        % add force to agent's current force
        data.floor(fi).agents(ai).f = data.floor(fi).agents(ai).f + Fi;
    end
end

