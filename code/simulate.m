function simulate(config_file)
% Run this to start the simulation

if nargin==0
    config_file='../data/config1.conf';
end

fprintf('Load config file...\n');
config = loadConfig(config_file);

data = initialize(config);

data.time = 0;
frame = 0;
fprintf('Start simulation...\n');

while (data.time < data.duration)
    tstart=tic;
    data = addDesiredForce(data);
    data = addWallForce(data);
    data = addAgentRepulsiveForce(data);
    data = applyForcesAndMove(data);
    
    % do the plotting
    set(0,'CurrentFigure',data.figure_floors);
    for floor=1:data.floor_count
        plotAgentsPerFloor(data, floor);
        plotFloor(data, floor);
    end
    if data.save_frames==1
        print('-depsc2',sprintf('frames/%s_%04i.eps', ...
            data.frame_basename,frame), data.figure_floors);
    end
    
    set(0,'CurrentFigure',data.figure_exit);
    plotExitedAgents(data);
    
    
    % print mean/median velocity of agents on each floor
%     for fi = 1:data.floor_count
%         avgv = arrayfun(@(agent) norm(agent.v), data.floor(fi).agents);
%         fprintf('Mean/median velocity on floor %i: %g/%g m/s\n', fi, mean(avgv), median(avgv));
%     end
    

    if (data.time + data.dt > data.duration)
        data.dt = data.duration - data.time;
        data.time = data.duration;
    else
        data.time = data.time + data.dt;
    end
    
    if data.agents_exited == data.total_agent_count
        fprintf('All agents are now saved (or are they?). Time: %.2f sec\n', data.time);
        fprintf('Total Agents: %i\n', data.total_agent_count);
        
        print('-depsc2',sprintf('frames/exited_agents_%s.eps', ...
            data.frame_basename), data.figure_floors);
        break;
    end
    
    
    telapsed = toc(tstart);
    pause(max(data.dt - telapsed, 0.01));
    fprintf('Frame %i done (took %.3fs; %.3fs out of %.3gs simulated).\n', frame, telapsed, data.time, data.duration);
    frame = frame + 1;
end

fprintf('Simulation done.\n');

