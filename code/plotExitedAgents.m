function plotExitedAgents(data)
%plot time vs exited agents

hold on;
plot(data.time, data.agents_exited, 'r-');
hold off;
