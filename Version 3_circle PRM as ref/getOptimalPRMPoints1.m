function optPRMPoints = getOptimalPRMPoints1(map,startLocation,endLocation)
% Closest to destiny

% Using PRM (probolistic roadmap method) to find path
prm = robotics.PRM(map);
% Set # of ramdon points
prm.NumNodes = 50;
prm.ConnectionDistance = 2.5;

plan_path = findpath(prm, startLocation, endLocation);

while isempty(plan_path)
    % No feasible path found yet, increase the number of nodes
    prm.NumNodes = prm.NumNodes + 50;
    
    % Use the |update| function to re-create the PRM roadmap with the changed
    % attribute
    update(prm);
    
    % Search for a feasible path with the updated PRM
    plan_path = findpath(prm,startLocation, endLocation);
end

PRMPathSize = size(plan_path,1);
optPRMPoints = zeros(PRMPathSize,2);

% Set resolution and radius of circle around PRM
circleRes = 30;
circleRadius = 1;

figure(1)
show(map);
hold all

for i = 2 : PRMPathSize-1
    centre = plan_path(i,:);
    theta = linspace(0,2*pi,circleRes).';
    circlePoints = [circleRadius.*cos(theta) + centre(1), circleRadius.*sin(theta) + centre(2)];
    circleOccupancy = getOccupancy(map,circlePoints);
    % To find not occupied points
    circleFreeIndex = find(circleOccupancy == 0);
    circleFreePoints = circlePoints(circleFreeIndex,:);
    plot(circleFreePoints(:,1), circleFreePoints(:,2));
    hold all
    circleNormDist = zeros(size(circleFreePoints,1),1);
    for j = 1 : size(circleFreePoints,1)
        circleNormDist(j) = norm(circleFreePoints(j,:) - endLocation);
    end
    optNormDist = min(circleNormDist);
    optNormIndex = find(circleNormDist == optNormDist);
    optPRMPoints(i,:) = circleFreePoints(optNormIndex,:);
end

%% Code to plot the PRM and optPRM points for tuning

figure(1)
hold all
plot(startLocation(1),startLocation(2),'o')

figure(1)
hold all
plot(endLocation(1),endLocation(2),'x')

figure(1)
hold all
plot(plan_path(:,1),plan_path(:,2),'.');

figure(1)
hold all
plot(optPRMPoints(:,1),optPRMPoints(:,2),'+');

end




