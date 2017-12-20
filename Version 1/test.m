%%
clear;
close all;
clc;
robotRadius = 0.15;
robot = RobotSimulator();
robot.enableLaser(true);
robot.setRobotSize(robotRadius);
robot.showTrajectory(true);


%%
mapInflated = copy(robot.Map);
inflate(mapInflated,robotRadius);
prm = robotics.PRM(mapInflated);
prm.NumNodes = 200;
prm.ConnectionDistance = 10;

startLocation = [3 5];
endLocation = [26.25 20.22];
path = findpath(prm, startLocation, endLocation)

while isempty(path)
    % No feasible path found yet, increase the number of nodes
    prm.NumNodes = prm.NumNodes + 10;

    % Use the |update| function to re-create the PRM roadmap with the changed
    % attribute
    update(prm);

    % Search for a feasible path with the updated PRM
    path = findpath(prm, startLocation, endLocation);
end

show(prm, 'Map', 'off', 'Roadmap', 'off')
%%
controller = robotics.PurePursuit
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.2;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.5;
goalRadius = 0.1;
robotCurrentLocation = path(1,:);
robotGoal = path(end,:);
distanceToGoal = norm(robotCurrentLocation - robotGoal);
controlRate = robotics.Rate(100);


%%

initialOrientation = 0;

robotCurrentPose = [robotCurrentLocation initialOrientation];

robot.setRobotPose(robotCurrentPose);

while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robot.getRobotPose);
    
    % Simulate the robot using the controller outputs
    drive(robot, v, omega);
    
    % Extract current location information from the current pose
    robotCurrentPose = robot.getRobotPose;
    
    % Re-compute the distance to the goal
    
    aa=robot.getRobotPose;
    bb=zeros(21,1);
    x = aa(1)+bb;
    y = aa(2)+bb;
    theta=(180/pi)*aa(3)+bb;
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);
    [range,angle]=robot.getRangeData;
    %data = [v, omega]
    data=[ theta range (180/pi)*angle (180/pi).*angle+theta x+range.*cos(angle+theta) y+range.*sin(angle+theta)]

    waitfor(controlRate);

end





