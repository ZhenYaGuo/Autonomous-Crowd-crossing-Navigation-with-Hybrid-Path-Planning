% This GOAL of this project is to ultilize the map and laser scan data to find the obstacle-free optimal path
% and use MPC to tracking the optimal path with obstacle-avoidance
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Using RobotSimulator
clear;
close all;
clc;

% Set robotsimulator and enable function
robotRadius = 0.3;
robot = RobotSimulator();
%% Enable Laser Scan
robot.enableLaser(true);
robot.setRobotSize(robotRadius);
robot.showTrajectory(true);

%% Set start point and goal
% Set START to BEGIN
%startLocation = [2.725 14.08];
startLocation = [2.7 1];
%startLocation = [4.225 10.58]
figure(1)
hold all
plot(startLocation(1),startLocation(2),'o')

% Set GOAL to REACH
endLocation = [14.38 2.225];
figure(1)
hold all
plot(endLocation(1),endLocation(2),'x')

%% Set up the inial position and pose
initialOrientation = pi/2;
%initialOrientation = 0;
robotCurrentLocation = startLocation;
robotCurrentPose = [robotCurrentLocation initialOrientation];
robot.setRobotPose(robotCurrentPose);

plan_path=[];

%% Use MPC to do path planing
% Continue use MPC and PRM until reach the goal or hit the obstacle
% Get optimized PRM circle waypoints as z_ref
mapInflated = copy(robot.Map);
inflate(mapInflated,robotRadius);
%% Here is where the map gets inflated
optPRMPoints = getOptimalPRMPoints1(mapInflated,startLocation,endLocation);
%PointNo=2

%% Define 3 walking human obstacles
% These 3 humans will walk linearly between their start and end positions

numHumans = 3;
map = copy(robot.Map);

humansStart = [8.8 2.2; 4 10.9; 2 3];
humansEnd = [12 6.5; 12 10; 5.2 8];
humansMoveDist = [(humansEnd(1,:) - humansStart(1,:))/86;
    (humansEnd(2,:) - humansStart(2,:))/30;
    (humansEnd(3,:) - humansStart(3,:))/50];
humansWalkDirection = [1 1;1 1;1 1];
humansCurPos = humansStart;

% To initialise humans on the map
map = moveHumans(map, humansCurPos, humansCurPos);
robot.Map = map;
robot.setRobotPose(robotCurrentPose);
robot.enableLaser(true);
robot.setRobotSize(robotRadius);
robot.showTrajectory(true);

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% get first reference point
read=0;
M = [];
N_optPRM=size(optPRMPoints,1);
dis_optPRM=[];
for i=1:N_optPRM
    dis_optPRM=[dis_optPRM;norm(robotCurrentPose(1:2)-optPRMPoints(i,:))]
end
[dis_min,PointNo]=min(dis_optPRM);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

while norm(robotCurrentPose(1:2) - endLocation)>0.1
    yalmip('clear')
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % dis_min>=5(2 connect point distance), then rerun PRM again to find
    % new path
    if norm(robotCurrentPose(1:2)-optPRMPoints(PointNo,:))>=5
            optPRMPoints=[];
            optPRMPoints=getOptimalPRMPoints1(mapInflated,robotCurrentPose(1:2),endLocation);
            PointNo = 1;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    dis_nextPRM=1;
    if norm(robotCurrentPose(1:2)-optPRMPoints(PointNo,:))<dis_nextPRM
        PointNo=PointNo+1
    end
    
    if PointNo==N_optPRM+1
        PointNo=N_optPRM;
    end

    read=robotCurrentPose(1:2);
    z_ref = optPRMPoints(PointNo,:)
    if PointNo==N_optPRM
        z_ref=endLocation;
        %     else
        %         z_ref = optPRMPoints(PointNo,:)
    end
    
    %plot reference point
    figure(1)
    plot_ref=[];
    plot_ref=[plot_ref,plot(z_ref(1),z_ref(2),'go','MarkerSize',10)]
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Using MPC Local Path Planer
    pose = robot.getRobotPose;
    [range,angle] = robot.getRangeData;
    laser=[range angle];

    robotCurrentPose = robot.getRobotPose;
    [get_path,sol,plotHandles] = mpc_controller(robotCurrentPose,z_ref,laser);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    
    % If the result is not successfully solved, then use the previous ones.
    % And reduce the size if use the previous ones.
    % If the path size too small to use, then use PRM get new plan_path.
    if sol.problem == 0
        plan_path = get_path;
        %     elseif size(plan_path,1) >= 8
        %         plan_path = plan_path(8:end,:);
    else
        % if MPC doesn't solve, then drive to the next optPRM point
        plan_path = optPRMPoints(PointNo:end,:);
        if norm(robotCurrentPose(1:2)-optPRMPoints(PointNo,:))<dis_nextPRM
            plan_path = optPRMPoints(PointNo+1:end,:);
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %plot the path 
    figure(1)
    hold all
    plan_path
    if size(plan_path,1)~=0
        
        plot(plan_path(:,1),plan_path(:,2),'.')
        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %  Use Pure Pursuit to contorl the car
    controller = robotics.PurePursuit;
    
    % Feed the middle point of plan_path to the pursuit controller

    if size(plan_path,1)~=0
        controller.Waypoints = plan_path(1:ceil(end/2),:);
    else
        controller.Waypoints = robotCurrentPose(1:2);
    end
    
    % The maximum angular velocity acts as a saturation limit for rotational velocity
    controller.DesiredLinearVelocity = 0.4;
    controller.MaxAngularVelocity = 20;
    
    % As a general rule, the lookahead distance should be larger than the desired
    % linear velocity for a smooth path. The robot might cut corners when the
    % lookahead distance is large. In contrast, a small lookahead distance can
    % result in an unstable path following behavior. A value of 0.6 m was chosen
    % for this example.
    controller.LookaheadDistance = 0.6;
    
    % The controller runs at 10 Hz.
    controlRate = robotics.Rate(10);
    
    % Set conditon for loop leaving
    robotCurrentLocation = robotCurrentPose(1:2);
    robotGoal = controller.Waypoints(end,:);
    distanceToGoal = norm(robotCurrentLocation - robotGoal);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Drive robot 30 times or close(0.1) to desired path end point
    flag=0;
    while ( distanceToGoal > 0.1 && flag < 30)
        [v, omega] = controller(robot.getRobotPose);
        drive(robot, v, omega);
        robotCurrentPose = robot.getRobotPose;
        distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);
        flag = flag + 1;
        waitfor(controlRate);
        % saves frame for movie
        M = [M, getframe];
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % If the robot stay at the same place, then turn the pose and look for
    % new feasible rigion to push robot move
    if norm(robotCurrentPose(1:2)-read)<=0.05
        for i=1:3
        drive(robot, -0.2, 100);
        robotCurrentPose = robot.getRobotPose;
        waitfor(controlRate);
        end     
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for j=1:4
        humansEndPos = humansCurPos + humansWalkDirection.*humansMoveDist;
        for i=1:numHumans
            pt1 = humansEndPos(i,:);
            pt2 = humansEnd(i,:);
            pt3 = humansStart(i,:);
            if (pdist([pt1;pt2],'euclidean') < 0.03)
                humansWalkDirection(i,:) = -humansWalkDirection(i,:);
            end
            if (pdist([pt1;pt3],'euclidean') < 0.03)
                humansWalkDirection(i,:) = -humansWalkDirection(i,:);
            end
        end
        map = moveHumans(map, humansCurPos, humansEndPos);
        robotCurrentPose = robot.getRobotPose;
        robot.Map = map;
        robot.setRobotPose(robotCurrentPose);
        robot.enableLaser(true);
        robot.setRobotSize(robotRadius);
        robot.showTrajectory(true);
        humansCurPos = humansEndPos;
        M = [M, getframe];
    end    
    
    %drive(robot, v, omega);
    %     for i = 1 : length(plotHandles)
    %         set(plotHandles(i),'Visible','off');
    %     end
    delete(plotHandles)
    delete(plot_ref)
end
%% write frames as .avi file
% v = VideoWriter('movie with humans final.avi');
% open(v);
% for i = 1:length(M);
%     writeVideo(v,M(i));
% end
% close(v);