%% Using RobotSimulator
clear;
close all;
clc;

% set robotset and enable function
robotRadius = 0.15;
robot = RobotSimulator();
robot.enableLaser(true);
robot.setRobotSize(robotRadius);
robot.showTrajectory(true);

%% set start point and goal

% Sather Gate upside
startLocation = [12.82 15.07];

% Sather Gate downside
%startLocation = [12.82 10.77];

% Dwinde Hall front
%startLocation = [3.175 9.275];

% Sproul Hall
%startLocation = [23.92 18.77];

% Dwinde Hall Inside
%startLocation = [1.625 2.975];

% Set GOAL to REACH
endLocation = [29.56 14.07];
figure(1)
hold all
plot(endLocation(1),endLocation(2),'x')


%% set up the inial position
initialOrientation = 0;
robotCurrentLocation = startLocation;
robotCurrentPose = [robotCurrentLocation initialOrientation];
robot.setRobotPose(robotCurrentPose);

plan_path=[];
%% Use MPC to do path planing
%% Continue use MPC and PRM until reach the goal or hit the obstacle
while norm(robotCurrentPose(1:2) - endLocation)>0.1
    z_ref = endLocation;
      aa=robot.getRobotPose;
      bb=zeros(21,1);
      x = aa(1)+bb;
      y = aa(2)+bb;
      theta=aa(3)+bb;
      [range,angle]=robot.getRangeData;
      data=[ (180/pi)*theta range (180/pi)*angle (180/pi).*(angle+theta) x+range.*cos(angle+theta) y+range.*sin(angle+theta)];

      % get valid laser date
      o_data=[];
      for i=1:21
          if ~isnan(data(i,2)) 
              o_data=[o_data; data(i,5:6)];
          end
      end
      ob_data=o_data;

     % get obstacle distance from robot
      o_data(:,1)=o_data(:,1)-aa(1);
      o_data(:,2)=o_data(:,2)-aa(2);

      get_dis=[];
      for i=1:size(o_data,1)
          get_dis=[get_dis;norm(o_data(i,:))];
      end

      % get within 20m distace range laser data 
      obs_ref=[];
      for i=1:size(get_dis,1)
          if get_dis(i)<=20
             obs_ref=[obs_ref;ob_data(i,:)];
          end
      end

    % Using MPC path planer
    robotCurrentPose = robot.getRobotPose;
    [get_path,sol] = mpc_controller(robotCurrentPose,z_ref,obs_ref);
    
    % if the result is not successfully solved, then use the previous ones.
    % And reduce the size if use the previous ones.
    % if the path size too small to use, then use PRM get new plan_path.
    if sol.problem==0
        plan_path=get_path;
    elseif size(plan_path,1)>=8
        plan_path=plan_path(8:end,:);
    else
        % copy the curent path and inflate each occupancy grid
        mapInflated = copy(robot.Map);
        inflate(mapInflated,robotRadius);

        % Using PRM (probolistic roadmap method) to find path
        prm = robotics.PRM(mapInflated);
        % Set # of ramdon points
        prm.NumNodes = 200;
        prm.ConnectionDistance = 1;
        
        plan_path = findpath(prm, robotCurrentPose(1:2), endLocation);

            while isempty(plan_path)
                % No feasible path found yet, increase the number of nodes
                prm.NumNodes = prm.NumNodes + 50;

                % Use the |update| function to re-create the PRM roadmap with the changed
                % attribute
                update(prm);

                % Search for a feasible path with the updated PRM
                plan_path = findpath(prm,robotCurrentPose(1:2), endLocation);
            end
            
    end
    
    %
    figure(1)
    hold all

    plot(plan_path(:,1),plan_path(:,2),'.')
%
    %  Use Pure Pursuit to contorl the car
    controller = robotics.PurePursuit;

    % feed the certain length(13 steps) desird path to controller
    if size(plan_path,1)>=13
        controller.Waypoints = plan_path(1:13,:);
    else
        controller.Waypoints = plan_path(1:end,:);
    end

    %The maximum angular velocity acts as a saturation limit for rotational velocity
    controller.DesiredLinearVelocity = 0.4;
    controller.MaxAngularVelocity = 20;

    % As a general rule, the lookahead distance should be larger than the desired
    % linear velocity for a smooth path. The robot might cut corners when the
    % lookahead distance is large. In contrast, a small lookahead distance can
    % result in an unstable path following behavior. A value of 0.5 m was chosen
    % for this example.
    controller.LookaheadDistance = 0.6;

    % The controller runs at 10 Hz.
    controlRate = robotics.Rate(10);

    % set conditon for loop leaving
    robotCurrentLocation = robotCurrentPose(1:2);
    robotGoal = controller.Waypoints(end,:);
    distanceToGoal = norm(robotCurrentLocation - robotGoal);
    
    
    % Drvie robot 80 times or close(0.03) to desired path end point 
    flag=0;
    while ( distanceToGoal > 0.03 && flag<80)       
        [v, omega] = controller(robot.getRobotPose);
        drive(robot, v, omega);
        robotCurrentPose = robot.getRobotPose;
        distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);
        flag=flag+1;
        waitfor(controlRate);
        

    end

    %drive(robot, v, omega);
end

