clear all; close all;
robotRadius = 0.3;
robot = RobotSimulator();
robot.enableLaser(true);
robot.setRobotSize(robotRadius);
robot.showTrajectory(true);

startLocation = [2.725 14.08];
endLocation = [14.38 2.225];
robotCurrentLocation = startLocation;
initialOrientation = -pi/4;
robotCurrentLocation = startLocation;
robotCurrentPose = [robotCurrentLocation initialOrientation];
robot.setRobotPose(robotCurrentPose);

mapInflated = copy(robot.Map);
inflate(mapInflated,robotRadius);

pose = robot.getRobotPose;
[range,angle] = robot.getRangeData;
laser=[range angle];
laserMaxrange = 5;

for m=1:21
    if isnan(range(m))
        range(m)=laserMaxrange;
    end
end

points = [range.*cos(angle+pose(3)) + pose(1), range.*sin(angle+pose(3)) + pose(2)];
plot(points(:,1), points(:,2), 'rx');

A = zeros(3,2,21);
b = zeros(3,21);

for m=1:20
    if (range(m) < laserMaxrange) && (range(m+1) < laserMaxrange)
        V = [robotCurrentLocation; points(m,:); points(m+1,:)];
        P = Polyhedron(V);
        plot(P);
        A(:,:,m) = P.A;
        b(:,m) = P.b;
    end
    if (range(m) == laserMaxrange) && (range(m+1) == laserMaxrange)
        V = [robotCurrentLocation; points(m,:); points(m+1,:)];
        P = Polyhedron(V);
        plot(P);
        A(:,:,m) = P.A;
        b(:,m) = P.b;
    end
    if (range(m) < laserMaxrange) && (range(m+1) == laserMaxrange)
        temp_point = [range(m).*cos(angle(m+1)+pose(3)) + pose(1), range(m).*sin(angle(m+1)+pose(3)) + pose(2)];
        V = [robotCurrentLocation; points(m,:); temp_point];
        P = Polyhedron(V);
        plot(P);
        A(:,:,m) = P.A;
        b(:,m) = P.b;
    end
    if (range(m) == laserMaxrange) && (range(m+1) < laserMaxrange)
        temp_point = [range(m+1).*cos(angle(m)+pose(3)) + pose(1), range(m+1).*sin(angle(m)+pose(3)) + pose(2)];
        V = [robotCurrentLocation; temp_point; points(m+1,:)];
        P = Polyhedron(V);
        plot(P);
        A(:,:,m) = P.A;
        b(:,m) = P.b;
    end
    
   
end