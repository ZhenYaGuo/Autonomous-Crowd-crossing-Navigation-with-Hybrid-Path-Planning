%% Use NOn-Convex set and Convex MPC to tracking the Optimal Path.
%%
function [plan_path,sol,plotHandles] = mpc_controller(pose,z_ref,laser) 
%% Set N-steps prediction
N = 20;
% Transform nan into a large number
laserMaxrange = 5;
for m=1:21
    if isnan(laser(m,1))
        laser(m,1)=laserMaxrange;
    end
end
range = laser(:,1);
angle = laser(:,2);
laser(:,1)=laser(:,1)-0.5;

%% Plot LIDAR points and feasible sets
plotHandles = [];
points = [range.*cos(angle+pose(3)) + pose(1), range.*sin(angle+pose(3)) + pose(2)];
plotHandles = [plotHandles, plot(points(:,1), points(:,2), 'rx')];
robotCurrentLocation = [pose(1), pose(2)];

for m=1:20

    if (range(m) < laserMaxrange) && (range(m+1) < laserMaxrange)
        V_lidar = [robotCurrentLocation; points(m,:); points(m+1,:)];
        P(m) = Polyhedron(V_lidar);
        
        plotHandles = [plotHandles, plot(P(m),'alpha', 0.2)];
%         A(:,:,m) = P.A;
%         b(:,m) = P.b;
    end
    if (range(m) == laserMaxrange) && (range(m+1) == laserMaxrange)
        V_lidar = [robotCurrentLocation; points(m,:); points(m+1,:)];
        P(m) = Polyhedron(V_lidar);
        
        plotHandles = [plotHandles, plot(P(m), 'alpha', 0.2)];
%         A(:,:,m) = P.A;
%         b(:,m) = P.b;
    end
    if (range(m) < laserMaxrange) && (range(m+1) == laserMaxrange)
        temp_point = [range(m).*cos(angle(m+1)+pose(3)) + pose(1), range(m).*sin(angle(m+1)+pose(3)) + pose(2)];
        V_lidar = [robotCurrentLocation; points(m,:); temp_point];
        P(m) = Polyhedron(V_lidar);
       
        plotHandles = [plotHandles, plot(P(m), 'alpha', 0.2)];
%         A(:,:,m) = P.A;
%         b(:,m) = P.b;
    end
    if (range(m) == laserMaxrange) && (range(m+1) < laserMaxrange)
        temp_point = [range(m+1).*cos(angle(m)+pose(3)) + pose(1), range(m+1).*sin(angle(m)+pose(3)) + pose(2)];
        V_lidar = [robotCurrentLocation; temp_point; points(m+1,:)];
        P(m) = Polyhedron(V_lidar);
        
        plotHandles = [plotHandles, plot(P(m), 'alpha', 0.2)];
%         A(:,:,m) = P.A;
%         b(:,m) = P.b;
    end
end

grid off
xlim([0 17.9])
ylim([0 17.2])

% convex_seta=Polyhedron('V',points);
% convex_seta.plot('alpha',0.2)
% grid off
% xlim([0 17.9])
% ylim([0 17.2])

%% The state of each step is in polar coordinates of the robot(r,theta)
theta = sdpvar(1,N+1); % theta, relative to the robot
r = sdpvar(1,N+1); % radius, relative to the robot

% x,y,beta is the global coordinate position
x=pose(1);
y=pose(2);
beta=pose(3);
%% Convert feasible robot(r,theta)_local fram into feasible robot(x,y)_local fram 
[x_local_fe,y_local_fe]=pol2cart(laser(:,2),laser(:,1));

% fitting the non-convex set into convex set
convex_set=Polyhedron('V',[x_local_fe,y_local_fe]);

% We are trying to use this convex set to get global minimum first 
% Then we filter the infeasible points given the feasible set

%% Define the local cart coordinate transfrom and global cart coordinate transfrom
tf_localxy=@(theta,r) [r*cos(theta);r*sin(theta)]; % get[x_local,y_local]
tf_globalxy=@(x_local,y_local,x,y,beta) [x;y]+[cos(beta) -sin(beta); sin(beta) cos(beta)]*[x_local;y_local];
% get [x_global;y_global]
%% Define objective function and Constr and close factor 'a'
J = 0;
Constr = [];
a = 0.05^2;
%% Define Initial Constranints;
% Convert to local cart coordinate
local_buffer = tf_localxy(theta(1),r(1));
x_local(1) = local_buffer(1);
y_local(1) = local_buffer(2);
% Convert to global cart coordinate
global_buffer = tf_globalxy(x_local(1),y_local(1),x,y,beta);
x_global(1) = global_buffer(1);
y_global(1) = global_buffer(2);
% let each point close to each other engouh
Constr = [Constr x_local(1)^2+y_local(1)^2<=a];
% Constr the point inside the local convex_set
Constr = [Constr convex_set.H(:,1:2)*[x_local(1);y_local(1)]<=convex_set.H(:,3)]
% Penalize the distance to reference
J=J+ (x_global(1)-z_ref(1))^2+(y_global(1)-z_ref(2))^2;
%% Define Constraints
for k = 2:N
    % Convert to local cart coordinate
    local_buffer= tf_localxy(theta(k),r(k));
    x_local(k) = local_buffer(1);
    y_local(k) = local_buffer(2);
    % Convert to global cart coordinate
    global_buffer= tf_globalxy(x_local(k),y_local(k),x,y,beta);
    x_global(k) = global_buffer(1);
    y_global(k) = global_buffer(2);
    % let each point close to each other engouh
    Constr= [Constr ((x_local(k)-x_local(k-1))^2+(y_local(k)-y_local(k-1))^2)<=a];
    % Constr the point inside the local convex_set
    Constr = [Constr convex_set.H(:,1:2)*[x_local(k);y_local(k)]<=convex_set.H(:,3)]; 
    % Penalize the distance to reference
    J=J+ (x_global(k)-z_ref(1))^2+(y_global(k)-z_ref(2))^2; 
end

%% Add more terminal cost for end point
J=J+ 9*(x_global(N)-z_ref(1))^2+(y_global(N)-z_ref(2))^2;

%% Using Optimize
% Set options for YALMIP and solver
% Choose maximum iterations to 350 to speed up
% Choose algorithm ('trust-region-reflective','sqp','sqp-legacy','interior-point')
options = sdpsettings('verbose',1,'solver','fmincon','fmincon.maxit',350, ...
    'fmincon.algorithm','interior-point');
sol = optimize(Constr, J, options);

%% Get Optimized Path
% Filter the infeasible points
thetaOpt = double(theta);
rOpt = double(r);
plan_path=[];
for k=1:N
    localOpt_buffer= tf_localxy(thetaOpt(k),rOpt(k));
    xOpt_local(k) = localOpt_buffer(1);
    yOpt_local(k) = localOpt_buffer(2);

    % find the point that is not inside the local feasible non_convex_set
    if ~inpolygon(xOpt_local(k),yOpt_local(k),x_local_fe,y_local_fe)
        break;
    end
    globalOpt_buffer= tf_globalxy(xOpt_local(k),yOpt_local(k),x,y,beta);
    xOpt_global(k) = globalOpt_buffer(1);
    yOpt_global(k) = globalOpt_buffer(2);
    plan_path=[plan_path;xOpt_global(k) yOpt_global(k)];
end



