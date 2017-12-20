function [plan_path,sol] = mpc_controller(pose,z_ref,laser)
%% Use human-simulated prior sector in this version of MPC
%% Set N-steps prediction
N = 10;
% transform nan into a large number
for m=1:21
    if isnan(laser(m,1))
        laser(m,1)=20;
    end
end
%% The state of each step is in polar coordinates of the robot(r,theta)
t = sdpvar(1,N+1); % theta, relative to the robot
r = sdpvar(1,N+1); % radius, relative to the robot

x=pose(1);
y=pose(2);
beta=pose(3);
%% Define objective function
J = 0;
Constr = [];
threshold=10; % threshold distance to consider an obstacle
safety=8; % safety distance for the farthest time step to land,<threshold
%% Define Constraints
for k = 1:N
    % take the 4 central sectors as the prior sector, laser No.9-13
    % if no obstacle exists in prior sector, set it as the feasible set
    if all(laser(9:13,1))>threshold
        Constr = [Constr laser(9,2)<=t(k)<=laser(13,2) 0<=r(k)<=safety];
    else
        % find the safest sector, where obstacle is assumedly the farthest
        [max_range inx]=max(laser(:,1));
        % update safety distance
        if max_range<safety
            safety=max_range;
        end
        % locate its sector, and assign the new feasible set to that sector
        if 1<=inx<=4
            Constr = [Constr laser(1,2)<=t(k)<=laser(4,2) 0<=r(k)<=safety];
        elseif 5<=inx<=8
            Constr = [Constr laser(5,2)<=t(k)<=laser(8,2) 0<=r(k)<=safety];
        elseif 14<=inx<=17
            Constr = [Constr laser(14,2)<=t(k)<=laser(17,2) 0<=r(k)<=safety];
        elseif 18<=inx<=21
            Constr = [Constr laser(18,2)<=t(k)<=laser(21,2) 0<=r(k)<=safety];
        end
    end
    % calculate absolute (x,y) of each time step
    z(k,:)=[x+r(k).*cos(t(k)+beta) y+r(k).*sin(t(k)+beta)];
    Contstr = [Constr t(1)==0 r(1)==0.1];
    J = J + (z(k,1) - z_ref(1))^2 + (z(k,2) - z_ref(2))^2;   
end

%% Using Optimize
% Set options for YALMIP and solver
% Choose maximum iterations to 350 to speed up
% Choose algorithm ('trust-region-reflective','sqp','sqp-legacy','interior-point')
options = sdpsettings('verbose',1,'solver','fmincon','fmincon.maxit',350, ...
    'fmincon.algorithm','interior-point');
sol = optimize(Constr, J, options);

%% Get Optimized Input
tOpt = double(t)
rOpt = double(r)
zOpt = [];
for i=1:N
    zOpt = [zOpt;x+rOpt(i).*cos(tOpt(i)+beta) y+rOpt(i).*sin(tOpt(i)+beta)];
end
plan_path = zOpt