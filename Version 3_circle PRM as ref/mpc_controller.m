function [plan_path,sol] = mpc_controller(z0Bar,z_ref,obs_ref)
%% Set N-steps prediction
N = 5;

%% The state is 2DOF (x,y), r is soft constraint
z = sdpvar(2,N+1);
r = sdpvar(1,N);

%% Assign the initial states
z0Bar = z0Bar(1:3)';

%% Define objective function
J = 0;
Constr = [];

%% Define Constraints
for k = 1:N
     Constr = [Constr 0 <= r(1,k) <= 0.05];
     Constr = [Constr ...
         ((z(1,k+1) - z(1,k))^2 + (z(2,k+1) - z(2,k))^2) <= r(1,k)];
            % Obstacle avoidance
             for l = 1:size(obs_ref,1)
                Constr = [Constr ...
                ((z(1,k) - obs_ref(l,1))^2 + (z(2,k) - obs_ref(l,2))^2) >= 0.4];
             end
             % Seems to fail to avoid
     J = J + r(1,k)^2 + (z(1,k) - z_ref(1))^2 + (z(2,k) - z_ref(2))^2;   
end
J = J + 10*(z(1,N+1) - z_ref(1))^2 + (z(2,N+1) - z_ref(2))^2;

% Control the look ahead path planing
% To follow PRM global obstacle-free path direction
Q = [cos(pi/2-z0Bar(3)) sin(pi/2-z0Bar(3))];
if z0Bar(3) > 0
    for i=1:N
        y=(z(2,i+1) - z0Bar(2));
        x=(z(1,i+1) - z0Bar(1));
        P=[x y];
        Constr = [Constr P(1)*Q(2) - Q(1)*P(2) <= 0];
    end
else
    for i=1:N
        y = (z(2,i+1) - z0Bar(2));
        x = (z(1,i+1) - z0Bar(1));
        P = [x y];
        Constr = [Constr P(1)*Q(2) - Q(1)*P(2) >= 0];
    end
end
% Seems fail to achieve
% The math is followed by 
% http://blog.csdn.net/modiz/article/details/9928553

% Initial State set to current pose
Constr = [Constr z(:,1) == z0Bar(1:2)];

%% Using Optimize
% Set options for YALMIP and solver
% Choose maximum iterations to 350 to speed up
% Choose algorithm ('trust-region-reflective','sqp','sqp-legacy','interior-point')
options = sdpsettings('verbose',0,'solver','fmincon','fmincon.maxit',350, ...
    'fmincon.algorithm','interior-point');
sol = optimize(Constr, J, options);

%% Get Optimized Input
zOpt = double(z)';
plan_path = zOpt(1:end,1:2);