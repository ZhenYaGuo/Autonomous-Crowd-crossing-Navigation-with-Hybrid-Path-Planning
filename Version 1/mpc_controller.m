function [plan_path]=mpc_controller(z0Bar,z_ref,o_data)
%% Set N and Final
N=20;
Ts=0.1;
%% the state is 3DOF, input is 2DOF
z=sdpvar(3,N+1);
u=sdpvar(2,N);

%% Assign the inisial state;
z0Bar=z0Bar';
%% Define Object function.
J=0;
%% Define Dynamic NLP system
f=@(z,u) [z(1)+Ts*u(1)*cos(z(3));...
          z(2)+Ts*u(1)*sin(z(3));...
          z(3)+u(2)*Ts];


%% Define Constrain
Constr=[];
for k=1:N
    Constr=[Constr z(:,k+1)==f(z(:,k),u(:,k))];
    J=J+100*(z(1,k)-z_ref(1))^2+100*(z(2,k)-z_ref(2))^2;
   
    % Obstabel avoid

end

% Obstabel avoid
if size(o_data,1)~=0
    for n=1:20
        for l=1:size(o_data,1)
                Constr = [Constr ...
                ((z(1,n)-o_data(l,1))^2+(z(2,n)-o_data(l,2))^2)>=0.01];
        
        end
    end
end



% input Bounded Constrain
for i=1:N
    Constr=[Constr [0.8;-pi]<=u(:,i)<=[0.8;pi]];
end

%State Bounded Constrain
%for i=1:N
%    Constr=[Constr [-20;-20;-2*pi]<=z(:,3)<=[20;20;2*pi]];
%end

%Final State Constrain

Constr=[Constr z(:,N+1)==f(z(:,N),u(:,N))];

%inital State
Constr=[Constr z(:,1)==z0Bar];

%% Using Opimize
% Set options for YALMIP and solver
options = sdpsettings('verbose',1,'solver','fmincon','usex0',1,'fmincon.maxit',200);
sol = optimize(Constr, J, options);

%% Get Optimized Input
zOpt=double(z)';
uOpt=double(u);
v=uOpt(1,1);
omega=uOpt(2,1);
z_next=zOpt(:,2);
plan_path=zOpt(1:N+1,1:2);



end