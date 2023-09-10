function fitt=fiteval(a)

global J m M e1 e2 e3 P L La g p ki kp 

h=.01; % Step size
tf=5; % Final time for simulation
t0=0;
%t=t0:h:tf;


% UAV parameters );         % kgm^2; UAV inertia; 3 x 3 matrix
% m=4.34;                      
% J=diag([0.0820,0.0845,0.1377]           % kg, mass of UAV
J = [-0.71 0.71 0; -0.71 -0.71 0; 0 0 1];
m = 0.5;

M=m*eye(3);                             % 
g=9.81;
% Initialization
R0=expmso3([0;0;0]);                    % initial attitude
%R0=[1 0 0;0 -1 0;0 0 -1];
b0=[1;0;0];                             % initial position
% g0=[R0 b0;0 0 0 1];                   % initial pose   
Om0=[0 0 0]';                           % initial angular velocity
nu0=[0 0 0]';                           % initial translational velocity

% xi0=[Om0;nu0];

e1=[1;0;0;];e2=[0;1;0];e3=[0;0;1];


% Generate smooth trajectory through given finite set of desired waypoints
% waypoints=[0 0 0;1 1 1];
% 
% [bb,vd,dvd]=waypoints_traj_gen(t,T,waypoints);

%vd0=[0.4;0.4*pi;0];
%dvd0=[0;0;-0.6*pi^2*1];

vd0=[0;0;0];
dvd0=[0;0;0];

%b0=[0 0 0]';

Om0=[0;0;0];

% Gain for s_d as per Proposition 1
%%%% use function desired_attitude in line 86 of LGVI_SE3_UAV

% P=28*eye(3);
% L=18*eye(3);
% La=6.5*eye(3);
% p=2;
% ki =0.02;%0.098;%3
% kp = 4.5;%>1


P = a(1)*eye(3);
L = a(2)*eye(3);
La = a(3)*eye(3);
p = a(4);
ki = a(5);
kp =a(6);


% lb = [0 0 0 1 0 0];
% ub = [10 10 10 2 10 10];

% P=38*eye(3);
% L=25*eye(3);
% La=3.5*eye(3);
% p=0.75;
% ki =0.04;%0.098;%3
% kp = 4.5;%>1

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

% Gain for s_d as per Proposition 2
%%%% use function desired_attitude_dot in line 86 of LGVI_SE3_UAV

% P=25*eye(3);
% 
% L=12.5*eye(3);
% 
% La=4*eye(3);
% 
% p=1.1;
% 
% ki =1.5;%0.098;%3
% kp = 5.5;%>1

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[t,R,Rd,b,fm,tau,nu,Omd,Om,Q,bd,vd,dvd,bt,vt] = LGVI_SE3_UAV(b0,R0,nu0,Om0,dvd0,t0,tf,h);

fitt=ones(1,3)*bt(:,end-200:end)'*ones(3,1)+ ones(1,3)*vt(:,end-200:end)'*ones(3,1);
