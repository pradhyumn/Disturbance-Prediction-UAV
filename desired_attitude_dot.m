% Generating desired attitude trajectory, Rd
function Rd=desired_attitude_dot(bt,vt,dvd)
global P L m g e1 e3


%v=R*nu;
%bt=b-bd;
% vt=v-vd;


r3d=(m*g*e3+P*bt+L*vt-m*dvd)/norm(m*g*e3+P*bt+L*vt-m*dvd);

%a=r3d/norm(r3d);
sd=[r3d(2)+r3d(3) r3d(3)-r3d(1) -r3d(1)-r3d(2)]';
% nsd=norm(sd1)
% %sd=[a(2)+a(3) a(3)-a(1) -a(1)-a(2)]';
% 
 %mu=4;
 %sd=cross(ones(3,1),r3d)+mu*e1;
%nsd1=norm(sd1)

%dot=sd'*r3d
r2d=cross(r3d,sd)/norm(cross(r3d,sd));

Rd=[cross(r2d,r3d) r2d r3d];
%r1d=[1 0 0]';
%Rd=[r1d cross(r3d,r1d) r3d];