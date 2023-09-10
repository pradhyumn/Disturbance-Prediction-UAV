% Generating desired attitude trajectory, Rd
function Rd=desired_attitude(bt,vt,dvd)
global P L m g e1 e3


%v=R*nu;
%bt=b-bd;
% vt=v-vd;


r3d=(m*g*e3+P*bt+L*vt-m*dvd)/norm(m*g*e3+P*bt+L*vt-m*dvd);

% A=2;
%  
% B=(r3d(1)-r3d(2))*(1+(r3d(3)/(r3d(1)+r3d(2))));
% 
% if A ~= B
%     mu=A;
% else
%     mu=B+1;
% end

% B=(r3d(1)-r3d(2))*(((r3d(3)/((r3d(1)+r3d(2))))-1));
% 
% if abs(B)>3
%     mu=B;
% else
%     mu=B+5;
% end
 mu=4;
sd=cross(ones(3,1),r3d)+mu*e1;



r2d=cross(r3d,sd)/norm(cross(r3d,sd));

Rd=[cross(r2d,r3d) r2d r3d];