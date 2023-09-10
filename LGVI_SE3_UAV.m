function [t,R,Rd,b,fm,tau,nu,Omd,Om,Q,bd,vd,dvd,bt,vt] = LGVI_SE3_UAV(b0,R0,nu0,Om0,dvd0,t0,tf,h)

global J M m e3 g 

b(:,1)=b0;
R(:,:,1)=R0;
nu(:,1)=[0;0;0];
Om(:,1)=[0;0;0];
Omd(:,1)=Om0;
%Rd(:,:,1)=R0;



Delt=tf-t0;                 % length of time interval; assumes tf > t0
n=fix(Delt/h);

t(1) = 0;
bd(:,1)=[0;0.6;0];
vd(:,1)=[.4*pi;0;.4];
dvd(:,1)=[0;-.6*pi^2;0];


% t = linspace(t0,tf,tf/h);
t(1) = 0;
bd=[0.4*sin(pi*t);0.6*cos(pi*t);0.4*t];
vd=[0.4*pi*cos(pi*t);-0.6*pi*sin(pi*t);0.4*ones(size(t))];
dvd=[-0.4*pi^2*sin(pi*t);-0.6*pi^2*cos(pi*t); 0*ones(size(t))];

bt(:,1)=b0-bd(:,1);
vt(:,1)=R(:,:,1)*nu(:,1)-vd(:,1);
Rd(:,:,1)=desired_attitude(bt(:,1),vt(:,1),dvd(:,1));

%r1d(:,1)=[1;0;0];
for k=1:n-1
    t(k+1)=t(k)+h;
    k
    
%  bd(:,k+1)=[0.4*t(k+1);0.4*sin(pi*t(k+1));0.6*cos(pi*t(k+1))];
%  vd(:,k+1)=[0.4;0.4*pi*cos(pi*t(k+1));-0.6*pi*sin(pi*t(k+1))];
%  dvd(:,k+1)=[0;-0.4*pi^2*sin(pi*t(k+1));-0.6*pi^2*cos(pi*t(k+1))];
 
%  r1d(:,k+1)=[cos(pi*t(k+1));sin(pi*t(k+1));0];
%  bd=[0.4*t;0.4*sin(pi*t);0.6*cos(pi*t)] 
%  vd(:,k+1)=(1/h)*(bd(:,k+1)-bd(:,k));
%  dvd(:,k+1)=(1/h)*(vd(:,k+1)-vd(:,k));
%  vd=[0.4*ones(size(t));0.4*pi*cos(pi*t);-0.6*pi*sin(pi*t)];
 

%# Z motion waypoints   
bd(:,k+1)=[0.4*sin(pi*t(k+1));0.6*cos(pi*t(k+1));0.4*t(k+1)];
vd(:,k+1)=[0.4*pi*cos(pi*t(k+1));-0.6*pi*sin(pi*t(k+1));0.4];
dvd(:,k+1)=[-0.4*pi^2*sin(pi*t(k+1));-0.6*pi^2*cos(pi*t(k+1));0];

%dvd(:,k+1)=[0;0;0];
%      bd(:,k+1)=[0.4*sin(pi*t(k+1));0.6*cos(pi*t(k+1));0.4*t(k+1)];
%      vd(:,k+1)=[0.4*pi*cos(pi*t(k+1));-0.6*pi*sin(pi*t(k+1));0.4];
%      dvd(:,k+1)=[-0.4*pi^2*sin(pi*t(k+1));-0.6*pi^2*cos(pi*t(k+1));0];
    
   
%     bd(:,k)=[0.4*t;0.4*sin(pi*t);0.6*cos(pi*t)];
%     vd(:,k)=[0.4*ones(length(t));0.4*pi*cos(pi*t);-0.6*pi*sin(pi*t)];
%     dvd(:,k)=[zeros(length(t));-0.4*pi^2*sin(pi*t);-0.6*pi^2*cos(pi*t)]; 
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
   % Solving the implicit equation of the kinematic eq's
    fi=h*Om(:,k);
%     nf=norm(fi);
%     G=sin(nf)/nf*J*fi+(1-cos(nf))/nf^2*cross(fi,J*fi);
%     while norm(G-h*J*Om(:,k)) > 1e-15
%         nf=norm(fi);
%         G=sin(nf)/nf*J*fi+(1-cos(nf))/nf^2*cross(fi,J*fi);
%         nabG=(cos(nf)*nf-sin(nf))/nf^3*(J*fi)*fi'+sin(nf)/nf*J...
%             +(sin(nf)*nf-2*(1-cos(nf)))/nf^4*cross(fi,J*fi)*fi'...
%             +(1-cos(nf))/nf^2*(skew(-J*fi)+[cross(fi,J(:,1)),cross(fi,J(:,2)),cross(fi,J(:,3))]);
%         fi=fi+inv(nabG)*(h*J*Om(:,k)-G);
%     end
    F(:,:,k)=expmso3(fi);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    R(:,:,k+1)=R(:,:,k)*F(:,:,k);       % next attitude
    b(:,k+1)=h*R(:,:,k)*nu(:,k)+b(:,k); % the next position
    
    % The control torque:
    bt(:,k+1)=b(:,k+1)-bd(:,k+1);
    fm(k)=trans_control_f(R(:,:,k),bt(:,k),nu(:,k),vd(:,k),dvd(:,k));
    
    nu(:,k+1)=M\(F(:,:,k)'*M*nu(:,k)+h*m*g*R(:,:,k+1)'*e3-h*fm(k)*e3);
   

    v(:,k+1)=R(:,:,k+1)*nu(:,k+1);
    vt(:,k+1)=v(:,k+1)-vd(:,k+1);
    
    Rd(:,:,k+1)=desired_attitude(bt(:,k+1),vt(:,k+1),dvd(:,k+1));
    
    Fd(:,:,k+1)=Rd(:,:,k)'*Rd(:,:,k+1);
    
    Omd(:,k+1)=(1/h)*(logmso3(Fd(:,:,k+1)));
    
    dOmd(:,k)=(1/h)*(Omd(:,k+1)-Omd(:,k));
    
    Q(:,:,k)=Rd(:,:,k)'*R(:,:,k);
    
    %om(:,k)=Om(:,k)-Q(:,:,k)'*Omd(:,k)
   
    tau(:,k)=attitude_control_tau(Om(:,k),Omd(:,k),dOmd(:,k),Q(:,:,k));
        
    Om(:,k+1)=J\((F(:,:,k)'*J*Om(:,k))+h*tau(:,k));
    
end


