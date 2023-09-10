clc
clear all;
close all;
tc=cputime;

h=0.02;
tf=100;
t0=0;

Delt=tf-t0;                 % length of time interval; assumes tf > t0
n=fix(Delt/h);

%initialize all values using zeros outside the loop. Increases efficiency
%and reduces runtime
t(1)=0;
%t=zeros(tf/h);
phi_d(1:3,1:tf/h)=zeros;
tau_d(1:3,1:tf/h)=zeros;
Xi_d(1:6,1:tf/h)=zeros;
e_k(1:6,1:tf/h)=zeros;
Xi_d_hat(1:6,1:tf/h)=zeros;

for k=1:n-1
    t(k+1)=t(k)+h;
    a1=0.3*(0.5*sin(10*t(k))+5*(1-cos(t(k)))+3.5*sin(3*(t(k)))+2.5*cos(5*t(k)));
    a2=0.3*(4.5*(1-cos(2*t(k)))+sin(8*t(k))+5*(sin(t(k)))^2);
    a3=0.5*(4*sin(3*t(k))+0.5*sin(10*t(k)));
    
    b1=0.1*(6*(sin(t(k)))^2+(1-cos(7*t(k)))+4*(sin(2*(t(k)))));
    b2=0.035*(6*(1-cos(t(k)))^2+cos(5*t(k))+3*sin(3*(t(k))));
    b3= 0.2*5*sin(t(k));
    
    phi_d_act(:,k)=[a1 a2 a3]';
    tau_d_act(:,k)=[b1 b2 b3]';
    
    phi_d(:,k)= [a1 a2 a3]';                %[sin(10*t(k))+10*sin(t(k)) cos(10*t(k))+10*cos(t(k)) 10*cos(t(k)+pi/2)]';
    tau_d(:,k)= [b1 b2 b3]';                %[sin(10*t(k)+pi)+10*sin(t(k)) cos(10*t(k)-pi/4)+10*cos(t(k)-pi/4) 9*cos(t(k)+pi/4)]';
    Xi_d(:,k)=[phi_d(:,k); tau_d(:,k)];
    Xi_d_hat(:,1)=Xi_d(:,1)+[randbump(1,3,1);randbump(0.2,3,1)];
    e_k(:,k)=Xi_d_hat(:,k)-Xi_d(:,k);
    e_k_k=e_k(:,k);
    Xi_d_k=Xi_d(:,k);
    Xi_d_hat_k_1=Holder_fn(e_k_k,Xi_d_k);
    Xi_d_hat(:,k+1)= Xi_d_hat_k_1;
    e_k_plot(k)=[[e_k(:,k)]'*e_k(:,k)]^0.5;
    
end
k=n;

a1_n=0.3*0.5*(sin(10*t(n))+10*(1-cos(t(n)))+7*sin(3*(t(n)))+5*cos(5*t(n)));
a2_n=0.3*0.5*(9*(1-cos(2*t(n)))+2*sin(8*t(n))+10*(sin(t(n)))^2);
a3_n=0.5*0.5*(8*sin(3*t(n))+sin(10*t(n)));
    
b1_n=0.1*(6*(sin(t(n)))^2+(1-cos(7*t(n)))+4*(sin(2*(t(n)))));
b2_n=0.035*(6*(1-cos(t(n)))^2+cos(5*t(n))+3*sin(3*(t(n))));
b3_n= 0.2*5*sin(t(n));

Xi_d(:,1:n-1)=[phi_d_act(:,1:n-1); tau_d_act(:,1:n-1)];
e_k(:,1:n-1)=Xi_d_hat(:,1:n-1)-Xi_d(:,1:n-1);

phi_d(:,n)= [a1_n a2_n a3_n]';                %[sin(10*t(n))+10*sin(t(n)) cos(10*t(n))+10*cos(t(n)) 10*cos(t(n)+pi/2)]';
tau_d(:,n)= [b1_n b2_n b3_n]';                                  %[sin(10*t(n)+pi)+10*sin(t(n)) cos(10*t(n)-pi/4)+10*cos(t(n)-pi/4) 9*cos(t(n)+pi/4)]';
Xi_d(:,n)=[phi_d(:,n);tau_d(:,n)];

phi_d_total(:)=(((Xi_d(1,:)).^2)+((Xi_d(2,:)).^2)+((Xi_d(3,:)).^2)).^0.5;
phi_d_hat_total(:)=(((Xi_d_hat(1,:)).^2)+((Xi_d_hat(2,:)).^2)+((Xi_d_hat(3,:)).^2)).^0.5;

tau_d_total(:)=(((Xi_d(4,:)).^2)+((Xi_d(5,:)).^2)+((Xi_d(6,:)).^2)).^0.5;
tau_d_hat_total(:)=(((Xi_d_hat(4,:)).^2)+((Xi_d_hat(5,:)).^2)+((Xi_d_hat(6,:)).^2)).^0.5;

e_k_phi(:)=phi_d_hat_total(:)-phi_d_total(:);
e_k_tau(:)=tau_d_hat_total(:)-tau_d_total(:);

e_k(:,n)=Xi_d_hat(:,n)-Xi_d(:,n);
e_k_plot(:,n)=[e_k(:,n)'*e_k(:,n)]^0.5;

for k=1:n-1
    
diff1(:,k)=abs(Xi_d(1,k+1)-Xi_d(1,k));
diff2(:,k)=abs(Xi_d(2,k+1)-Xi_d(2,k));
diff3(:,k)=abs(Xi_d(3,k+1)-Xi_d(3,k));
diff(:,k)=sqrt((diff1(:,k)^2)+(diff1(:,k)^2)+(diff3(:,k)^2));

end

Max_k=max(diff);
Min_k=min(diff);

disp(cputime-tc);

figure
plot(t,[e_k_phi(:)],'LineWidth',2);
xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
ylabel('$e^\phi_k$ ','interpreter','latex','fontsize',18);
grid on

figure
plot(t,[e_k_tau(:)],'LineWidth',2);
xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
ylabel('$e^\tau_k$ ','interpreter','latex','fontsize',18);
grid on

% figure
% plot(t,[e_k(1,:)],'LineWidth',2);
% xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
% ylabel('f_d_x','interpreter','latex','fontsize',18);
% grid on
% 
% figure
% plot(t,[e_k(2,:)],'LineWidth',2);
% xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
% ylabel('f_d_y','interpreter','latex','fontsize',18);
% grid on
% 
% figure
% plot(t,[e_k(3,:)],'LineWidth',2);
% xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
% ylabel('f_d_z','interpreter','latex','fontsize',18);
% grid on
% 
% figure
% plot(t,[e_k(4,:)],'LineWidth',2);
% xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
% ylabel('tau_d_x','interpreter','latex','fontsize',18);
% grid on
% 
% figure
% plot(t,[e_k(5,:)],'LineWidth',2);
% xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
% ylabel('tau_d_y','interpreter','latex','fontsize',18);
% grid on
% 
% figure
% plot(t,[e_k(6,:)],'LineWidth',2);
% xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
% ylabel('tau_d_z','interpreter','latex','fontsize',18);
% grid on
% 
% 
% figure
% plot(t,e_k_plot,'LineWidth',2);
% xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
% ylabel("sqrt(e.e')",'interpreter','latex','fontsize',18);
% grid on







