function Results_JIRS2016(t,bd,b,bt,vt,R,Rd,Q,fm,tau, Omd, Om)


%l=2;
figure
plot3(bd(1,:),bd(2,:),bd(3,:),'k','LineWidth',1.5,'LineStyle',':');
hold on
plot3(b(1,:),b(2,:),b(3,:),'m','LineWidth',1.5);
grid on
%camproj perspective, rotate3d on
xlabel('$X\ \mathrm{(m)}$','interpreter','latex','Fontsize',16)
ylabel('$Y\ \mathrm{(m)}$','interpreter','latex','Fontsize',16)
zlabel('$Z\ \mathrm{(m)}$','interpreter','latex','Fontsize',16)
%axis([-l l -l l -l l])
%set(gcf,'Position',[100 500 500 220])
%axis equal
ax=0.15; % Body axes length

 for m=1:100:length(t);
    plot3([b(1,m) ax*R(1,1,m)+b(1,m)], [b(2,m) -ax*R(2,1,m)+b(2,m)], [b(3,m) -ax*R(3,1,m)+b(3,m)] ,'g','linewidth',3);
    plot3([b(1,m) ax*R(1,2,m)+b(1,m)], [b(2,m) -ax*R(2,2,m)+b(2,m)], [b(3,m) -ax*R(3,2,m)+b(3,m)] ,'b','linewidth',3);
    plot3([b(1,m) ax*R(1,3,m)+b(1,m)], [b(2,m) -ax*R(2,3,m)+b(2,m)], [b(3,m) -ax*R(3,3,m)+b(3,m)] ,'r','linewidth',3);
    hold on
 end
    locs = axis; % get current axis boundaries
        hold on;
        plot3([0 locs(2)], [0 0], [0 0],'g','LineWidth',2);
        plot3([0 0], [0 locs(4)], [0 0],'b','LineWidth',2);
        plot3([0 0], [0 0], [0 locs(6)],'r','LineWidth',2);
        plot3([locs(1) 0], [0 0], [0 0],'g--','LineWidth',2);
        plot3([0 0], [locs(3) 0], [0 0],'b--','LineWidth',2);
        plot3([0 0], [0 0], [locs(5) 0],'r--','LineWidth',2);
az = 139;
el = 24;
legend('$b_d$ desired trajectory','$b$ achieved trajectory','Orientation','vertical','Location','NorthWest');
set(legend, 'Box', 'off')
h = legend;
set(h, 'interpreter', 'latex','fontsize',14)
view(az, el);

figure
plot(t,bt,'LineWidth',2);
xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
ylabel('$\tilde{b}$ (m)','interpreter','latex','fontsize',18);
legend('$\tilde{b}_{x}$','$\tilde{b}_{y}$','$\tilde{b}_{z}$','Orientation','horizontal','Location','SouthEast');
set(legend, 'Box', 'off')
h = legend;
set(gcf,'Position',[100 500 500 220])
set(h, 'interpreter', 'latex','fontsize',18)
%title('Control Torque','interpreter', 'latex','fontsize',18)
grid on

figure
plot(t,vt,'LineWidth',2);
xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
ylabel('$\tilde{v}$ (m/s)','interpreter','latex','fontsize',18);
legend('$\tilde{v}_{x}$','$\tilde{v}_{y}$','$\tilde{v}_{z}$','Orientation','horizontal','Location','NorthEast');
set(legend, 'Box', 'off')
h = legend;
set(gcf,'Position',[100 500 500 220])
set(h, 'interpreter', 'latex','fontsize',18)
%title('Control Torque','interpreter', 'latex','fontsize',18)
grid on





figure
plot(t(1:length(t)-1),fm,'LineWidth',2);
xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
ylabel('$f$ (N)','interpreter','latex','fontsize',18);
%legend('$\tilde{v}_{x}$','$\tilde{v}_{y}$','$\tilde{v}_{z}$','Orientation','horizontal','Location','NorthEast');
set(legend, 'Box', 'off')
h = legend;
set(gcf,'Position',[100 500 500 220])
set(h, 'interpreter', 'latex','fontsize',18)
%title('Control Torque','interpreter', 'latex','fontsize',18)
grid on



% l=length(t);
% 
% for k = 1:l;
% nbd(k)=norm(bd(:,k));
% nb(k)=norm(b(:,k));
% nbt(k)=norm(bt(:,k));
% nvt(k)=norm(vt(:,k));
% nOm(k)=norm(Om(:,k));
% om(:,k)=norm(Om(:,k)-(Rd(:,:,k)'*R(:,:,k))*Omd(:,k));
% end
% 
% figure
% plot(t,om,'LineWidth',2);
% xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
% ylabel('$\omega$ (rad/s)','interpreter','latex','fontsize',18);
% %legend('$\tilde{v}_{x}$','$\tilde{v}_{y}$','$\tilde{v}_{z}$','Orientation','horizontal','Location','NorthEast');
% set(legend, 'Box', 'off')
% h = legend;
% set(gcf,'Position',[100 500 500 220])
% set(h, 'interpreter', 'latex','fontsize',18)
% %title('Control Torque','interpreter', 'latex','fontsize',18)
% grid on
% 
% figure
% plot(t,nbt,'LineWidth',2);
% xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
% ylabel('$\|\tilde{b}\|$ (m)','interpreter','latex','fontsize',18);
% %legend('$\tau_{x}$','$\tau_{y}$','$\tau_{z}$','Orientation','horizontal','Location','NorthEast');
% set(legend, 'Box', 'off')
% h = legend;
% set(gcf,'Position',[100 500 500 220])
% set(h, 'interpreter', 'latex','fontsize',18)
% %title('Control Torque','interpreter', 'latex','fontsize',18)
% grid on
% 
% figure
% plot(t,nvt,'LineWidth',2);
% xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
% ylabel('$\|\tilde{v}\|$ (m/s)','interpreter','latex','fontsize',18);
% %legend('$\tau_{x}$','$\tau_{y}$','$\tau_{z}$','Orientation','horizontal','Location','NorthEast');
% set(legend, 'Box', 'off')
% h = legend;
% set(gcf,'Position',[100 500 500 220])
% set(h, 'interpreter', 'latex','fontsize',18)
% %title('Control Torque','interpreter', 'latex','fontsize',18)
% grid on
% 
% figure
% plot(t,nOm,'LineWidth',2);
% xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
% ylabel('$\|\Omega\|$ (rad/s)','interpreter','latex','fontsize',18);
% %legend('$\tau_{x}$','$\tau_{y}$','$\tau_{z}$','Orientation','horizontal','Location','NorthEast');
% set(legend, 'Box', 'off')
% h = legend;
% set(gcf,'Position',[100 500 500 220])
% set(h, 'interpreter', 'latex','fontsize',18)
% %title('Control Torque','interpreter', 'latex','fontsize',18)
% grid on


%
n=length(t)-1;
for k = 1:n;
Phi(k)= norm(logmso3(Q(:,:,k)));
ntau(k)=norm(tau(:,k));
end

% figure
% plot(t(1:n),Phi,'Linewidth',2);
% xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
% ylabel('$\|\Phi\|$ rad','interpreter','latex','fontsize',18);
% %legend('$\tau_{x}$','$\tau_{y}$','$\tau_{z}$','Orientation','horizontal','Location','NorthEast');
% set(legend, 'Box', 'off')
% h = legend;
% set(gcf,'Position',[100 500 500 220])
% set(h, 'interpreter', 'latex','fontsize',18)
% %title('Control Torque','interpreter', 'latex','fontsize',18)
% grid on
figure
plot(t(1:n),ntau,'LineWidth',2);
xlabel('$t$ (s)','interpreter', 'latex','fontsize',18); 
ylabel('$\|\tau\|$ (Nm)','interpreter','latex','fontsize',18);
%legend('$\tau_{x}$','$\tau_{y}$','$\tau_{z}$','Orientation','horizontal','Location','NorthEast');
set(legend, 'Box', 'off')
h = legend;
set(gcf,'Position',[100 500 500 220])
set(h, 'interpreter', 'latex','fontsize',18)
%title('Control Torque','interpreter', 'latex','fontsize',18)
grid on

