clear;
clc;
close all;

%Initial States

%Defense
x_0_a1 = [0,0,0,0]; %unicycle [x,y,rho,theta]

%Intruder
x_0_d = [2,-3,pi/2];

%Define Time
timesteps = 120;
starttime = 0;
endtime = 13;
t = linspace(starttime,endtime,timesteps); %Duration
t_total = endtime - starttime;

%Settings
Settings = 0;

%Choose beta (Inverse Optimality)
beta = 1;

%Choose gamma alpha(h(x)) = gamma*h(x)
gamma = 1;

%Defense activation radius
r = 2;

Params = [beta,gamma,r]; %Parameters

index = 2;

%% Sim

x_a1 = zeros(length(t),index);
y_a1 = zeros(length(t),index);
rho_a1 = x_a1;
theta_a1 = x_a1;

x_d = x_a1;
y_d = x_a1;
theta_d = x_a1;

u1 = x_a1;
u2 = x_a1;
u3 = x_a1;
u4 = x_a1;

% x_0_ref = x_0_a1;

for i = 1:index
    x_initial = [x_0_a1, x_0_d];
    [T,X] = ode89(@(T,X)eom(T,X,Params,x_initial,i),t,x_initial);

    x_a1(:,i) = X(:,1);
    y_a1(:,i) = X(:,2);
    rho_a1(:,i) = X(:,3);
    theta_a1(:,i) = X(:,4);
    x_d(:,i) = X(:,5);
    y_d(:,i) = X(:,6);
    theta_d(:,i) = X(:,7);

    %Control Input
    for j = 1:length(t)
        [~,Uv,Uw,U0_v,U0_w] = eom(T(j),X(j,:),Params,x_initial,i);
        u1(j,i) = Uv;
        u2(j,i) = Uw;
        u3(j,i) = U0_v;
        u4(j,i) = U0_w;
    end
    
end

% Setting up the Plot
figure; hold on

% title(sprintf('Trajectory ($\\theta_{0} = 0$) \nTime: %0.2f sec', t(1)), 'Interpreter', 'Latex');

xlabel('$x$','Interpreter', 'Latex')
ylabel('$y$','Interpreter', 'Latex')
grid minor  % Adding grid lines
xlim([-4,4]);
ylim([-4,4]);
daspect([1 1 1])  % Equal axis aspect ratio

%Plot Wall
xL = xlim;
yL = ylim;

x_cont = linspace(xL(1),xL(2),300);
y_cont = linspace(yL(1),yL(2),300);

% %Contour
% x_d = linspace(xL(1),xL(2),300);
% y_d = linspace(yL(1),yL(2),300);

% line([-d,-d],yL,'Color','black');
% line([d,d],yL,'Color','black');
line(xL,[0,0],'Color','black','LineStyle','--');
% line(xL,[-1.5,-1.5],'Color','black','LineStyle','--');
% line([-d_delta,-d_delta],yL,'Color','black','LineStyle','-.');
% line([d_delta,d_delta],yL,'Color','black','LineStyle','-.');

%Color array
% color = ['b','r','g','m'];
% Plotting the first iteration

% Create file name variable
filename = 'unknown_obst_uni_2.gif';

leg = string.empty; 

% for i = 1:1 %Number of entries before
%     leg(i) = '';
% end

leg(1) = '';
leg(2) = '';
leg(3) = 'Agent (Robust CBF)';
leg(4) = '';
leg(5) = '';
leg(6) = '';
leg(7) = 'Agent (Standard CBF)';
leg(8) = '';
leg(9) = 'Obstacle';

color = ['b','r'];
 
for i = 1:index

    if i >= 2
        delete(circ_a1);
        delete(circ_d);
    end
    
    p_a1(i) = plot(x_a1(1,i),y_a1(1,i),color(i),'linewidth',2.5);
    m_a1(i) = scatter(x_a1(1,i),y_a1(1,i),50,'filled',color(i));
    circ_a1(i) = circle(x_a1(1,i),y_a1(1,i),r/2,color(i));
    
    p_d(i) = plot(x_d(1,i),y_d(1,i),'k','linewidth',2.5);
    m_d(i) = scatter(x_d(1,i),y_d(1,i),50,'filled','k');
    circ_d = circle(x_d(1,i),y_d(1,i),r/2,'k');

%    k = 1;
%    z2 = zeros(length(x_cont),length(y_cont));
%    lam = 10;
%    a = 7;
%         
%     for j = 1:length(x_cont)
%         for l = 1:length(y_cont)
%             x_h = (x_cont(j)-x_d(k))*cos(theta_d(k)) + (y_cont(l)-y_d(k))*sin(theta_d(k));
%             y_h = -(x_cont(j)-x_d(k))*sin(theta_d(k)) + (y_cont(l)-y_d(k))*cos(theta_d(k));
%         
%             z2(j,l) = y_h^2  + (x_h^2)/(1 + a/(1+ exp(lam*x_h))) - r^2;
%         end
%     end
% 
%     [C2,H2] = contour(x_cont,y_cont,z2(:,:)',[0,0],'color','r','linewidth',2.5);
end


for k = 1:length(t)
   for i = 1:index
       if k <= 10

       p_a1(i).XData = x_a1(1:k,i);
       p_a1(i).YData = y_a1(1:k,i);
        
       p_d(i).XData = x_d(1:k,i);
       p_d(i).YData = y_d(1:k,i);
       else

       p_a1(i).XData = x_a1(k-10:k,i);
       p_a1(i).YData = y_a1(k-10:k,i);

       p_d(i).XData = x_d(k-10:k,i);
       p_d(i).YData = y_d(k-10:k,i);

       end

       m_a1(i).XData = x_a1(k,i);
       m_a1(i).YData = y_a1(k,i);
       delete(circ_a1(i));
       circ_a1(i) = circle(x_a1(k,i),y_a1(k,i),r/2,color(i));  
       
       m_d(i).XData = x_d(k,i);
       m_d(i).YData = y_d(k,i);

       delete(circ_d);
       circ_d = circle(x_d(k,i),y_d(k,i),r/2,'k');

%        z2 = zeros(length(x_cont),length(y_cont));
%        lam = 10;
%        a = 7;
%             
%         for j = 1:length(x_cont)
%             for l = 1:length(y_cont)
%                 x_h = (x_cont(j)-x_d(k))*cos(theta_d(k)) + (y_cont(l)-y_d(k))*sin(theta_d(k));
%                 y_h = -(x_cont(j)-x_d(k))*sin(theta_d(k)) + (y_cont(l)-y_d(k))*cos(theta_d(k));
%             
%                 z2(j,l) = y_h^2  + (x_h^2)/(1 + a/(1+ exp(lam*x_h))) - r^2;
%             end
%         end
% 
%         delete(H2);
%         [C2,H2] = contour(x_cont,y_cont,z2(:,:)',[0,0],'color','r','linewidth',2.5);
   end

    fontsize(legend(leg, 'Interpreter', 'Latex','location','northwest'),12,'points')
    % Updating the title
%     title(sprintf('Trajectory ($\\theta_{0} = 0 $) \nTime: %0.2f sec'...
%         ,t(k)), 'Interpreter', 'Latex');

    title(sprintf('Trajectory \nTime: %0.2f sec',t(k)), 'Interpreter', 'Latex');
    % Delay
    pause(0.01)

% %     Saving the figure (Uncomment to save .gif file in folder)
% 
%     frame = getframe(gcf);
%     im = frame2im(frame);
%     [imind,cm] = rgb2ind(im,256);
%     if k == 1
%         imwrite(imind,cm,filename,'gif', 'Loopcount',inf,...
%         'DelayTime',0.01);
%     else
%         imwrite(imind,cm,filename,'gif','WriteMode','append',...
%         'DelayTime',0.01);
%     end

end


%%
time_snap = [0, 1.35, 3.1, 5, 8.5, 12];
k = round(time_snap .* timesteps./endtime + 1);

fig = figure;
set(fig, 'Position', [150, 150, 800, 600]);
p = tiledlayout(2,3);
p.TileSpacing = 'compact';
p.Padding = 'compact';
aspectRatio = [2, 3, 1];

for j = 1:length(k)

    nexttile
    hold on
    xlabel('$x$','Interpreter', 'Latex','FontSize',15)
    ylabel('$y$','Interpreter', 'Latex','FontSize',15)
%     grid minor  % Adding grid lines
    xlim([-4,4]);
    ylim([-4,4]);
    daspect([1 1 1])  % Equal axis aspect ratio
    
    leg = string.empty;
    
    leg(1) = '';
    leg(2) = '';
    leg(3) = 'Agent (Robust CBF)';
    leg(4) = '';
    leg(5) = '';
    leg(6) = '';
    leg(7) = '';
    leg(8) = '';
    leg(9) = 'Agent (Standard CBF)';
    leg(10) = '';
    leg(11) = 'Aircraft';
    leg(12) = '';
    leg(13) = '';
    
    q = k(j);

    for i = 1:index
        
        if q == q
    
        plot(x_a1(1:q,i),y_a1(1:q,i),color(i),'linewidth',2.5);
        plot(x_d(1:q,i),y_d(1:q,i),'k','linewidth',2.5);
    
        elseif q >= 20
    
        plot(x_a1(q-20:q,i),y_a1(q-20:q,i),color(i),'linewidth',2.5);
        plot(x_d(q-20:q,i),y_d(q-20:q,i),'k','linewidth',2.5);
    
        end
    
        scatter(x_a1(q,i),y_a1(q,i),50,'filled',color(i));
        circle(x_a1(q,i),y_a1(q,i),r/2,color(i));
        
        scatter(x_d(q,i),y_d(q,i),50,'filled','k');
        circle(x_d(q,i),y_d(q,i),r/2,'k');

%        z2 = zeros(length(x_cont),length(y_cont));
%        lam = 10;
%        a = 7;
%             
%         for j = 1:length(x_cont)
%             for l = 1:length(y_cont)
%                 x_h = (x_cont(j)-x_d(q,i))*cos(theta_d(q,i)) + (y_cont(l)-y_d(q,i))*sin(theta_d(q,i));
%                 y_h = -(x_cont(j)-x_d(q,i))*sin(theta_d(q,i)) + (y_cont(l)-y_d(q,i))*cos(theta_d(q,i));
%             
%                 z2(j,l) = y_h^2  + (x_h^2)/(1 + a/(1+ exp(lam*x_h))) - r/2;
%             end
%         end

%         [C2,H2] = contour(x_cont,y_cont,z2(:,:)',[0,0],'color','k');
    
    end

    plot(x_d(1:end,1),y_d(1:end,1),'k','linewidth',1.5,'LineStyle','--');
    
    fontsize(legend(leg, 'Interpreter', 'Latex','location','best'),12,'points')
    
    title(sprintf('Time: %0.2f sec',t(q)), 'Interpreter', 'Latex','FontSize',16);
    
end
    filename = sprintf('system_snapshot');
    dir = '/Users/zeartful/Desktop/UCSD/Matlab Codes/ACC2025/Collision Avoid/snapshot';
    file = fullfile(dir, filename);
    
%     saveas(gcf,file,'jpg')
%     saveas(gcf,file,'epsc')
%% plot states
figure
r2 = tiledlayout(2,2);
r2.TileSpacing = 'compact';
r2.Padding = 'compact';
% r2.Title.String = 'States';
% r2.Title.Interpreter = 'Latex';
% fontsize(r2.Title,17,'points')
% set(gcf, 'Units', 'Normalized', 'OuterPosition', [0.25 0.25 0.5 0.6]);

nexttile
hold on
fontsize(title('$x$-position','Interpreter', 'Latex'),15,'points')
grid on
xlabel('$t$','Interpreter', 'Latex','FontSize',17)
ylabel('$x(t)$','Interpreter', 'Latex','FontSize',17)
xlim([starttime,endtime]);
% ylim([-1.5 1.5])

plot(t,x_a1(:,1),'linewidth',2,'Color','blue');
plot(t,x_a1(:,2),'linewidth',2,'Color','red');
fontsize(legend('Agent (Robust CBF)','Agent (Standard CBF)','Interpreter', 'Latex','location','best'),13,'points');
% fontsize(leg,13,'points')

% fontsize(legend(leg, 'Interpreter', 'Latex','location','best'),12,'points')

nexttile
hold on
fontsize(title('$y$-position','Interpreter', 'Latex'),15,'points')
grid on
xlabel('$t$','Interpreter', 'Latex','FontSize',17)
ylabel('$y(t)$','Interpreter', 'Latex','FontSize',17)
xlim([starttime,endtime]);
% ylim([1.1*u2_const(1), 0.1])

plot(t,y_a1(:,1),'linewidth',2,'Color','blue');
plot(t,y_a1(:,2),'linewidth',2,'Color','red');

fontsize(legend('Agent (Robust CBF)','Agent (Standard CBF)','Interpreter', 'Latex','location','best'),13,'points');

nexttile
hold on
fontsize(title('Forward Velocity','Interpreter', 'Latex'),15,'points')
grid on
xlabel('$t$','Interpreter', 'Latex','FontSize',17)
ylabel('$v(t)$','Interpreter', 'Latex','FontSize',17)
xlim([starttime,endtime]);
ylim([-1,2.5])
% ylim([min(u_0) max(u_0)])

plot(t,rho_a1(:,1),'linewidth',2,'Color','blue');
plot(t,rho_a1(:,2),'linewidth',2,'Color','red');

fontsize(legend('Agent (Robust CBF)','Agent (Standard CBF)','Interpreter', 'Latex','location','best'),13,'points');

nexttile
hold on
fontsize(title('Heading Angle','Interpreter', 'Latex'),15,'points')
grid on
xlabel('$t$','Interpreter', 'Latex','FontSize',17)
ylabel('$\theta(t)$','Interpreter', 'Latex','FontSize',17)
xlim([starttime,endtime]);
ylim([-0.8,0.6])
% ylim([min(u_0) max(u_0)])
% ylim([1.1*u2_const(1), 0.1])

plot(t,theta_a1(:,1),'linewidth',2,'Color','blue');
plot(t,theta_a1(:,2),'linewidth',2,'Color','red');

fontsize(legend('Agent (Robust CBF)','Agent (Standard CBF)','Interpreter', 'Latex','location','best'),13,'points');

% saveas(gcf,'unknown_obst_uni_states', 'epsc')
% saveas(gcf,'unknown_obst_uni_states', 'jpg')
%% plot control actions
% u1 = zeros(length(t),index);
% u2 = zeros(length(t),index);

figure
r2 = tiledlayout(2,1);
% r2.Title.String = 'Controller Action';
% r2.Title.Interpreter = 'Latex';

nexttile
hold on
fontsize(title('Forward Acceleration Input','Interpreter', 'Latex'),16,'points')
grid on
xlabel('$t$','Interpreter', 'Latex','FontSize',16)
ylabel('$u_v(t)$','Interpreter', 'Latex','FontSize',16)
xlim([starttime,endtime]);
% ylim([-2.5 2.5])

for k = 1:index
   plot(t,u1(:,k),'linewidth',2.5)
   plot(t,u3(:,k),'linewidth',2.5,'LineStyle','--')
end

leg = string.empty;
leg(2) = 'Nominal Input';
leg(1) = 'Actual Input';
fontsize(legend(leg, 'Interpreter', 'Latex','location','best'),14,'points')

nexttile
hold on
fontsize(title('Steering Input','Interpreter', 'Latex'),16,'points')
grid on
xlabel('$t$','Interpreter', 'Latex','FontSize',16)
ylabel('$u_{\omega}(t)$','Interpreter', 'Latex','FontSize',16)
xlim([starttime,endtime]);
% ylim([1.1*u2_const(1), 0.1])

for k = 1:index
   plot(t,u2(:,k),'linewidth',2.5)
   plot(t,u4(:,k),'linewidth',2.5,'LineStyle','--')
end

fontsize(legend(leg, 'Interpreter', 'Latex','location','best'),14,'points')
% saveas(gcf,'unknown_obst_uni_control2','epsc')
% saveas(gcf,'unknown_obst_uni_control2','jpg')


%% h
h = zeros(length(t),index);

for k = 1:length(t)
    for i = 1:index
    h(k,i) = (x_a1(k,i) - x_d(k,i))^2 + (y_a1(k,i) -y_d(k,i))^2 - r^2;
    end
end
figure
hold on
grid on
plot(t,h)
xlabel('t')
ylabel('h')
xlim([0,8]);


%% Functions

function [dxdt,u_v,u_w,u0_v,u0_w] = eom(t,s,params,x_initial,i)

x_a1 = s(1);
y_a1 = s(2);
rho_a1 = s(3);
theta_a1 = s(4);

x_d = s(5);
y_d = s(6);
theta_d = s(7);

x0_a1 = x_initial(1);
y0_a1 = x_initial(2);
rho0_a1 = x_initial(3);
theta0_a1 = x_initial(4);
x0_d = x_initial(5);
y0_d = x_initial(6);

% x_ref = s(7);
% y_ref = s(8);
% rho_ref = s(9)
% theta_ref = s(10);
% 
% dx_ref = 1;
% dy_ref = 0;
% dth_ref = 0;

beta = params(1);
gamma = params(2);
r = params(3);

%Obstacle
urho_d = 1;
uth_d = 2*cos(2*t);
% uth_d = 0;

dx_d = urho_d*cos(theta_d);
dy_d = urho_d*sin(theta_d);
dth_d = uth_d;

M = abs(urho_d);
% M = 0;
M_1 = M;
M_2 = 1;

% %Reference Tracking Nominal Control (Agent 1)
% e1 = cos(theta_a1)*(x_ref - x_a1) + sin(theta)*(y_ref - y_a1);
% e2 = -sin(theta_a1)*(x_ref - x_a1) + cos(theta)*(y_ref - y_a1);
% e3 = theta_ref - theta_a1;
% 
% k1 = 1;
% k2 = 1;
% k3 = 1;
% 
% u0_1 = rho*cos(e3) + k1*e1;
% u0_2 = k2*rho*e2 + k3*rho*sin(e3);

k = 1;
u0_v = -k*(rho_a1 - 1);
u0_w = -k*(theta_a1 - theta0_a1);
% u0_w = -k*(theta_a1 - pi/2);

%Control Agent 1

if i == 1
    %h1 M
    h1_a1 = (x_a1 - x_d)^2 + (y_a1 - y_d)^2 - r^2;
    h10_a1 = (x0_a1 - x0_d)^2 + (y0_a1 - y0_d)^2 - r^2;
    eps = 0.01;
    
    Lf_h1 = 2*(x_a1 - x_d)*rho_a1*cos(theta_a1) + 2*(y_a1 - y_d)*rho_a1*sin(theta_a1);
    norm_Lp_h1 = sqrt(4*(x_a1-x_d)^2 + 4*(y_a1-y_d)^2);
    robust_term_h1 = sqrt(eps + norm_Lp_h1^2);
    
    c1 = max(0, (-Lf_h1 + robust_term_h1*M)/h10_a1) + 2;
    c1 = ceil(c1);
    
    %h2
    h2_a1 = c1*h1_a1 + Lf_h1 - norm_Lp_h1*M;
    
    Lf_h2 = 2*c1*rho_a1*((x_a1 -x_d)*cos(theta_a1) + (y_a1 - y_d)*sin(theta_a1)) + 2*rho_a1^2 ...
            -(2*M/robust_term_h1)*(Lf_h1);%2M because of Lf_h1 is multiplied by 2
    
    Lg_h2_uv = 2*(x_a1 - x_d)*cos(theta_a1) + 2*(y_a1 - y_d)*sin(theta_a1);
    Lg_h2_uw = -2*(x_a1 - x_d)*rho_a1*sin(theta_a1) + 2*(y_a1 - y_d)*rho_a1*cos(theta_a1);
    
    norm_Lg_h2 = sqrt(Lg_h2_uv^2 + Lg_h2_uw^2);
    
    Lp_h2_dx = -2*c1*(x_a1 - x_d) - 2*rho_a1*cos(theta_a1) + (4*M/robust_term_h1)*(x_a1 - x_d);
    Lp_h2_dy = -2*c1*(y_a1 - y_d) - 2*rho_a1*sin(theta_a1) + (4*M/robust_term_h1)*(y_a1 - y_d);
    
    norm_Lp_h2 = sqrt(Lp_h2_dx^2 + Lp_h2_dy^2);
    robust_term_h2 = sqrt(eps + norm_Lp_h2^2);
    
    %control
    
    u_v = u0_v + beta*(Lg_h2_uv/norm_Lg_h2)*max(0, -Lf_h2 - Lg_h2_uv*u0_v - Lg_h2_uw*u0_w + robust_term_h2*M - gamma*h2_a1);
    u_w = u0_w + beta*(Lg_h2_uw/norm_Lg_h2)*max(0, -Lf_h2 - Lg_h2_uv*u0_v - Lg_h2_uw*u0_w + robust_term_h2*M - gamma*h2_a1);
    
    % % test
    % u_w = u0_w + beta*(Lg_h2_uw/norm_Lg_h2)*max(0, -Lf_h2 - Lg_h2_uv*rho_a1 - Lg_h2_uw*u0_w + norm_Lp_h2*M - gamma*h2_a1);
    % u_v = u0_v;
elseif i == 2
    %h1 M1,M2
    M_1 = 0;
    M_2 = 0;

    h1_a1 = (x_a1 - x_d)^2 + (y_a1 - y_d)^2 - r^2;
    h10_a1 = (x0_a1 - x0_d)^2 + (y0_a1 - y0_d)^2 - r^2;
    
    Lf_h1 = 2*(x_a1 - x_d)*rho_a1*cos(theta_a1) + 2*(y_a1 - y_d)*rho_a1*sin(theta_a1);
    norm_Lp_h1 = abs(2*(x_a1 - x_d)*cos(theta_d) + (2*(y_a1-y_d)*sin(theta_d)));
    
    c1 = max(0, (-Lf_h1 + norm_Lp_h1*M)/h10_a1) + 2;
    c1 = ceil(c1);

    %h2
    h2_a1 = c1*h1_a1 + Lf_h1 - norm_Lp_h1*M_1;
    norm_term = sign(2*(x_a1-x_d)*cos(theta_d) + 2*(y_a1-y_d)*sin(theta_d));
    
    Lf_h2 = c1*Lf_h1 + 2*rho_a1^2 - M_1*norm_term *(2*rho_a1*cos(theta_a1)*cos(theta_d)...
        + 2*rho_a1*sin(theta_a1)*sin(theta_d));
    
    Lg_h2_uv = 2*(x_a1 - x_d)*cos(theta_a1) + 2*(y_a1 - y_d)*sin(theta_a1);
    Lg_h2_uw = -2*(x_a1 - x_d)*rho_a1*sin(theta_a1) + 2*(y_a1-y_d)*rho_a1*cos(theta_a1);
    
    norm_Lg_h2 = sqrt(Lg_h2_uv^2 + Lg_h2_uw^2);
    
    Lp_h2_dv = -2*c1*((x_a1 - x_d)*cos(theta_d) + (y_a1 - y_d)*sin(theta_d))...
                -2*rho_a1*(cos(theta_a1) + sin(theta_a1))...
                +M_1*norm_term*(2*cos(theta_d) + 2*sin(theta_d));
    Lp_h2_dw = M_1*norm_term*2*((x_a1 - x_d)*sin(theta_d) - 2*(y_a1 - y_d)*cos(theta_d));
    
    %control
    
    u_v = u0_v + beta*(Lg_h2_uv/norm_Lg_h2)*max(0, -Lf_h2 - Lg_h2_uv*u0_v - Lg_h2_uw*u0_w + abs(Lp_h2_dv)*M_1 ...
                                + abs(Lp_h2_dw)*M_2 - gamma*h2_a1);
    u_w = u0_w + beta*(Lg_h2_uw/norm_Lg_h2)*max(0, -Lf_h2 - Lg_h2_uv*u0_v - Lg_h2_uw*u0_w + abs(Lp_h2_dv)*M_1 ...
                                + abs(Lp_h2_dw)*M_2 - gamma*h2_a1);
end

dx_a1 = rho_a1*cos(theta_a1);
dy_a1 = rho_a1*sin(theta_a1);
drho_a1 = u_v;
dth_a1 = u_w;

% dx_a1 = rho_a1*cos(theta_a1);
% dy_a1 = rho_a1*sin(theta_a1);
% drho_a1 = u0_v;
% dth_a1 = u0_w;

dxdt = [dx_a1; dy_a1; drho_a1; dth_a1; dx_d; dy_d; dth_d];

end

function p = circle(x,y,r,c)
%x and y are the coordinates of the center of the circle
%r is the radius of the circle
%0.01 is the angle step, bigger values will draw the circle faster but
%you might notice imperfections (not very smooth)
ang=0:0.01:2*pi; 
xp=r*cos(ang);
yp=r*sin(ang);
p = plot(x+xp,y+yp,c);
end


