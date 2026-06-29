close all; clear; clc;
import casadi.*;

%Minimum Time Problem

%Parameters
nx = 3;
nu = 1;
N = 40;
g = 9.81;

%Initial constraint
x0bar = [0; 10; 0];

%Final Constraint
xf_target = [10; 5];

%dynamics
dynamics = @(x,u) [x(3) * sin(u); -x(3) * cos(u); g * cos(u)];

x = MX.sym('x',nx,1);
u = MX.sym('u',nu,1);
DT = MX.sym('DT');

%RK4
k1 = dynamics(x,u);
k2 = dynamics(x+(DT/2)*k1, u);
k3 = dynamics(x +(DT/2)*k2, u);
k4 = dynamics(x + DT*k3, u);
x_next = x + (DT/6)*(k1+2*k2+2*k3+k4);

%Symbolic blueprint
F = Function('F', {x,u,DT},{x_next});
U = MX.sym('U',N);
U0 = 0.1*ones(N,1);

w = [U; DT];
w0 = [U0; 0.05];

X = F(x0bar, U(1),DT);
for i = 1:N-1
	X = [X, F(X(:,i),U(i+1),DT)];
end

% Objective Function
L = N * DT;

%Terminal Constraint
g_expr = X(1:2,end);

%Control constraint
lb_u = -pi * ones(N,1);
ub_u = pi * ones(N,1);

% Step size constraint
lb_dt = 0.001;
ub_dt = 1.0;

lbw = [lb_u; lb_dt];
ubw = [ub_u; ub_dt];

%NLP solver
nlp = struct('x',w,'f',L,'g',g_expr);
solver =nlpsol('solver','ipopt',nlp);
sol = solver('x0',w0,'lbx',lbw,'ubx',ubw,'lbg',xf_target,'ubg',xf_target);
w_opt = full(sol.x);

%Extract Trajectories
U_opt = w_opt(1:N);
DT_opt = w_opt(end);
T_f_opt = N * DT_opt;

FX = Function('FX',{U,DT},{X});
X_opt = [x0bar, full(FX(U_opt,DT_opt))];

time_steps = 0:DT_opt:T_f_opt;

%%Post Processing
figure(1); clf;

subplot(2,2,1); hold on;
plot(X_opt(1,:),X_opt(2,:), '-o', 'LineWidth',1.5);
plot(x0bar(1),x0bar(2),'ro','MarkerFaceColor', 'r', 'MarkerSize',8);
plot(xf_target(1),xf_target(2),'go','MarkerFaceColor', 'g','MarkerSize',8);
grid on; axis equal;
xlabel('Position x');ylabel('Position y');
title('Profile');
legend('Trajectory','Start A', 'End B');

%plot state dimensions
subplot(2,2,2); hold on;
plot(time_steps,X_opt(1,:),'LineWidth',1.5);
plot(time_steps,X_opt(2,:),'LineWidth',1.5);
plot(time_steps,X_opt(3,:),'LineWidth',1.5);
grid on;
xlabel('time(s)');
ylabel('Values');
legend('x(t)','y(t)','v(t)');

%Plot control action
subplot(2,2,3); hold on;
stairs(time_steps,[U_opt;nan],'r','LineWidth',1.5);
grid on;
xlabel('time(s)');
ylabel('Control u(t)');
title('Control Trajectories');
