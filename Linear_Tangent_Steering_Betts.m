close all; clear; clc;
import casadi.*;

%Parameters
nx = 4;
nu = 1;
a = 100;
N = 100;

%Initial Conditions
x0bar = [0; 0; 0; 0];

%Terminal Conditions
x2 = 5;
x3 = 45;
x4 = 0;

%Dynamics
dynamics = @(x,u) [x(3); x(4); a * cos(u); a * sin(u)];

x = MX.sym('x',nx,1);
u = MX.sym('u',nu,1);
dt = MX.sym('dt');

%RK4
k1 = dynamics(x,u);
k2 = dynamics(x + (dt/2)*k1, u);
k3 = dynamics(x + (dt/2)*k2, u);
k4 = dynamics(x + dt*k3, u);
x_next = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

F = Function('F',{x,u,dt},{x_next});
U = MX.sym('U',N,1);
DT = MX.sym('DT');
U0 = 0.1*ones(N,1);
w = [U;DT];
w0 = [U0;0.01];

X = F(x0bar,U(1),DT);
for i = 1:N-1
	X = [X, F(X(:,i),U(i+1),DT)];
end

X_full = [x0bar,X];

%Objective
L = N * DT;

%Terminal Constraints
g_expr = X_full(2,end);
g_expr = [g_expr;X_full(3,end)];
g_expr = [g_expr;X_full(4,end)];

%Control Constraints
lb_control = -inf * ones(N,1);
ub_control = inf * ones(N,1);

%Step size Constraint
lb_dt = 0.001;
ub_dt = 1.0;

lbx = [lb_control;lb_dt];
ubx = [ub_control;ub_dt];

lbg = [x2; x3; x4];
ubg = [x2; x3; x4];

%NLP solver
nlp = struct('x',w,'f',L,'g',g_expr);
solver = nlpsol('solver','ipopt',nlp);
sol = solver('x0',w0,'lbx',lbx,'ubx',ubx,'lbg',lbg,'ubg',ubg);
w_opt = full(sol.x);

%Extract Trajectory
U_opt = w_opt(1:N);
DT_opt = w_opt(end);
tf_opt = N * DT_opt;

FX = Function('FX',{w},{X_full});
X_opt = full(FX(w_opt));
time_steps = 0:DT_opt:tf_opt;

%Plots
figure(1); clf;
subplot(3,1,1); hold on;
plot(time_steps,X_opt(1,:),'-b','LineWidth', 1.5)
plot(time_steps,X_opt(2,:), '--r','LineWidth',1.5) 
plot(tf_opt,x2, 'ro', 'MarkerFaceColor', 'r');
grid on;
title('State Profiles:Position');
xlabel('Time');
ylabel('Position');
legend('x_1(t) - Free Target', 'x_2(t) - Target = 5', 'Location', 'best');

%Velocity Plot
subplot(3,1,2); hold on;
plot(time_steps,X_opt(3,:), '-g', 'LineWidth',1.5)
plot(time_steps,X_opt(4,:), '--m', 'LineWidth', 1.5)
plot(tf_opt,x3,'go', 'MarkerFaceColor','g');
plot(tf_opt,x4, 'mo', 'MarkerFaceColor', 'm');
grid on;
title('State Profiles: Velocity');
xlabel('Time');
ylabel('Velocity');
legend('x_3(t) - Target = 45', 'x_4(t) - Target = 0', 'Location', 'best');

%Plot Steering Angle
subplot(3,1,3);
stairs(time_steps,[U_opt;nan],'k','LineWidth',1.5)
grid on;
title('Optimal Steering Control Trajectory');
xlabel('Time');
ylabel('Angle');
