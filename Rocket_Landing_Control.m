close all; clear; clc;
import casadi.*;

%%Rocker Landing Problem

%Parameters
nx = 2;
nu = 1;
N = 100;
T_f = 2.0;
DT = T_f / N;

%Initial Parameters
x0bar = [15; -5];

%dynamics 
gravity = 9.81;
drag_coeff = 0.1;
dynamics = @(x,u) [x(2); -gravity - drag_coeff*x(2)*abs(x(2)) + u];

x = MX.sym('x',nx,1);
u = MX.sym('u',nu,1);

%RK4 solver
k1 = dynamics(x, u);
k2 = dynamics(x + (DT/2)*k1, u);
k3 = dynamics(x + (DT/2)*k2, u);
k4 = dynamics(x + (DT*k3), u);
x_next = x + (DT/6)*(k1+2*k2+2*k3+k4);

F = Function('F',{x,u},{x_next});

U = MX.sym('U',N);
U0 = 15 * ones(N,1);

X = F(x0bar, U(1));
for i = 1:N-1
	X = [X, F(X(:,i),U(i+1))];
end

%Objective:Minimize total fuel energy 
L = sum(U.^2) * DT;

% hard constraints: Final position and velocity should be zero.
g_expr = X(:,end);

% Actuator constraints: Cannot exceed 25N from 0N.
lb_control = 0 * ones(N,1);
ub_control  = 25 * ones(N,1);

%NLP Solver
nlp = struct('x',U,'f',L,'g',g_expr);
solver = nlpsol('solver','ipopt',nlp);
sol = solver('x0',U0,'lbx',lb_control,'ubx',ub_control,'lbg',[0; 0], 'ubg',[0; 0]);
U_opt = full(sol.x);

%Extract Trajectories
FX = Function('FX',{U},{X});
X_opt = [x0bar, full(FX(U_opt))];

time_steps = 0:DT:T_f;

%Post processing
figure(1); clf;
subplot(2,1,1); hold on;
plot(time_steps, X_opt(1,:), '-o', 'LineWidth', 1.5);
plot(time_steps, X_opt(2,:), '--s', 'LineWidth', 1.5);
grid on; ylabel('Rocket States');
title('Nonlinear Rocket Landing Trajectory');
legend('Position','velocity');

subplot(2,1,2); hold on;
stairs(time_steps, [U_opt;nan], 'r','LineWidth',1.5)
grid on; ylabel('Thrust Force [N]'); xlabel('Time [seconds]');
title('Rocket Landing control');
legend('Engine Thrust u(t)');
