close all; clear; clc;
import casadi.*;

% Parameters
nx = 2;
nu = 1;
N = 40;
T_f = 1;
DT = T_f / N;

%Initial Parameters
x0bar = [0; 1];

%dynamics
dynamics = @(x,u) [x(2); u];

x = MX.sym('x',nx,1);
u = MX.sym('u',nu,1);

k1 = dynamics(x,u);
k2 = dynamics(x + (DT/2)* k1, u);
k3 = dynamics(x + (DT/2)* k2, u);
k4 = dynamics(x + (DT*k3), u);
x_next = x + (DT/6)*(k1 + 2*k2 + 2*k3 + k4);

%Symbolic Blue print
F = Function('F',{x,u},{x_next});
U = MX.sym('U',N);
U0 = zeros(N,1);
X = F(x0bar,U(1));
for i =1:N-1
	X = [X, F(X(:,i),U(i+1))];
end

% Objective
L = 0.5 * sum(U.^2) * DT;

% Terminal Constraints
g_expr = X(:,end);
lbg = [0; -1];
ubg = [0; -1];

%Path Constraint
path_cons = X(1,:)';
g_expr = [g_expr; path_cons];

lbg = [lbg; -inf*ones(N,1)];
ubg = [ubg; (1/9)*ones(N,1)];

%Control Constraint
u_m = 50.0;
lb_u = -u_m * ones(N,1);
ub_u = u_m * ones(N,1);

%NLP solver
nlp = struct('x',U,'f',L,'g',g_expr);
solver = nlpsol('solver','ipopt',nlp);
sol = solver('x0',U0,'lbx',lb_u,'ubx',ub_u,'lbg',lbg,'ubg',ubg);
U_opt = full(sol.x);

% Extract Trajectories
FX = Function('FX', {U},{X});
X_opt = [x0bar, full(FX(U_opt))];

time_steps = 0:DT:T_f;

%Post processing
figure(1); clc;
subplot(2,1,1); hold on;
plot(time_steps, X_opt(1,:), '-o', 'LineWidth', 1.5);
plot(time_steps, X_opt(2,:), '--s','LineWidth', 1.5);
yline(1/9, 'r:', 'Displacment Upper Bound (1/9)', 'LineWidth', 1.2);
grid on;
title('State Trajectories');
legend('Position', 'velocity');
ylabel('State');

subplot(2,1,2); hold on;
plot(time_steps, [U_opt; nan], 'r', 'LineWidth', 1.5);
grid on;
title('Control Trajectories');
legend('control u(t)');
xlabel('Time');
ylabel('Acceleration');



