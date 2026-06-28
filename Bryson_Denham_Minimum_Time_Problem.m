close all; clear; clc;
import casadi.*;

%Parameters
nx = 2;
nu = 1;
N = 200;

%Initial
x0bar = [0; 1];

%Dynamics
dynamics = @(x,u) [x(2); u];

x = MX.sym('x', nx, 1);
u = MX.sym('u', nu, 1);
DT = MX.sym('DT');

k1 = dynamics(x, u);
k2 = dynamics(x +(DT/2)*k1, u);
k3 = dynamics(x +(DT/2)*k2, u);
k4 = dynamics(x + (DT*k3), u);
x_next = x + (DT/6)*(k1 + 2*k2 + 2*k3 + k4);

%Symbolic Blueprint
F = Function('F',{x,u,DT},{x_next});
U = MX.sym('U',N);
U0 = zeros(N,1);

%Combine Initial guess for control and time steps
w = [U;DT];
w0 = [U0; 0.1];

X = F(x0bar,U(1),DT);
for i = 1:N-1
	X = [X, F(X(:,i), U(i+1), DT)];
end

% objective
L = N * DT;

%Terminal Constraints
g_expr = X(:, end);
lbg = [0; -1];
ubg = [0; -1];

%Path Constraint
path_cons = X(1,:)';
g_expr = [g_expr; path_cons];

lbg = [lbg; -inf * ones(N,1)];
ubg = [ubh; (1/9) * ones(N,1)];

%Control Constraint
u_m = 2.0;
lb_u = -u_m * ones(N,1);
ub_u = u_m * ones(N,1);

% Time_step Constraint
lb_dt = 0.001;
ub_dt = 1.0;

lbw = [lb_u; lb_dt];
ubw = [ub_u; ub_dt];

%Nlp solver
nlp = struct('x',w,'f',L,'g',g_expr);
solver = nlpsol('solver','ipopt',nlp);
sol = solver('x0',w0,'lbx',lbw,'ubx',ubw,'lbg',lbg,'ubg',ubg);
w_opt = full(sol.x);

%Extract Trajectories
U_opt =w_opt(1:N);
DT_opt = w_opt(end);
T_f_opt = N * DT_opt;

FX = Function('FX',{U},{X});
X_opt = [x0bar, full(FX(w_opt,DT_opt))];

time_steps = 0:DT_opt:T_f_opt;

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






