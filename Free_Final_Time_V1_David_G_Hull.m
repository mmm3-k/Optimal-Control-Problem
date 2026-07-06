close all; clear; clc;
import casadi.*;

% Parameters
nx = 1;
nu = 1;
N = 50;
k_param = 1; % Weight factor 'k' from the problem statement

% Decision Variables
U = MX.sym('U', N, 1);
tf = MX.sym('tf'); % Free final time as a decision variable

DT = tf / N;

% Initial Parameters
x0bar = 0; % From image: x0 = 0

% Dynamics
dynamics = @(x,u) [u];

x = MX.sym('x', nx, 1);
u = MX.sym('u', nu, 1);

% RK4 Integration (dependent on symbolic DT)
k1 = dynamics(x, u);
k2 = dynamics(x + (DT/2)*k1, u);
k3 = dynamics(x + (DT/2)*k2, u);
k4 = dynamics(x + (DT*k3), u);
x_next = x + (DT/6)*(k1 + 2*k2 + 2*k3 + k4);

% Symbolic Blueprint
F = Function('F', {x, u, tf}, {x_next});

% Forward Simulation (Single Shooting)
X = x0bar;
for i = 1:N
    X = [X, F(X(:, i), U(i), tf)];
end
xf = X(:, end); % Terminal state

% Performance Index (Objective Function)
% J = tf + k * integral(u^2 dt)
L = tf + k_param * sum(U.^2) * DT;

% Group decision variables
opt_vars = [U; tf];

% Bounds on variables
lbw = [-inf * ones(N, 1); 0.01]; % tf must be strictly positive
ubw = [inf * ones(N, 1); inf];

% Initial guess
w0 = [zeros(N, 1); 1.0]; % Initial guess for controls and tf = 1.0

% Terminal State Constraint (xf = 1)
g = xf;
lbg = 1;
ubg = 1;

% NLP Solver Configuration
nlp = struct('x', opt_vars, 'g', g, 'f', L);
solver = nlpsol('solver', 'ipopt', nlp);
sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw, 'lbg', lbg, 'ubg', ubg);

% Extract Optimal Trajectories
w_opt = full(sol.x);
U_opt = w_opt(1:N);
tf_opt = w_opt(end);
DT_opt = tf_opt / N;

% Reconstruct State Trajectory Numerically
X_opt = zeros(1, N+1);
X_opt(1) = x0bar;
for i = 1:N
    k1_n = dynamics(X_opt(i), U_opt(i));
    k2_n = dynamics(X_opt(i) + (DT_opt/2)*k1_n, U_opt(i));
    k3_n = dynamics(X_opt(i) + (DT_opt/2)*k2_n, U_opt(i));
    k4_n = dynamics(X_opt(i) + (DT_opt*k3_n), U_opt(i));
    X_opt(i+1) = X_opt(i) + (DT_opt/6)*(k1_n + 2*k2_n + 2*k3_n + k4_n);
end

time_steps = 0:DT_opt:tf_opt;

% Plotting Results
figure(1); clf;
subplot(2,1,1); hold on;
plot(time_steps, X_opt, '-o', 'LineWidth', 2);
grid on;
title(sprintf('State Trajectory (Optimal tf = %.3f)', tf_opt));
ylabel('State (x)');

subplot(2,1,2); hold on;
plot(time_steps, [U_opt; nan], '-r', 'LineWidth', 2);
grid on;
title('Control Trajectory');
ylabel('Control (u)');
