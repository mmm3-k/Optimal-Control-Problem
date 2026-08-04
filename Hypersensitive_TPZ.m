close all; clear; clc;
import casadi.*;

%Parameters
nx = 1;
nu = 1;
N = 1000;
tf = 1000;
DT = tf/N;

%Initial Conditions
x0 = 1.5;
xf = 1.0;

%Dynamics
dynamics = @(x,u) -x + u;

%HS Symbolics
x_sym = MX.sym('x',nx);
u_sym = MX.sym('u',nu);
f_dyn = Function('f_dyn',{x_sym,u_sym}, {dynamics(x_sym,u_sym)});

% Decision Variables
X = MX.sym('X',nx, N+1);
U = MX.sym('U',nu, N+1);

% Flatten into a single decision vector
V = [reshape(X,nx*(N+1),1); reshape(U,nu*(N+1),1)];

g_col = [];
L = 0;

cost = @(x,u) 0.5 * (x^2 + u^2);

for k = 1:N
    xk = X(:,k);
    xk1 = X(:,k+1);
    
    uk = U(:,k);
    uk1 = U(:,k+1);
    
    fk = f_dyn(xk,uk);
    fk1 = f_dyn(xk1,uk1);
    
    % Trepezoidal collocation constraints
    g_col = [g_col; xk1 - (xk + (DT/2) * (fk + fk1))];
    
    % Trepezoidal Integration for Cost Function
    Lk = cost(xk,uk);
    Lk1 = cost(xk1,uk1);
    
    L = L + (DT/6) * (Lk + Lk1);
end

% Terminal Cost at the final node (Fixed: index fits nx = 1)
L = L + 500 * (X(1,N+1) - xf)^2; 

%% Bounds and Boundary Conditions
lbx = -inf(size(V));
ubx = inf(size(V));

n_X    = nx * (N+1);
n_Xmid = nx * N;
n_U    = nu * (N+1);

% Constraints on Actuators
idx_U_start = n_X + 1;
lbx(idx_U_start:end) = -10;
ubx(idx_U_start:end) = 10;

% Fix Initial state (Fixed: changed x0bar to x0)
lbx(1:nx) = x0;
ubx(1:nx) = x0;

% NLP Solver Setup
nlp = struct('x',V,'f', L, 'g',g_col);
opts = struct;
opts.ipopt.print_level = 5;
solver = nlpsol('solver', 'ipopt',nlp,opts);

% Initial guess
v0 = zeros(size(V));
sol = solver('x0',v0,'lbx', lbx, 'ubx', ubx, 'lbg', zeros(size(g_col)), 'ubg', zeros(size(g_col)));
V_opt = full(sol.x);

% Processing the output
X_opt      = reshape(V_opt(1:n_X), nx, N+1);
X_mid_opt  = reshape(V_opt(n_X+1:n_X+n_Xmid), nx, N);
U_opt      = reshape(V_opt(n_X+n_Xmid+1:end), nu, N+1);
time_steps = 0:DT:N*DT;

% Plotting
figure(1); clf;
subplot(2,1,1); hold on;
plot(time_steps, X_opt(1,:), '-b', 'LineWidth', 1.5);
grid on;
ylabel('State (x)');
title('Optimal State Trajectory(Hypersensitive Problem(TPZ)');

subplot(2,1,2); hold on;
plot(time_steps, U_opt(1,:), '-r', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Control (u)');
title('Optimal Control Input');