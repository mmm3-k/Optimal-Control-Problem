close all; clear; clc;
import casadi.*;

%Parameters
nx = 4;
nu = 1;
N = 50;
DT = 0.1;
x0bar = [0; 3.14; 0; 0];

%System constants
m1 = 1.0;
m2 = 0.3;
l = 0.5;
g = 9.81;

%Dynamics
dynamics = @(x,u) [...
	x(3); ...
	x(4); ...
	(-m2*g*sin(x(2))*cos(x(2)) - (u + m2*l*(x(4)^2)*sin(x(2)))) / (m2*cos(x(2))^2 - (m1 + m2)); ...
	((m1 +m2)*g*sin(x(2)) + cos(x(2))*(u + m1*l*(x(4)^2)*sin(x(2))))/ (m2*l*cos(x(2))^2 - (m1 + m2)*l)...
];

%HS
x_sym = MX.sym('x',nx);
u_sym = MX.sym('u',nu);
f_dyn = Function('f_dyn',{x_sym,u_sym}, {dynamics(x_sym,u_sym)});

% Decision Variables
X = MX.sym('X',nx, N+1);
X_mid = MX.sym('X_mid', nx,N);
U = MX.sym('U',nu, N+1);

V = [reshape(X,nx*(N+1),1); reshape(X_mid,nx*N,1);reshape(U,nx*(N+1)];

g_col = [];
L = 0;

cost = @(x,u) 10*x(1)^2  + 100*x(2)^2 + 0.1*x(3)^2 + 0.1*x(4)^2 + 0.01*u^2;

for k = 1:N
	xk = X(:,k);
	xmid = X_mid(:,k+1);
	xk1 = X(:,k+1);
	
	uk = U(:,k);
	uk1 = U(:,k+1);
	umid = 0.5*(uk + uk1);
	
	fk = f_dyn(xk,uk);
	fk1 = f_dyn(xk1,uk1);
	fmid = f_dyn(xmid,umid);
	
	g_col = [g_col; xmid - (0.5*(xk+xk1) + (DT/8)*(fk - fk1))];
	g_col = [g_col; xk1 - (xk + (DT/6) * (fk + 4*fmid + fk1);)];
	
	%Simpson Integration
	Lk = cost(xk,uk);
	Lmid = cost(xmid,umid);
	Lk1 = cost(xk1,uk1);
	
	L = L + (DT/6) * (Lk + 4*Lmid + Lk1);
end

% Terminal Cost at the final node
L =  L + 500*X(1,N+1)^2 + 2000*X(2,N+1)^2 + 100*X(3,N+1)^2 + 100*X(4,N+1)^2;

%% Bounds and Boundary Conditions
lbx = -inf(size(V));
ubx = inf(size(V));

n_X    = nx * (N+1);
n_Xmid = nx * N;
n_U    = nu * (N+1);

%Constraints on Actuators
idx_U_start = n_X + n_Xmid + 1;
lbx(idx_U_start:end) = -10;
ubx(idx_U_start:end) = 10;

% Fix Initial state
lbx(1:nx) = x0bar;
ubx(1:nx) = x0bar;

%NLP Solver
nlp = struct('x',V, L, 'g',g_col);
opts = struct;
opts.ipopt.print_level = 5;
solver = nlpsol('solver', 'ipopt',nlp,opts);

%Initial guess
v0 = zeros(size(V));
sol = solver('x0',v0,'lbx', lbx, 'ubx', ubx, 'ubg', zeros(size(g_col)), 'ubg', zeros(size(g_col)));
V_opt = full(sol.x);

%Processing the output
X_opt      = reshape(V_opt(1:n_X), nx, N+1);
X_mid_opt  = reshape(V_opt(n_X+1:n_X+n_Xmid), nx, N);
U_opt      = reshape(V_opt(n_X+ n_Xmid+1:end),nu, N+1);
time_steps = 0:DT:N*DT;

%Plotting
figure(1); clf;
subplot(3,1,1); hold on;
plot(time_steps,X_opt(1,:),'-o', 'LineWidth', 1.5);
plot(time_steps,X_opt(2,:),'-s', 'LineWidth', 1.5);
grid on;
title('State Trajectory (Hermite- Simpson Collocation)')
legend('Car Position(x)', 'Pole Angle(\theta)')
xlabel('Time(s)')

subplot(3,1,2); hold on;
stairs(time_steps,X_opt(1,:),'r','LineWidth', 1.5);
grid on;
title('Linear and angular velocity');
xlabel('Time s');

subplot(3,1,3); hold on;
stairs(time_steps,U_opt,'r','LineWidth', 1.5);
grid on;
title('Control Trajectory');
xlabel('Time s');


