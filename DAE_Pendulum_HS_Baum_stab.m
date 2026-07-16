close all; clear; clc;
import casadi.*;

%Parameters
nx = 4;
nu = 1;
nz = 1;
N = 30;
T = 2.0;
dt = T/N;

%Physical Parameters
g = 9.81;
m = 1.0;
L = 1.0;

% Stabilize DAE by Baumgarte method
alpha = 100.0;
beta = 1000.0;

% Define DAE system
x_mat = MX.sym('x_mat',nx,1);
z_mat = MX.sym('z_mat',nz,1);
u_mat = MX.sym('u_mat',nu,1);

x1 = x_mat(1); 
x2 = x_mat(2);
x3 = x_mat(3);
x4 = x_mat(4);
y = z_mat(1);
u = u_mat(1);

%Dynamics
dynamics = [...
		x3;...
		x4;...
		(1/m) * (-2*x1*y + (u*x2)/ L);...
		(1/m) * (-m*g - 2*x2*y - (u*x1)/L)...
];

% Kinematic Constraints
G_pos = x1^2 + x2^2 - L^2;
G_vel = 2*x1*x3 + 2*x2*x4;
G_acc = 2*(x3^2 + x4^2 + x1 * dynamics(3) + x2 * dynamics(4));

%Baumgarte Stabilization Constraints
g_alg_baumgarte = G_acc + 2 * alpha * G_vel + (beta^2) * G_pos;

f_fun =  Function('f_fun', {x_mat, z_mat, u_mat},{dynamics});
g_fun = Function('g_fun', {x_mat, z_mat, u_mat}, {g_alg_baumgarte});
pos_fun = Function('pos_fun', {x_mat},{G_pos});

% Decision variables formulation
X = MX.sym('X',nx, 2*N+1);
Z = MX.sym('Z',nz, 2*N+1);
U = MX.sym('U',nu, 2*N+1);

w = [X(:);Z(:);U(:)];
x_start = [L; 0; 0; 0];

%Initialize guess
X0 = repmat([L;0;0;0],1, 2*N+1);
Z0 = zeros(nz,2*N+1);
U0 = zeros(nu, 2*N+1);
w0 = [X0(:); Z0(:); U0(:)];

lbX = -inf * ones(nx, 2*N+1); ubX = inf * ones(nx, 2*N+1);
lbZ = -inf * ones(nz, 2*N+1); ubZ = inf * ones(nz, 2*N+1);
lbU = -inf * ones(nu, 2*N+1); ubU = inf * ones(nu, 2*N+1);

lbw = [lbX(:); lbZ(:); lbU(:)];
ubw = [ubX(:); ubZ(:); ubU(:)];

g = [];
lbg = [];
ubg = [];

g = [g; X(:,1) - x_start];
lbg = [lbg;zeros(nx,1)];
ubg = [ubg;zeros(nx,1)];

g = [g;X(1,end)];
lbg = [lbg; 0];
ubg = [ubg; 0];

g = [g;X(3,end)];
lbg = [lbg; 0];
ubg = [ubg; 0];

for i =1:N
	idx_k = 2*i-1;
	idx_mid = 2*i;
	idx_kpl = 2*i+1;
	
	x_k = X(:,idx_k);
	z_k = Z(:,idx_k);
	u_k = U(:,idx_k);
	
	x_mid = X(:,idx_mid);
	z_mid = Z(:,idx_mid);
	u_mid = U(:,idx_mid);
	
	x_kpl = X(:,idx_kpl);
	z_kpl = Z(:,idx_kpl);
	u_kpl = U(:,idx_kpl);
	
	f_k = f_fun(x_k, z_k, u_k);
	f_mid = f_fun(x_mid,z_mid,u_mid);
	f_kpl = f_fun(x_kpl, z_kpl, u_kpl);
	
	%Hermite Simpson Midpoint Interpolation Defect
	x_mid_interp = 0.5*(x_k + x_kpl)+ (dt/8) * (f_k- f_kpl);
	g = [g; x_mid - x_mid_interp];
	lbg = [lbg; zeros(nx,1)];
	ubg = [ubg; zeros(nx,1)];
	
	%Defect Constraint
	x_clct_defect = x_kpl - x_k - (dt/6)*(f_k +4*f_mid + f_kpl);
	g = [g; x_clct_defect];
	lbg = [lbg; zeros(nx,1)];
	ubg = [ubg; zeros(nx,1)];
	
	% stabilized Baumgarte Path constraints (only at boundary nodes)
	g = [g; g_fun(x_k, z_k, u_k)];
	lbg = [lbg;0];
	ubg = [ubg;0];
	g = [g; g_fun(x_mid,z_mid,u_mid)];
	lbg = [lbg;0];
	ubg = [ubg;0];
end

g = [g; g_fun(X(:,end),Z(:,end),U(:,end))];
lbg = [lbg; 0];
ubg = [ubg; 0];

%Objective Function
L_running = 0;
for i = 1:N
	idx_k = 2*i-1;
	idx_mid = 2*i;
	idx_kpl = 2*i+1;
	L_running = L_running + (dt/6)*(U(:,idx_k)^2 + 4*U(:,idx_mid)^2 + U(:,idx_kpl)^2);
end

L = L_running;

%Create NLP solver
nlp = struct('x',w,'f',L,'g',g);
opts = struct;
opts.ipopt.max_iter = 500;
opts.ipopt.tol = 1e-6;
solver = nlpsol('solver','ipopt',nlp,opts);
sol = solver('x0',w0,'lbx',lbw,'ubx',ubw,'lbg',lbg,'ubg',ubg);


%Extract solutions
w_opt = full(sol.x);
X_opt = reshape(w_opt(1:nx*(2*N+1)),nx, 2*N+1);
Z_opt = reshape(w_opt(nx*(2*N+1)+1:(nx+nz)*(2*N+1)),nz, 2*N+1)
U_opt = reshape(w_opt((nx+nz)*(2*N+1)+1:end),nu,2*N+1);
t_grid = linspace(0,T,2*N+1);

% post optimization verification
diff_defect = zeros(nx,N);
t_intervals = zeros(1,N);
position_error = zeros(1, 2*N+1);

for k =1:(2*N+1)
	position_error(k) = full(pos_fun(X_opt(:,k)));
end

for i = 1:N
	idx_k = 2*i-1;
	idx_mid = 2*i;
	idx_kpl = 2*i+1;
	t_intervals(i) = t_grid(idx_mid);
	
	f_k = full(f_fun(X_opt(:, idx_k), Z_opt(:, idx_k), U_opt(:, idx_k)));
	f_mid = full(f_fun(X_opt(:,idx_mid),Z_opt(:,idx_mid), U_opt(:,idx_mid)));
	f_kpl = full(f_fun(X_opt(:, idx_kpl), Z_opt(:,idx_kpl), U_opt(:, idx_kpl)));
	
	diff_defect(:,i) = X_opt(:,idx_kpl) - X_opt(:,idx_k) - (dt/6) * (f_k + 4*f_mid + f_kpl);
end
diff_defect_norm = sqrt(sum(diff_defect.^2,1));

%Plotting results
figure(1); clf;
subplot(4,1,1); hold on;
plot(t_grid,X_opt(1,:),'-o', 'LineWidth',1.5);
plot(t_grid,X_opt(2,:),'--s','LineWidth',1.5);
grid on;
title('Pendululm Coordinates(Hermite Simpson Method)');
legend('x_1(Horizontal)','x_2(Vertical)','Location','best');

subplot(4,1,2); hold on;
plot(t_grid,Z_opt,'-dg', 'LineWidth',1.5);
plot(t_grid,U_opt,'-or','LineWidth',1.5);
grid on;
title('Multiplier force and Applied Torque');
legend('Algebraic Multiplier','Control Input','Location','best');

subplot(4,1,3); hold on;
plot(t_grid,position_error,'-mx','LineWidth', 1.5);
grid on;
title('Drift error');
ylabel('Error');
legend('Invariance Error','Location', 'best');

subplot(4,1,4); hold on;
stem(t_intervals,diff_defect_norm,'b', 'LineWidth',1.5,'MarkerFaceColor','b');
grid on;
title('Differential Dynamic Defect Norm per Interval');
ylabel('Defect Norm');
xlabel('Time');

