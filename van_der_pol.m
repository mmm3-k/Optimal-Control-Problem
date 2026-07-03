close all; clear; clc;
import casadi.*;

%Parameters
nx = 2;
nu = 1;
N = 100;
T = 10.0;
DT = T / N;

%Initial Parameters
x0bar = [1; 1];

%Dynamics
dynamics = @(x,u) [(1 - x(2)^2 * x(1) - x(2) + u; x(1)];

%Implicit RADAU IIA (2stage, 3rd Order) via rootfinder
x_k = MX.sym('x_k',nx,1);
u_k = MX.sym('u_k',nu,1);

%2 Unknown Intermediate stage states
x_s1 = MX.sym('x_s1',nx,1);
x_s2 = MX.sym('x_s2',nx,1);

f_s1 = dynamics(x_s1,u_k);
f_s2 = dynamics(x_s2,u_k);

res_s1 = x_s1 - (x_k + DT * ((5/12)*f_s1 - (1/12)*f_s2));
res_s2 = x_s2 - (x_k + DT * ((3/4) *f_s1 - (1/4)* f_s2));

res_combine = [res_s1; res_s2];
x_unknows  = [x_s1;x_s2];

nlp_root =  struct('x',x_unknows,'p',[x_k; u_k],'g',res_combine);
opts_root = struct();
opts_root.abstol = 1e-10;
implicit_step = rootfinder('implicit_step','newton',nlp_root,opts_root);
res_step = implicit_step('x0',[x_k;x_k],'p',[x_k;u_k]);
x_next_computed  = res_step.x(nx+1:end);

F_radau = Function('F_radau',{x_k,u_k},{x_next_computed});

U = MX.sym('U',nu,N);
X = x0bar;
L = 0;
for k = 1:N
	x_next = F_radau(X(:,k),U(:,k));
	X = [X,x_next];
	
	L = L + (X(1,k)^2 + X(2,k)^2 + U(:,k)^2) * DT;
end

g_expr = X(:,end);

%NLP struct
nlp = struct('x',reshape(U, N*nu, 1),'f', L, 'g', g_expr);
opts = struct();
opts.ipopt.print_level = 5;
opts.ipopt.tol = 1e-8;

solver = nlpsol('solver','ipopt',nlp, opts);

% Control Constraints
lbu = -0.75;
ubu = 1.0;
lbx = lbu * ones(N,1);
ubx = ubu * ones(N,1);

U0 = -0.75 * ones(N,1);
sol = solver('x0',U0,'lbx',lbx,'ubx',ubx,'lbg',[0;0],'ubg',[0;0]);
U_opt = full(sol.x);

%High precision forward verification simulaiton(ODE45)
t_fine = [];
X_fine = [];
x_current = x0bar;

for k = 1:N
	u_current = U_opt(k);
	tspan = [(k-1)*DT, k*DT];
	[t_seg, x_seg] = ode45(@(t,x) dynamics(x, u_current),tspan,x_current,...
							odeset('AbsTol',1e-12,'RelTol',1e-10));
	
	if k == 1
		t_fine = [t_fine; t_seg];
		X_fine = [X_fine; x_seg];
	else
		t_fine = [t_fine; t_seg(2:end)];
		X_fine = [X_fine; x_seg(2:end,:)];
	end
	x_current = x_seg(end,:)';
end

FX = Function('FX',{reshape(U, N*nu,1)},{X});
X_opt = full(FX(U_opt));
t_grid = 0:DT:T;

X_fine_interp = interpl(t_fine,X_fine,t_grid)';

%Abs Error
abs_error_x0 = abs(X_opt(1,:) - X_fine_interp(1,:));
abs_error_x1 = abs(X_opt(2,:) - X_fine_interp(2,:));

% Relative Error
eps_safety = 1e-6;
rel_error_x0 = abs_error_x0 ./ (abs(X_fine_interp(1,:)) + eps_safety);
rel_error_x1 = abs_error_x1 ./ (abs(X_fine_interp(2,:)) + eps_safety);

%Ploting
figure(1); clf;
subplot(4,1,1); hold on;
plot(t_grid,X_opt(1,:),'r-', 'LineWidth', 2);
plot(t_grid,X_opt(2,:), 'b-', 'LineWidth',2);
grid on;
title('State Trajectory');

subplot(4,1,2); hold on;
stairs(t_grid,[U_opt;nan],'k','LineWidth',2);
grid on;
title('Optimal control Trajectory');
legend('Control input u');
ylabel('control Magnitude');

subplot(4,1,3); hold on;
plot(t_grid, abs_error_x0,'r-','LineWidth',1.5);
plot(t_grid, abs_error_x1,'b-', 'LineWidth',1.5);
grid on;
title('Absolute error');

subplot(4,1,4); hold on;
plot(t_grid,rel_error_x0,'r-', 'LineWidth',1.5);
plot(t_grid,rel_error_x1, 'b-', 'LineWidth',1.5);
grid on;
title('Relative Error');



