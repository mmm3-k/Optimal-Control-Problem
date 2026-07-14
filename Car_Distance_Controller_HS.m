close all; clear; clc;
import casadi.*;

%%Initial Parameters
nx = 2;
nu = 1;
N = 20;
T = 2.0;
dt = T/N;
x0bar = [5; 1];

%Dynamics
dynamics = @(x,u) [x(2); u - 0.5*x(2)];

X = MX.sym('X',nx, 2*N +1);
U = MX.sym('U',nu, 2*N +1);

%Combine decision variables
w = [X(:); U(:)];

%Initial guess
X0 = repmat(x0bar,1,2*N+1);
U0 = zeros(nu,2*N+1);
w0 = [X0(:); U0(:)];

%Constraints
g = [];
lbg = [];
ubg = [];

g = [g; X(:,1)- x0bar];
lbg = [lbg; zeros(nx,1)];
ubg = [ubg; zeros(nx,1)];

for i = 1:N
	idx_k = 2*i - 1;
	idx_mid = 2*i;
	idx_kpl = 2*i +1;
	
	x_k = X(:,idx_k);
	x_mid = X(:,idx_mid);
	x_kpl = X(:, idx_kpl);
	
	u_k = U(:,idx_k);
	u_mid = U(:,idx_mid);
	u_kpl = U(:,idx_kpl);
	
	f_k = dynamics(x_k,u_k);
	f_mid = dynamics(x_mid,u_mid);
	f_kpl = dynamics(x_kpl,u_kpl);
	
	x_mid_interp = 0.5* (x_k + x_kpl) + (dt/8) * (f_k - f_kpl);
	g = [g;x_mid - x_mid_interp];
	lbg = [lbg; zeros(nx,1)];
	ubg = [ubg; zeros(nx,1)];
	
	x_clct_defect = x_kpl - x_k - (dt/6) * (f_k + 4 *f_mid + f_kpl);
	g = [g; x_clct_defect];
	lbg = [lbg; zeros(nx,1)];
	ubg = [ubg; zeros(nx,1)];
end

%Cost function Integration(Simpson's Rule)
L_running = 0;
for i = 1:N
	idx_k = 2*i - 1;
	idx_mid = 2*i;
	idx_kpl = 2*i +1;
	
	L_running = L_running +(dt/6)*...
		(0.1*U(:,idx_k)^2 + 4*(0.1*U(:, idx_mid)^2) + 0.1*U(:,idx_kpl)^2);
end

L_terminal = 100*(X(1,end)^2 + X(2,end)^2);
L =  L_running + L_terminal;

%NLP Solver
nlp = struct('x',w,'f',L,'g',g);
solver = nlpsol('solver','ipopt',nlp);
sol = solver('x0',w0,'lbg',lbg,'ubg',ubg);

%Post processing
w_opt = full(sol.x);
X_opt = reshape(w_opt(1:nx*(2*N+1)),nx,2*N+1);
U_opt = reshape(w_opt(nx*(2*N+1)+1:end),nu,2*N+1);

t_grid = linspace(0,T,2*N+1);

%Plotting
figure(1);clf;
subplot(2,1,1); hold on;
plot(t_grid,X_opt(1,:), '-o','LineWidth',1.5);
plot(t_grid,X_opt(2,:),'--s','LineWidth',1.5);
grid on;title('Cart State Trajectory(Local Hermite-Simpson Method');
legend({'Position,Velocity'});
xlabel('Time');

subplot(2,1,2);
plot(t_grid,U_opt,'-or','LineWidth',1.5)
grid on;title('Applied Force');
legend('Force');
xlabel('Time');
	



