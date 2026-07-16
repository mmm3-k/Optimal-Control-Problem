close all; clear;clc;
import casadi.*;

%Parameters
nx = 2;
nu = 1;
N = 20;
T = 2.0;
dt =  T/N;
x0bar = [5;1];

%Dynamics
dynamics = @(x,u) [x(2); u - 0.5*x(2)];

X = MX.sym('X',nx,N+1);
U = MX.sym('U',nu,N+1);
w = [X(:); U(:)];

%Initial Guess
X0 = repmat(x0bar,1,N+1);
U0 = zeros(nu, N+1);
w0 = [X0(:); U0(:)];

g = [];
lbg = [];
ubg = [];

g = [g;X(:,1)-x0bar];
lbg = [lbg; zeros(nx,1)];
ubg = [ubg; zeros(nx,1)];

%Trapezoidal Collocation Constraints
for i = 1:N
	x_k	  = X(:,i);
	x_kpl = X(:,i+1);
	
	u_k   = U(:,i);
	u_kpl = X(:,i+1);
	
	%Evaluate dynamics at node k and k+1
	f_k   = dynamics(x_k,u_k);
	f_kpl = dynamics(x_kpl,u_kpl);
	
	%Defect Constraints
	x_clct_defect =  x_kpl - x_k - (dt/2) * (f_k+f_kpl);
	
	g = [g; x_clct_defect];
	lbg = [lbg;zeros(nx,1)];
	ubg = [ubg;zeros(nx,1)];
end

% Cost Function Integration
L_running = 0;
for i = 1:N
	L_running = L_running + (dt/2)*(0.1*U(:,i)^2 + 0.1*U(:,i+1)^2);
end

%Terminal boundary state Penalty
L_terminal = 100 * (X(1,end)^2 + X(2,end)^2);
L = L_running + L_terminal;

%NLP solver
nlp = struct('x',w,'f',L,'g',g);
solver = nlpsol('solver','ipopt',nlp);
sol = solver('x0',w0,'lbg',lbg,'ubg',ubg);

%Post processing
w_opt = full(sol.x);
X_opt = reshape(w_opt(1:nx*(N+1)),nx, N+1);
U_opt = reshape(w_opt(nx*(N+1)+1:end),nu,N+1);
t_grid = linspace(0,T,N+1);

%Plotting
figure(1); clf;
subplot(2,1,1); hold on;
plot(t_grid,X_opt(1,:),'-o','LineWidth', 1.5);
plot(t_grid,X_opt(2,:),'--s','LineWidth',1.5);
grid on; title('Cart State Trajectory (Local Trapezoidal Collocation)');
legend({'Position,Velocity'});
xlabel('Time');

subplot(2,1,2);
plot(t_grid,U_opt,'-or','LineWidth',1.5);
grid on; title('Applied Force');
legend('Force');
xlabel('Time');


