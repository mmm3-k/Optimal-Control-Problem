close all; clear; clc;
import casadi.*;

%Parameters
nx = 3;
nu = 1;
N  = 100;
g = 9.81;
h_val = 0.1;

%Initial Conditions
x0bar = [0; 0; 0];
xf = 1;

%Dynamics
dynamics = @(x,u) [x(3) * cos(u); x(3) * sin(u); g * sin(u)];

x = MX.sym('x',nx,1);
u = MX.sym('u',nu,1);
dt = MX.sym('dt');

%RK4
k1 = dynamics(x,u);
k2 = dynamics(x + (DT/2)*k1, u);
k3 = dynamics(x + (DT/2)*k2, u);
k4 = dynamics(x + DT*k3, u);
x_next = x + (DT/6)*(k1 + 2*k2 + 2*k3 + k4);

%Symbolic Blue print
F = Function('F', {x,u,DT},{x_next});

U = MX.sym('U',N);
U0 = 0.1* ones(N,1);
DT = MX.sym('DT');
DT0 = 0.1;

w = [U;DT];
w0 = [U0; DT0];

%Simulation Loop
X = F(x0bar,U(1),DT);
for i =1:N-1
	X = [X, F(X(:,i),U(i+1), DT)];
end
X_full = [x0bar, X];

%Objective
J = N * DT;

%Terminal Constraint(xf =1)
g_terminal = X_full(1,end);

%Path inequaility constraint(y - x/2 <=h)
g_path = (X_full(2,:) - X_full(1,:)/2 - h_val)';

%Combine constraints
g_combine = [g_terminal;g_path];

%Control constraints
lbu = -pi * ones(N,1);
ubu = pi * ones(N,1);

%Step size constraints
lb_dt = 0.001;
up_dt = 1.0;

%Combine two constraints
lbw = [lbu;lb_dt];
ubw = [ubu;ub_dt];

lbg = [xf; - inf * ones(N+1, 1)];
ubg = [xf; 0 * ones(N+1,1)];

%NLP Solver
nlp = struct('x',w,'f', J, 'g', g_combine);
solver = nlpsol('solver','ipopt',nlp);
sol = solver('x0',w0,'lbx',lbw,'ubx',ubw,'lbg',lbg,'ubg',ubg);
w_opt = full(sol.x);

%Extract Results
U_opt =  w_opt(1:N);
DT_opt = w_opt(end);
tf_opt = N * DT_opt;

% Post processing
FX = Function('FX',{w},{X_full});
X_opt = full(FX(w_opt));
time_steps = 0:DT_opt:tf_opt;

figure(1);clf;
subplot(2,1,1); hold on;
plot(X_opt(1,:), X_opt(2,:),'-b', 'LineWidth',2);
x_line = linspace(0,1,100);
y_line = x_line / 2 + h_val;
plot(x_line,y_line, '--r', 'LineWidth', 1.5);
grid on;
title('Brachistochrone Path Profile');
xlabel('Horizontal Position x (m)');
ylabel('Vertical Position y (m););
legend('Optimal Trajectory','Path Boundary Constraint(y \leq x/2 + h)','Location', 'best');

%Control Profile
subplot(2,1,2);
stairs(time_steps, [U_opt;nan],'m','LineWidth',1.5);
grid on;
title('Optimal control angle trajectory u(t));
xlabel('Time (sec)');
ylabel('Angle u (radians)');