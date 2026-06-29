close all; clear; clc;
import casadi.*;

%Parameters
nx = 2;
nu = 1;
N  = 50;

%Initial Conditions
x0bar = [-0.5; 0];

%Dynamics
dynamics = @(x,u) [x(2); 0.00 * u - 0.0025 * cos(3*x(1))];

x = MX.sym('x',nx, 1);
u = MX.sym('u',nu, 1);
DT = MX.sym('dt');

%RK4
k1 = dynamics(x,u);
k2 = dynamics(x + (DT/2)*k1, u);
k3 = dynamics(x + (DT/2)*k2, u);
k4 = dynamics(x + (DT*k3), u);
x_next = x + (DT/6)*(k1 + 2*k2 + 2*k3 + k4);

F = Function('F',{x,u,DT},{x_next});
U = MX.sym('U',N);
U0 = zeros(N,1);
w = [U;DT];
w0 = [U0;0.1];

X = F(x0bar,U(1),DT);
for i =1:N-1
	X = [X,F(X(:,i),U(i+1),DT)];
end

%Objective 
L = N * DT;

%Hard Constrain
g_expr = X(:,end);

%Control Constraints
u_max = 1.0;
lb_control = -u_max * ones(N,1);
ub_control = u_max * ones(N,1);

%Step size Constraints
lb_dt = 0.001;
up_dt = 1.0;

lbw = [lb_control;lb_dt];
ubw = [ub_control;ub_dt];


%NLP solver
nlp = struct('x',w,'f',L,'g',g_expr);
solver = nlpsol('solver','ipopt',nlp);
sol = solver('x0',u0,'lbx',lbw,'ubx',ubw,'lbg',[0.5;0],'ubg',[0.5;inf]);
w_opt = full(sol.x);

%% Extract Trajectories
U_opt = w_opt(1:N);
DT_opt = w_opt(end);
T_f_opt = N * DT_opt;


FX = Function('FX',{U,DT},{X});
X_opt = [x0bar,full(FX(U_opt,DT_opt))];

time_steps = 0:DT_opt:T_f_opt;

%Plotting
figure(1);clf;
subplot(2,1,1); hold on;
plot(time_steps,X_opt(1,:),'-o', 'LineWidth', 1.5)
plot(time_steps,X_opt(2,:),'--s','LineWidth', 1.5)
grid on;
title(sprintf('State Trajectory (Total Time: %.2fs)',T_f_opt));
legend({'Position',Velocity'});
ylabel('States');

subplot(2,1,2); hold on;
stairs(time_steps,[U_opt;nan],'r','LineWidth', 1.5);
grid on;
title('Control Trajectory (Acceleration)');
legend('Control');
xlabel('Time');
ylabel('Acceleration');