close all; clear; clc;
import casadi.*;

%%Model info
%Unconstrained Double Integrator with objective is position and velocity should be zero at time t_f

%Parameters
nx = 2;
nu = 1;
N = 20;
T_f = 1.5;
DT = T_f / N;

%Initial Conditions
x0bar = [1; 1];

%Dynamics
dynamics = @(x,u) [x(2); u];

x = MX.sym('x',nx,1);
u = MX.sym('u',nu,1);

%RK4 
k1 = dynamics(x, u);
k2 = dynamics(x + (DT/2)*k1, u);
k3 = dynamics(x + (DT/2)*k2, u);
k4 = dynamics(x + DT*k3, u);
x_next =  x + (DT/6)*(k1 + 2*k2 + 2*k3 +k4);

F = Function('F',{x,u},{x_next});
U = MX.sym('U',N);
U0 = zeros(N,1);

X = F(x0bar,U(1));
for i = 1:N-1
	X = [X,F(X(:,i),U(i+1))];
end

%Objective
L = 0.5 * sum(U.^2) * DT;

%Terminal Constraints
g_expr = X(:,end);

%NLP Solver
nlp = struct('x',U,'f',L,'g',g_expr);
solver = nlpsol('solver','ipopt',nlp);
sol = solver('x0',U0,'lbg',[0;0],'ubg',[0;0]);
U_opt = full(sol.x);

%Extract Trajectory
FX =  Function('FX',{U},{X});
X_opt = [x0bar, full(FX(U_opt))];

time_steps = 0:DT:T_f;

%Plotting
figure(1); clf;
subplot(2,1,1); hold on;
plot(time_steps,X_opt(1,:),'-o', 'LineWidth', 1.5)
plot(time_steps,X_opt(2,:),'--s', 'LineWidth', 1.5)
grid on;
title('State Trajectories');
legend({'Position','velocity'});
ylabel('state');

subplot(2,1,2); hold on;
stairs(time_steps,[U_opt;nan],'r','LineWidth',1.5)
grid on;
title('Control Trajectory (Acceleration)');
legend('Control');
xlabel('Time');
ylabel('Acceleration');
