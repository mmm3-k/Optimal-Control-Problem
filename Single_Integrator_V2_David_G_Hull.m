close all; clear; clc;
import casadi.*;

%Parameters
nx = 1;
nu = 1;
N = 50;
tf = 1;
DT = tf/N;

%Initial Parameters
x0bar = 1;

%Dynamics
dynamics = @(x,u) [u];

x = MX.sym('x',nx,1);
u = MX.sym('u',nu,1);

%RK4 
k1 = dynamics(x,u);
k2 = dynamics(x + (DT/2)*k1,u);
k3 = dynamics(x + (DT/2)*k2,u);
k4 = dynamics(x + (DT*k3),u);
x_next = x + (DT/6)*(k1 + 2*k2 + 2*k3 + k4);

%Symbolic Blueprint
F = Function('F',{x,u},{x_next});
U =  MX.sym('U',N,1);
U0 = zeros(N,1);

X = F(x0bar,U(1));
for i = 1:N-1
	X = [X,F(X(:,i),U(i+1))];
end

X_all = [x0bar,X];

L = sum2((U'.^2 - X_all(1,1:N))) * DT;

%NLP Solver
nlp = struct('x',U,'f',L);
solver = nlpsol('solver','ipopt',nlp);
sol = solver('x0',U0);
U_opt = full(sol.x);

%Extract trajectories
FX = Function('FX',{U},{X});
X_opt = [x0bar, full(FX(U_opt))];

time_steps = 0:DT:tf;

%Plotting
figure(1); clf;
subplot(2,1,1); hold on;
plot(time_steps,X_opt(1,:),'-o','LineWidth',2);
grid on;
title('State Trajectory');
legend({'Position'});
ylabel('states');

subplot(2,1,2); hold on;
plot(time_steps,[U_opt;nan],'-r','LineWidth',2);
grid on;
title('Control Trajectory');
legend('Control');
ylabel({'Velocity'});

