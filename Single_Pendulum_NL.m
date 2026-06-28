close all; clear; clc;
import casadi.*;

%% Model Information
%Single nonlinear pendulum(1DOF) and 1 Controller at joint.

%Parameters
nx = 2;
nu = 1;
N = 50;
DT = 0.1;

%Initial Parameters
x0bar =[pi; 0];

%Dynamics
dynamics = @(x,u) [x(2); -sin(x(1))+u];

x = MX.sym('x',nx);
u = MX.sym('u',nu,1);

%RK4 solver
k1 = dynamics(x, u);
k2 = dynamics(x +(DT/2)*k1, u);
k3 = dynamics(x +(DT/2)*k2, u);
k4 = dynamics(x +(DT * k3), u);
x_next = x +(DT/6)*(k1 + 2*k2+ 2*k3+ k4);

%Symbolic blueprint
F = Function('F', {x,u},{x_next});

U = MX.sym('U',N,1);
U0 = 0.2 * ones(N,1);

X = F(x0bar,U(1));
for i = 1:N-1
	X = [X, F(X(:,i),U(i+1))];
end

%Objective Function
L = 10*sum(X(:,end).^2);


%NLP formulation
nlp = struct('x',U,'f',L);
solver = nlpsol('solver','ipopt',nlp);
sol = solver('x0',U0);
U_opt = full(sol.x);

FX = Function('FX',{U},{X});
X_opt = [x0bar, full(FX(U_opt))];

%Post Processing
figure(1); clf;
subplot(2,1,1); hold on;
plot(0:N, X_opt(1,:))
plot(0:N, X_opt(2,:));
title('state trajectory');
legend('\phi','\omega')

subplot(2,1,2); hold on;
stairs(0:N, [full(U_opt); nan]);
title('control trajectory');
legend('\tau')
