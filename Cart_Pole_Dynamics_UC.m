close all; clear; clc;
import casadi.*;

%Parameters
nx = 4;
nu = 1;
N = 50;
DT = 0.1;
x0bar = [0; 0.2; 0; 0];

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

%% RK4
x = MX.sym('x',nx,1);
u = MX.sym('u',nu,1);

k1 = dynamics(x,u);
k2 = dynamics(x + (DT/2)*k1, u);
k3 = dynamics(x + (DT/2)*k2, u);
k4 = dynamics(x + (DT*k3), u);
x_next = x + (DT/6)*(k1 + 2*k2 + 2*k3 + k4);

%% Symbolic blueprint
F = Function('F',{x,u},{x_next});
U=  MX.sym('U',N);
U0 = 0.1 * ones(N,1);

X = F(x0bar,U(1));
for i = 1:N-1
	X = [X, F(X(:,i),U(i+1))];
end

%Objective Function
%Sum of stage cost plus terminal cost
L_stage = sum(sum(X(:,1:end-1).^2)) + 2*sum(U.^2);
L_terminal =  100 * sum(X(:,end).^2);

L = L_stage + L_terminal;

%NLP Solver
nlp = struct('x',U,'f',L);
opts = struct;
opts.ipopt.print_level = 5;
solver = nlpsol('solver','ipopt',nlp,opts);
sol = solver('x0',U0);
U_opt = (sol.x);

FX = Function('FX',{U},{X});
X_opt = [x0bar, full(FX(U_opt))];

figure(2); clf;
subplot(2,1,1); hold on;
plot(0:N, X_opt(1,:), 'LineWidth',1.5);
plot(0:N, X_opt(2,:), 'LineWidth', 1.5);
grid on;
title('State Trajectory');
legend('Cart Position', 'Pole Angle');


subplot(2,1,2); hold on;
stairs(0:N, [full(U_opt); nan], 'r', 'LineWidth', 1.5)
grid on;
title('Control Trajectory');
legend('Force');

