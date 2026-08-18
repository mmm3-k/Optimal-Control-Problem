# Optimal Control using Local Hermite-Simpson Collocation

This repository contains a MATLAB implementation of a Direct Transcription method—specifically the **Local Hermite-Simpson Direct Collocation Method**—using the [CasADi](https://casadi.org) optimization framework and the IPOPT NLP solver. The script optimizes the trajectory of a dynamic cart system to minimize control effort while steering the system toward the origin.

---

## Problem Explanation

The script solves an **Optimal Control Problem (OCP)** over a finite time horizon $T = 2.0$ seconds divided into $N = 20$ mesh intervals. 

Because Hermite-Simpson is a high-order implicit collocation method, each interval requires three distinct nodes: a starting node ($x_k$), a midpoint ($x_{\text{mid}}$), and an ending node ($x_{k+1}$). This results in a discrete state and control grid spanning $2N + 1 = 41$ points.

### System Dynamics
The underlying system models a basic mass-cart assembly with standard velocity-dependent damping friction. The state-space equations are:
$$\dot{x}_1 = x_2 \quad \text{(Velocity equation for Position } x_1\text{)}$$
$$\dot{x}_2 = u - 0.5x_2 \quad \text{(Acceleration equation with a damping coefficient of 0.5)}$$

Where:
* $X = [x_1, x_2]^T$ represents the state vector (Position, Velocity).
* $U = [u]$ represents the control input (Force/Acceleration command).

---

## Objective and Constraints

The physical objective is to find the optimal control trajectory $u(t)$ that drives the cart from its initial position toward the origin balance point ($0,0$) within $2.0$ seconds while consuming minimal control energy.

### 1. Objective Function (Cost)
The optimization minimizes a cost function composed of an integrated running cost (control effort) and a heavily weighted terminal penalty (final boundary error):

$$\min_{X, U} \quad J = \int_{0}^{T} 0.1 u(t)^2 \, dt + 100 \left( x_1(T)^2 + x_2(T)^2 \right)$$

In the code, the continuous running cost integral is approximated numerically across each interval using **Simpson's Quadrature Rule**:
$$\int_{t_k}^{t_{k+1}} L(u) \, dt \approx \frac{\Delta t}{6} \left( 0.1 u_k^2 + 4(0.1 u_{\text{mid}}^2) + 0.1 u_{k+1}^2 \right)$$

### 2. Constraints

The problem is transformed into a Non-Linear Programming (NLP) problem subject to algebraic equality constraints, meaning the lower bound (`lbg`) and upper bound (`ubg`) for these constraint arrays are set strictly to `0`.

* **Initial Boundary Constraint:** Enforces that the trajectory must begin precisely at the user-defined state $x_0 = [5, 1]^T$:
  $$X_1 - x_0 = 0$$

* **Hermite-Simpson Midpoint Interpolation:** Forces the internal midpoint state variables to match a cubic spline interpolation driven by the system derivatives:
  $$x_{\text{mid}} - \left[ \frac{1}{2}(x_k + x_{k+1}) + \frac{\Delta t}{8}(f_k - f_{k+1}) \right] = 0$$

* **Collocation Defect Constraints (State Continuity):** Links adjacent nodes together by ensuring the forward state matches Simpson's integration step, maintaining physics continuity:
  $$x_{k+1} - x_k - \frac{\Delta t}{6} \left( f_k + 4f_{\text{mid}} + f_{k+1} \right) = 0$$

*(Note: $f_k, f_{\text{mid}}, f_{k+1}$ represent the evaluated system dynamics function $\dot{x} = f(x,u)$ at each node).*

---

## Code Structure & Tooling

* **CasADi (`MX.sym`)**: Used to construct symbolic graph structures for decision variables (`w`), costs, and algorithmic constraints.
* **IPOPT**: An interior-point optimizer invoked by CasADi to solve large-scale sparse non-linear problems.
* **Post-processing**: The flattened output optimization array `w_opt` is reshaped back into matrix tracking formats for clear plotting.

### Prerequisites
1. MATLAB
2. [CasADi for MATLAB](https://casadi.orgget/) added to your MATLAB path environment.
