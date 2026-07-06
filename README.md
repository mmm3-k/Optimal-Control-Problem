# Optimal-Control-Problem

# Learning Basics of OCP

## Problem 1: Brachistochrone with Path Constraints (`Brachistochrone_PC.m`)

### 🎯 Objective Function
The objective is to minimize the total travel time ($t_f$) using a constant time step ($\Delta t$) over $N = 100$ intervals:

$$\min_{u, \Delta t} \quad J = t_f = N \cdot \Delta t$$

---

### 🛑 Boundary Conditions & Constraints

#### 1. Boundary Conditions
* **Initial State:** The system starts from rest at the origin:
  $$x(0) = 0, \quad y(0) = 0, \quad v(0) = 0$$
* **Terminal State:** The problem terminates when the horizontal position reaches $x_f = 1$:
  $$x(t_f) = 1$$

#### 2. Path Inequality Constraint
The trajectory must stay below the linear boundary ($h = 0.1$) at all times:
$$y(t) \leq \frac{1}{2}x(t) + 0.1$$

#### 3. Variable Bounds (Box Constraints)
* **Control Bounds:** The steering angle $u$ is restricted to a full circle sweep:
  $$-\pi \leq u(t) \leq \pi$$
* **Time Step Bounds:** The decision variable $\Delta t$ is bounded for solver stability:
  $$0.001 \leq \Delta t \leq 1.0 \text{ seconds}$$

