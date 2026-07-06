# Optimal-Control-Problem

# Learning Basics of OCP

## Problem 1: Brachistochrone with Path Constraints (`Brachistochrone_PC.m`)

### ⚙️ System Dynamics
The system state vector is defined as $\mathbf{x} = [x, y, v]^T$, representing horizontal position ($x$), vertical position ($y$), and velocity ($v$). Controlled by the steering angle $u$, the continuous-time system dynamics are governed by:

$$\dot{x} = v \cos(u)$$
$$\dot{y} = v \sin(u)$$
$$\dot{v} = g \sin(u)$$

Where $g = 9.81 \text{ m/s}^2$ is the acceleration due to gravity. The continuous dynamics are discretized over each time step $\Delta t$ using a 4th-order Runge-Kutta (RK4) integration scheme.

---

### 🎯 Objective Function
The objective is to minimize the total travel time ($t_f$) using a constant time step ($\Delta t$) over $N = 100$ intervals:

$$\min_ \quad J = t_f = N \cdot \Delta t$$

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

  #### Problem 2: Brachistochrone with UnConstraints (`Brachistochrone_UC.m`)
  This problem is taken from https://openmdao.github.io/dymos/examples/brachistochrone/brachistochrone.html
