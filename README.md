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

---

## Problem 2: Brachistochrone Unconstrained (`Brachistochrone_UC.m`)
*Reference: [Dymos Brachistochrone Example](https://openmdao.github.io/dymos/examples/brachistochrone/brachistochrone.html)*

### ⚙️ System Dynamics
The unconstrained problem evaluates a bead falling down a frictionless wire profile. In this variation, the state vector remains $\mathbf{x} = [x, y, v]^T$, but the tracking configuration changes the vertical axis orientation downward ($y_0 = 10 \rightarrow y_f = 5$). The coordinate control uses the wire angle tangent ($\theta$):

$$\frac{d x}{d t} = v \sin(\theta)$$
$$\frac{d y}{d t} = -v \cos(\theta)$$
$$\frac{d v}{d t} = g \cos(\theta)$$

---

### 🎯 Objective Function
The objective is to find the continuous profile curve that minimizes final time ($t_f$):

$$\min_{\theta} \quad J = t_f$$

---

### 🛑 Boundary Conditions & Constraints

#### 1. Boundary Conditions
* **Initial State:** The bead is released from rest at a high platform $A(0,10)$:
  $$x(0) = 0, \quad y(0) = 10, \quad v(0) = 0$$
* **Terminal State:** The target endpoint settles at platform $B(10,5)$, leaving terminal velocity free:
  $$x(t_f) = 10, \quad y(t_f) = 5, \quad v(t_f) = \text{free}$$

#### 2. Path Constraints
* None (Unconstrained wire geometry).

#### 3. Control Bounds
* **Control Bounds:** The steering angle $u$ is restricted to a full circle sweep:
  $$-\pi \leq u(t) \leq \pi$$
* **Time Step Bounds:** The decision variable $\Delta t$ is bounded for solver stability:
  $$0.001 \leq \Delta t \leq 1.0 \text{ seconds}$$

  ## Problem 3: Nonlinear Pendulum Regulator (`Pendulum_Regulator.m`)

### ⚙️ System Dynamics
The system represents a single-degree-of-freedom (1DOF) nonlinear pendulum with a single torque controller at the joint. The state vector is defined as $\mathbf{x} = [\phi, \omega]^T$, where $\phi$ is the angular position (rad) and $\omega$ is the angular velocity (rad/s). Controlled by the input torque $u$, the continuous-time dynamics are:

$$\dot{\phi} = \omega$$
$$\dot{\omega} = -\sin(\phi) + u$$

The continuous system is discretized over a fixed time step $\Delta t = 0.1 \text{ s}$ over a horizon of $N = 50$ intervals using a 4th-order Runge-Kutta (RK4) integration scheme.

---

### 🎯 Objective Function
The objective is a terminal cost regularizer designed to penalize deviations of the final state from the origin (bringing the pendulum to rest at the vertical-down equilibrium position):

$$\min_{u} \quad J = 10 \cdot \left( \phi(t_f)^2 + \omega(t_f)^2 \right)$$

---

### 🛑 Boundary Conditions & Constraints

#### 1. Boundary Conditions
* **Initial State:** The pendulum starts in the completely inverted vertical-up position at rest:
  $$\phi(0) = \pi, \quad \omega(0) = 0$$
* **Terminal State:** The final states are unconstrained but heavily penalized in the cost function.

#### 2. Path & Variable Constraints
* **State / Control Constraints:** None (unconstrained optimization problem).

  ## Problem 4: Nonlinear Rocket Landing (`Rocket_Landing.m`)

### ⚙️ System Dynamics
The system represents a 1D vertical rocket landing problem. The state vector is defined as $\mathbf{x} = [p, v]^T$, where $p$ is the altitude or vertical position (m) and $v$ is the vertical velocity (m/s). Controlled by the thrust force input $u$, the continuous-time system dynamics are subject to gravity and aerodynamic drag:

$$\dot{p} = v$$
$$\dot{v} = -g - c_d \cdot v |v| + u$$

Where:
* $g = 9.81 \text{ m/s}^2$ is the acceleration due to gravity.
* $c_d = 0.1$ is the aerodynamic drag coefficient.

The continuous dynamics are discretized over a fixed terminal time $T_f = 2.0 \text{ s}$ divided into $N = 100$ intervals ($\Delta t = 0.02 \text{ s}$) using a 4th-order Runge-Kutta (RK4) integration scheme.

---

### 🎯 Objective Function
The objective is a stage cost that minimizes the total control/fuel energy consumption over the landing trajectory:

$$\min_{u} \quad J = \sum_{i=1}^{N} u_i^2 \cdot \Delta t$$

---

### 🛑 Boundary Conditions & Constraints

#### 1. Boundary Conditions
* **Initial State:** The rocket begins its landing descent phase from an altitude of 15 meters with a downward velocity:
  $$p(0) = 15, \quad v(0) = -5$$
* **Terminal State (Pinpoint Soft Landing):** The rocket must reach zero altitude with zero velocity at the exact final time step $t_f$:
  $$p(t_f) = 0, \quad v(t_f) = 0$$

#### 2. Actuator Constraints (Box Constraints)
* **Thrust Limits:** The engine cannot provide negative thrust (no reverse thrusters) and is capped at a maximum thrust of 25 N:
  $$0 \leq u(t) \leq 25 \text{ N}$$


