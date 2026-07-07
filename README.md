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

  ## Problem 5: Cart-Pole Stabilization (`Cart_Pole.m`)

### ⚙️ System Dynamics
The system represents a classic underactuated Cart-Pole system (inverted pendulum mounted on a motorized cart). The state vector is defined as $\mathbf{x} = [p, \theta, \dot{p}, \dot{\theta}]^T$, where:
* $p$ is the horizontal position of the cart (m).
* $\theta$ is the angular position of the pole (rad), where $\theta = 0$ is the vertical upright equilibrium position.
* $\dot{p}$ is the linear velocity of the cart (m/s).
* $\dot{\theta}$ is the angular velocity of the pole (rad/s).

Controlled by a horizontal force input $u$ applied directly to the cart, the highly nonlinear continuous-time system dynamics are governed by:

$$\dot{x}_1 = \dot{p}$$
$$\dot{x}_2 = \dot{\theta}$$
$$\dot{x}_3 = \frac{-m_2 g \sin(\theta)\cos(\theta) - \left(u + m_2 l \dot{\theta}^2 \sin(\theta)\right)}{m_2 \cos^2(\theta) - (m_1 + m_2)}$$
$$\dot{x}_4 = \frac{(m_1 + m_2)g \sin(\theta) + \cos(\theta)\left(u + m_1 l \dot{\theta}^2 \sin(\theta)\right)}{m_2 l \cos^2(\theta) - (m_1 + m_2)l}$$

Where:
* $m_1 = 1.0 \text{ kg}$ is the mass of the cart.
* $m_2 = 0.3 \text{ kg}$ is the mass of the pole.
* $l = 0.5 \text{ m}$ is the length of the pole.
* $g = 9.81 \text{ m/s}^2$ is the acceleration due to gravity.

The system is discretized over a fixed time step $\Delta t = 0.1 \text{ s}$ across a horizon of $N = 50$ intervals using a 4th-order Runge-Kutta (RK4) integration scheme.

---

### 🎯 Objective Function
The objective function balances regularizing the intermediate states, minimizing control effort (stage costs), and strictly penalizing deviations from the origin at the final horizon (terminal cost):

$$\min_{u} \quad J = \sum_{i=1}^{N-1} \left( \|\mathbf{x}_i\|_2^2 + 2 u_i^2 \right) + 100 \|\mathbf{x}_N\|_2^2$$

---

### 🛑 Boundary Conditions & Constraints

#### 1. Boundary Conditions
* **Initial State:** The cart starts at the origin, but the pole begins with a slight angular offset from the vertical upright position:
  $$p(0) = 0, \quad \theta(0) = 0.2, \quad \dot{p}(0) = 0, \quad \dot{\theta}(0) = 0$$
* **Terminal State:** Unconstrained in the NLP definition, but heavily suppressed toward zero by the terminal weight factor ($100$).

#### 2. Path & Variable Constraints
* **State / Control Constraints:** None (unconstrained optimization problem).


#### 2. Actuator Constraints (Box Constraints)
* **Thrust Limits:** The engine cannot provide negative thrust (no reverse thrusters) and is capped at a maximum thrust of 25 N:
  $$0 \leq u(t) \leq 25 \text{ N}$$

  ## Problem 6: Double Integrator Minimum-Energy Trajectory (`Double_Integrator.m`)

### ⚙️ System Dynamics
The system represents a classic unconstrained linear Double Integrator (e.g., a frictionless point mass under direct acceleration control). The state vector is defined as $\mathbf{x} = [p, v]^T$, where $p$ is the position (m) and $v$ is the velocity (m/s). Controlled by the linear acceleration input $u$ ($\text{m/s}^2$), the continuous-time state-space dynamics are governed by:

$$\dot{p} = v$$
$$\dot{u} = u$$

The continuous system is discretized over a fixed terminal landing horizon $T_f = 1.5 \text{ s}$ divided into $N = 20$ intervals ($\Delta t = 0.075 \text{ s}$) using a 4th-order Runge-Kutta (RK4) integration scheme.

---

### 🎯 Objective Function
The objective is a quadratic stage cost representing the minimization of control energy (\(L_2\)-norm of acceleration) throughout the transition horizon:

$$\min_{u} \quad J = \frac{1}{2} \sum_{i=1}^{N} u_i^2 \cdot \Delta t$$

---

### 🛑 Boundary Conditions & Constraints

#### 1. Boundary Conditions
* **Initial State:** The system starts away from the origin with an initial position offset and a positive velocity vector:
  $$p(0) = 1, \quad v(0) = 1$$
* **Terminal State (Pinpoint Origin Rest):** The vehicle must perfectly settle at zero position and zero velocity at the final time step $t_f$:
  $$p(t_f) = 0, \quad v(t_f) = 0$$

#### 2. Path & Variable Constraints
* **State / Control Constraints:** None (completely unconstrained workspace and infinite actuator capability).

  ## Problem 7: Constrained Double Integrator Maximum-Position Trajectory (`Double_Integrator_Max_Pos.m`)

### ⚙️ System Dynamics
The system represents a constrained linear Double Integrator under direct acceleration control. The state vector is defined as $\mathbf{x} = [p, v]^T$, where $p$ is the position (m) and $v$ is the velocity (m/s). Controlled by the bounded acceleration input $u$ ($\text{m/s}^2$), the continuous-time state-space dynamics are governed by:

$$\dot{p} = v$$
$$\dot{v} = u$$

The continuous system is discretized over a fixed terminal horizon $T_f = 1.0 \text{ s}$ divided into $N = 40$ intervals ($\Delta t = 0.025 \text{ s}$) using a 4th-order Runge-Kutta (RK4) integration scheme.

---

### 🎯 Objective Function
The objective is to maximize the final horizontal position reached at the terminal time step $t_f$. In minimization form, this is expressed as the negative of the final position state:

$$\min_{u} \quad J = -p(t_f)$$

---

### 🛑 Boundary Conditions & Constraints

#### 1. Boundary Conditions
* **Initial State:** The system starts completely at rest at the origin:
  $$p(0) = 0, \quad v(0) = 0$$
* **Terminal State (Zero Terminal Velocity):** The vehicle's position at $t_f$ is left free to maximize, but it must come to a complete stop with zero final velocity:
  $$p(t_f) = \text{free}, \quad v(t_f) = 0$$

#### 2. Actuator Constraints (Box Constraints)
* **Acceleration Limits:** The control input is symmetrically bounded to represent physical actuator saturation limits, driving a classic bang-bang control profile:
  $$-1.0 \leq u(t) \leq 1.0 \text{ m/s}^2$$

  ## Problem 8: Minimum-Time 2D Particle Steering (`Particle_Steering.m`)
*Reference: This problem is taken from "Practical Methods for Optimal Control and Estimation Using Nonlinear Programming" by John T. Betts.*

### ⚙️ System Dynamics
The system describes a 2D point mass or particle moving under a constant acceleration magnitude $a$, where the control variable $u$ dictates the steering direction/angle of the acceleration vector. The state vector is defined as $\mathbf{x} = [x, y, v_x, v_y]^T$, representing horizontal position ($x$), vertical position ($y$), horizontal velocity ($v_x$), and vertical velocity ($v_y$). The continuous-time system dynamics are:

$$\dot{x} = v_x$$
$$\dot{y} = v_y$$
$$\dot{v}_x = a \cos(u)$$
$$\dot{v}_y = a \sin(u)$$

Where $a = 100 \text{ m/s}^2$ is the constant acceleration magnitude. The continuous dynamics are discretized over a free terminal horizon $t_f = N \cdot \Delta t$ split into $N = 100$ intervals, using a 4th-order Runge-Kutta (RK4) integration scheme where the time step $\Delta t$ is a decision variable.

---

### 🎯 Objective Function
The objective is to minimize the total travel time ($t_f$) required to satisfy the terminal conditions:

$$\min_{u, \Delta t} \quad J = t_f = N \cdot \Delta t$$

---

### 🛑 Boundary Conditions & Constraints

#### 1. Boundary Conditions
* **Initial State:** The particle starts fully at rest from the origin:
  $$x(0) = 0, \quad y(0) = 0, \quad v_x(0) = 0, \quad v_y(0) = 0$$
* **Terminal State:** The particle must reach a specific vertical position and velocity vector at the final time $t_f$, while the final horizontal position $x(t_f)$ is left entirely free:
  $$x(t_f) = \text{free}, \quad y(t_f) = 5, \quad v_x(t_f) = 45, \quad v_y(t_f) = 0$$

#### 2. Variable Bounds (Box Constraints)
* **Control Bounds:** The steering angle $u$ is completely unconstrained:
  $$-\infty \leq u(t) \leq \infty \text{ rad}$$
* **Time Step Bounds:** The grid size $\Delta t$ is bounded to maintain numerical integration stability:
  $$0.001 \leq \Delta t \leq 1.0 \text{ seconds}$$

  ## Problem 9: Mass-Spring System Energy Optimization (`Mass_Spring_Hull.m`)
*Reference: This problem is taken from "Optimal Control Theory for Applications" by David G. Hull.*

### ⚙️ System Dynamics
The system represents a structural mass-spring arrangement operating under direct velocity control. The state vector is scalar, $\mathbf{x} = [x]$, representing the physical position of the mass (m). Controlled directly by the velocity input $u$ ($\text{m/s}$), the continuous-time dynamics are governed by:

$$\dot{x} = u$$

The continuous system is discretized over a fixed terminal horizon $t_f = 1.0 \text{ s}$ split into $N = 50$ intervals ($\Delta t = 0.02 \text{ s}$) using a 4th-order Runge-Kutta (RK4) integration scheme. 

---

### 🎯 Objective Function
The objective function maps an energy-balancing index across the transition horizon. It minimises the difference between the squared velocity (resembling a scaled kinetic energy) and a high-frequency position penalty parameter (resembling a structural spring potential energy):

$$\min_{u} \quad J = \sum_{i=1}^{N} \left( u_i^2 - \alpha^2 x_i^2 \right) \cdot \Delta t$$

Where the system frequency parameter $\alpha$ is defined by structural constants:
* $m = 5 \text{ kg}$ (Mass)
* $k = 2000 \text{ N/m}$ (Spring stiffness coefficient)
* $\alpha = \frac{k}{m} = 400$

---

### 🛑 Boundary Conditions & Constraints

#### 1. Boundary Conditions
* **Initial State:** The mass begins with an initial displacement offset from the structural rest position:
  $$x(0) = 0.1 \text{ m}$$
* **Terminal State (Return to Origin):** The system must perfectly return to the structural origin at the final horizon timestamp $t_f$:
  $$x(t_f) = 0$$

#### 2. Actuator Constraints (Box Constraints)
* **Velocity Limits:** The system control input is bounded symmetrically to represent strict speed limits on the joint runner:
  $$-15 \leq u(t) \leq 15 \text{ m/s}$$

  ## Problem 10: Minimum-Time Mountain Car Escape (`Mountain_Car.m`)
*Reference: This problem is taken from the [Dymos Mountain Car Example](https://openmdao.github.io/dymos/examples/mountain_car/mountain_car.html), which evaluates a classic reinforcement learning benchmark within an optimal control framework.*

### ⚙️ System Dynamics
The system simulates an underpowered car trapped inside a deep valley or "well." The vehicle lacks the engine power to directly accelerate uphill out of the basin and must rock back and forth repeatedly to build up enough kinetic energy to escape. 

The state vector is defined as $\mathbf{x} = [x, v]^T$, representing the car's horizontal position ($x$) and vertical velocity ($v$). Controlled by a bounded throttle/force input $u$, the continuous-time nonlinear system dynamics are governed by:

$$\dot{x} = v$$
$$\dot{v} = 0.001 u - 0.0025 \cos(3x)$$

The continuous equations of motion are discretized over a free terminal horizon $t_f = N \cdot \Delta t$ split into $N = 150$ intervals using a 4th-order Runge-Kutta (RK4) integration scheme, where the time step $\Delta t$ is structured as an optimization decision variable.

---

### 🎯 Objective Function
The objective is to minimize the total time $t_f$ required for the vehicle to successfully climb up and escape out of the positive side of the valley grid:

$$\min_{u, \Delta t} \quad J = t_f = N \cdot \Delta t$$

---

### 🛑 Boundary Conditions & Constraints

#### 1. Boundary Conditions
* **Initial State:** The car begins fully at rest from a stationary position low inside the western ridge:
  $$x(0) = -0.5, \quad v(0) = 0$$
* **Terminal State:** The problem terminates successfully when the vehicle breaches the target hill crest ($x_f = 0.5$) with a non-negative outward velocity:
  $$x(t_f) = 0.5, \quad v(t_f) \geq 0$$

#### 2. Variable Bounds (Box Constraints)
* **Actuator/Control Bounds:** The engine throttle effort is restricted to a normalized maximum authority profile, generating a strong bang-bang control response:
  $$-1.0 \leq u(t) \leq 1.0$$
* **Time Step Bounds:** The grid delta variable $\Delta t$ is bounded to maintain integration fidelity:
  $$0.001 \leq \Delta t \leq 1.0 \text{ seconds}$$

  ## Problem 11: State-Control Tracking Regularization (`State_Tracking_Hull.m`)
*Reference: This problem is taken from "Optimal Control Theory for Applications" by David G. Hull.*

### ⚙️ System Dynamics
The system model uses a single-degree-of-freedom scalar dynamic environment operating under direct velocity control. The state vector is scalar, $\mathbf{x} = [x]$, representing the system's position metric. Controlled directly by the linear velocity input $u$ ($\text{m/s}$), the continuous-time kinematic dynamics are governed by:

$$\dot{x} = u$$

The continuous system is discretized over a fixed terminal landing horizon $t_f = 1.0 \text{ s}$ divided into $N = 50$ intervals ($\Delta t = 0.02 \text{ s}$) using a 4th-order Runge-Kutta (RK4) integration scheme.

---

### 🎯 Objective Function
The objective function tracks a squared difference penalty index across the trajectory. It minimizes the cumulative quadratic deviation between the state position and the velocity input effort at each grid node:

$$\min_{u} \quad J = \sum_{i=1}^{N} \left( u_i - x_i \right)^2 \cdot \Delta t$$

---

### 🛑 Boundary Conditions & Constraints

#### 1. Boundary Conditions
* **Initial State:** The tracking system begins with a positive unit displacement offset from the origin:
  $$x(0) = 1$$
* **Terminal State:** The final state $x(t_f)$ is left unconstrained (completely free at the terminal boundary).

#### 2. Path & Variable Constraints
* **State / Control Constraints:** None (unconstrained optimization problem with infinite actuator authority).

## Problem 12: Van der Pol Oscillator Stabilization (`VanderPol_Radau.m`)
*Reference: This problem is a classic nonlinear tracking benchmark evaluating the control of an unstable limit cycle oscillator, commonly featured in optimal control suites like Dymos.*

### ⚙️ System Dynamics
The system represents a controlled Van der Pol oscillator. The state vector is defined as $\mathbf{x} = [x_1, x_2]^T$, where $x_1$ represents the first derivative or velocity state and $x_2$ defines the primary position coordinate. Controlled by a scalar forcing input $u$, the non-conservative continuous-time dynamics are governed by:

$$\dot{x}_1 = (1 - x_2^2)x_1 - x_2 + u$$
$$\dot{x}_2 = x_1$$

#### 🧩 Collocation & Numerical Integration
Unlike traditional explicit Runge-Kutta formulations, this problem implements a fully implicit **2-stage, 3rd-order Radau IIA collocation method**. At each discretization step, the intermediate stage states ($\mathbf{x}_{s1}, \mathbf{x}_{s2}$) are solved simultaneously via an algebraic Newton-Raphson rootfinder (`casadi.rootfinder`) enforcing:

$$\mathbf{r}_1 = \mathbf{x}_{s1} - \left(\mathbf{x}_k + \Delta t \left[\frac{5}{12}\mathbf{f}(\mathbf{x}_{s1}, u_k) - \frac{1}{12}\mathbf{f}(\mathbf{x}_{s2}, u_k)\right]\right) = \mathbf{0}$$
$$\mathbf{r}_2 = \mathbf{x}_{s2} - \left(\mathbf{x}_k + \Delta t \left[\frac{3}{4}\mathbf{f}(\mathbf{x}_{s1}, u_k) + \frac{1}{4}\mathbf{f}(\mathbf{x}_{s2}, u_k)\right]\right) = \mathbf{0}$$

The system is evaluated over a fixed terminal time $T = 10.0 \text{ s}$ split into $N = 100$ segments ($\Delta t = 0.1 \text{ s}$).

---

### 🎯 Objective Function
The objective is an energy-penalizing tracking controller (Quadratic Regulator style stage cost) designed to suppress the oscillator's limit-cycle dynamics and minimize actuator expenditure over the grid:

$$\min_{u} \quad J = \sum_{i=1}^{N} \left( x_{1,i}^2 + x_{2,i}^2 + u_i^2 \right) \cdot \Delta t$$

---

### 🛑 Boundary Conditions & Constraints

#### 1. Boundary Conditions
* **Initial State:** The oscillator is initialized at a displaced state out of its natural equilibrium:
  $$x_1(0) = 1, \quad x_2(0) = 1$$
* **Terminal State (Pinpoint Equilibrium Rest):** The trajectory must strictly force both states to settle exactly at the origin origin by the final step $t_f$:
  $$x_1(t_f) = 0, \quad x_2(t_f) = 0$$

#### 2. Actuator Constraints (Box Constraints)
* **Control Input Limits:** The force command is bounded asymmetrically, restricting maximum directional thrust capabilities:
  $$-0.75 \leq u(t) \leq 1.0$$

## Problem 13: Single Integrator State-Control Tracking (`Single_Integrator_David_G_Hull.m`)
*Reference: This problem is taken from "Optimal Control Theory for Applications" by David G. Hull.*

### ⚙️ System Dynamics
The system represents a basic unconstrained linear Single Integrator operating under direct velocity control. The state vector is scalar, $\mathbf{x} = [x]$, tracking a single position variable. Controlled directly by the velocity input $u$ ($\text{m/s}$), the continuous-time kinematic relationship is defined as:

$$\dot{x} = u$$

The continuous kinematic system is discretized over a fixed execution horizon $t_f = 1.0 \text{ s}$ split into $N = 50$ intervals ($\Delta t = 0.02 \text{ s}$) using a 4th-order Runge-Kutta (RK4) numerical integration scheme.

---

### 🎯 Objective Function
The objective function tracks a quadratic regularization index across the grid. It balances the relationship between the vehicle state and its driving actuator input by minimizing the integrated squared difference between position and velocity:

$$\min_{u} \quad J = \sum_{i=1}^{N} \left( u_i - x_i \right)^2 \cdot \Delta t$$

---

### 🛑 Boundary Conditions & Constraints

#### 1. Boundary Conditions
* **Initial State:** The tracking system begins with a clear positive displacement offset from its target center line:
  $$x(0) = 1$$
* **Terminal State:** The final boundary node state $x(t_f)$ is left entirely unconstrained (completely free).

#### 2. Path & Variable Constraints
* **State / Control Constraints:** None (unconstrained optimization formulation allowing infinite actuator authority).


## Problem 14: Single Integrator Linear-Quadratic Trade-Off (`Single_Integrator_V2_David_G_Hull.m`)
*Reference: This problem variation is taken from "Optimal Control Theory for Applications" by David G. Hull.*

### ⚙️ System Dynamics
The system represents an unconstrained linear Single Integrator operating under direct velocity control. The state vector is scalar, $\mathbf{x} = [x]$, tracking a single position variable. Controlled directly by the velocity input $u$ ($\text{m/s}$), the continuous-time kinematic relationship is defined as:

$$\dot{x} = u$$

The continuous kinematic system is discretized over a fixed execution horizon $t_f = 1.0 \text{ s}$ split into $N = 50$ intervals ($\Delta t = 0.02 \text{ s}$) using a 4th-order Runge-Kutta (RK4) numerical integration scheme.

---

### 🎯 Objective Function
The objective function balances control effort regularization against state promotion. It minimizes the integrated sum of the squared control input minus the linear state value across the grid, encouraging the state to grow while penalizing quadratic control expenditure:

$$\min_{u} \quad J = \sum_{i=1}^{N} \left( u_i^2 - x_i \right) \cdot \Delta t$$

---

### 🛑 Boundary Conditions & Constraints

#### 1. Boundary Conditions
* **Initial State:** The tracking system begins with a positive unit displacement offset from its center origin:
  $$x(0) = 1$$
* **Terminal State:** The final boundary node state $x(t_f)$ is left unconstrained (completely free).

#### 2. Path & Variable Constraints
* **State / Control Constraints:** None (unconstrained optimization formulation allowing infinite actuator authority).







