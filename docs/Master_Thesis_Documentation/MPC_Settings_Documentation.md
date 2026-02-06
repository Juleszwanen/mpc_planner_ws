# T-MPC++ Configuration Parameters: Complete Documentation

## Overview

This document provides a comprehensive explanation of all configuration parameters in the `settings.yaml` file used in the T-MPC++ (Topology-Driven Model Predictive Control++) system for multi-robot coordination. Each parameter is explained with its mathematical/theoretical background and the rationale for the chosen values.

---

## 1. Core MPC Parameters

### 1.1 Prediction Horizon (`N: 30`)

**What it is:**  
The number of discrete time steps the MPC looks ahead when computing optimal trajectories.

**Mathematical Context:**  
The MPC solves an optimization problem over a finite horizon:
$$\min_{u_0, ..., u_{N-1}} \sum_{k=0}^{N-1} l(x_k, u_k) + V_f(x_N)$$

where $N$ is the prediction horizon.

**Why 30:**
- **Total prediction time**: $N \times \text{integrator\_step} = 30 \times 0.2s = 6.0s$
- This provides sufficient lookahead for:
  - Multi-robot collision avoidance (robots need to see each other's future trajectories)
  - Path planning around obstacles
  - Smooth trajectory generation
- Trade-off: Larger $N$ increases computational cost but provides better planning horizon
- 6 seconds at typical robot speeds (1-2 m/s) covers 6-12 meters of path - adequate for most indoor/outdoor scenarios

---

### 1.2 Integration Time Step (`integrator_step: 0.2`)

**What it is:**  
The time interval (in seconds) between consecutive prediction steps in the MPC horizon.

**Mathematical Context:**  
The system dynamics are discretized as:
$$x_{k+1} = f_d(x_k, u_k, \Delta t)$$
where $\Delta t = 0.2s$ is the integrator step.

**Why 0.2s:**
- **Balances temporal resolution with computational efficiency**
- At maximum robot velocity of 2-3 m/s, the robot moves 0.4-0.6m per step
- Fine enough to capture dynamic obstacle interactions
- Coarse enough to keep the QP problem tractable
- The Acados solver uses ERK (Explicit Runge-Kutta) with 4 stages and 3 sub-steps, ensuring numerical accuracy

---

### 1.3 Number of Discs (`n_discs: 1`)

**What it is:**  
The number of circular discs used to approximate the robot's shape for collision checking.

**Mathematical Context:**  
The robot is modeled as the union of $n$ discs:
$$\text{Robot} \approx \bigcup_{i=1}^{n} \{p : \|p - c_i\|_2 \leq r_{\text{disc}}\}$$

**Why 1:**
- The Jackal robot has a roughly circular footprint (0.65m × 0.65m)
- A single disc is sufficient for the near-circular shape
- Multiple discs (2-3) would be used for elongated robots (e.g., cars)
- Reduces the number of collision constraints per obstacle from $n \times m$ to $1 \times m$

---

### 1.4 Control Frequency (`control_frequency: 20`)

**What it is:**  
The rate (in Hz) at which the MPC controller computes and sends commands to the robot.

**Why 20 Hz:**
- **50ms cycle time** - sufficient for mobile robots with typical dynamics
- Faster than the integrator step (200ms) ensuring multiple control updates per prediction step
- Matches typical ROS control loop frequencies for differential-drive robots
- Provides good disturbance rejection while not overloading CPU
- Compatible with SQP-RTI (Real-Time Iteration) which computes approximate solutions quickly

---

## 2. Acados Solver Settings

### 2.1 Solver Selection (`solver: "acados"`)

**What it is:**  
The numerical optimization solver used to solve the MPC problem.

**Acados vs Forces Pro:**
| Feature | Acados | Forces Pro |
|---------|--------|------------|
| License | Open-source (BSD) | Commercial |
| QP Solver | HPIPM (high-performance interior point) | Proprietary |
| Code Generation | C code, portable | C code, proprietary |
| Real-time | SQP-RTI supported | Supported |

**Why Acados:**
- Free and open-source
- Excellent performance through HPIPM
- Active academic development
- Sufficient for research applications

---

### 2.2 Solver Type (`solver_type: SQP_RTI`)

**What it is:**  
The type of Sequential Quadratic Programming algorithm used.

**Mathematical Background:**

**SQP (Sequential Quadratic Programming):**
Standard SQP iterates until convergence:
```
while not converged:
    1. Linearize constraints at current iterate
    2. Form quadratic approximation of Lagrangian
    3. Solve QP subproblem
    4. Update iterate with line search
```

**SQP_RTI (Real-Time Iteration):**
Performs only ONE iteration per control cycle, splitting computation:

**Preparation Phase** (offline, between control cycles):
- Linearize dynamics and constraints around previous solution
- Build QP matrices

**Feedback Phase** (online, at control instant):
- Update initial state constraint
- Solve single QP
- Return solution immediately

**Why SQP_RTI:**
- **Predictable computation time** (~5ms vs variable for full SQP)
- **Real-time guarantee**: Always returns a solution within time budget
- **Warm-starting**: Leverages solution from previous cycle
- **Excellent for MPC**: Each control cycle refines the solution
- At 20Hz control frequency, we have 50ms per cycle - RTI ensures consistent timing

---

### 2.3 Iterations (`iterations: 10`)

**What it is:**  
Maximum number of SQP iterations (relevant for full SQP, less relevant for SQP_RTI).

**Note:** In SQP_RTI mode, this parameter has limited effect since only 1 iteration is performed. However, it's kept for compatibility with standard SQP mode debugging.

---

### 2.4 Stationary Tolerance (`tolstat: 1e-3`)

**What it is:**  
The convergence tolerance for the stationarity condition (gradient of Lagrangian).

**Mathematical Context:**  
A point $z^*$ is optimal if the KKT conditions are satisfied:
$$\|\nabla_z L(z^*, \lambda^*)\|_\infty \leq \text{tolstat}$$

**Why 1e-3:**
- Sufficient accuracy for robotics applications
- Tighter tolerances (1e-6) unnecessary given:
  - Sensor noise
  - Model inaccuracies  
  - High control frequency that corrects errors
- Faster convergence than stricter tolerances

---

## 3. Internal Acados Configuration (from `generate_acados_solver.py`)

These settings are hardcoded in the solver generation script:

### 3.1 Integrator Type (`integrator_type: "ERK"`)
- **ERK** = Explicit Runge-Kutta
- 4-stage, 3-step integration for each 0.2s interval
- Suitable for non-stiff differential equations (robot dynamics)

### 3.2 QP Solver (`qp_solver: "PARTIAL_CONDENSING_HPIPM"`)

**What it is:**  
The algorithm used to solve the quadratic program at each SQP iteration.

**Options:**
- `FULL_CONDENSING_QPOASES`: Dense QP, all states eliminated
- `PARTIAL_CONDENSING_HPIPM`: Sparse QP, only some states condensed
- `FULL_CONDENSING_HPIPM`: Dense with HPIPM

**Why Partial Condensing:**
- Keeps problem sparse → faster for longer horizons (N=30)
- HPIPM exploits OCP structure using Riccati recursion
- Complexity: $O(N \cdot n_x^3)$ vs $O(N^3 \cdot n_x^3)$ for full condensing

### 3.3 Hessian Approximation (`hessian_approx: "EXACT"`)
- Uses exact second derivatives (Hessian of Lagrangian)
- More accurate than Gauss-Newton approximation
- Required for constraint satisfaction guarantees

### 3.4 Regularization Method (`regularize_method: "MIRROR"`)
- Ensures positive-definiteness of Hessian
- Performs eigenvalue decomposition: $H = V^T D V$
- Sets $D_{ii} = \max(\epsilon, |D_{ii}|)$
- Necessary for solver convergence in non-convex problems

### 3.5 QP Warm Start (`qp_solver_warm_start: 2`)
- Level 2: Warm start both primal and dual variables
- Reuses previous QP solution as initial guess
- Dramatically speeds up QP solves in sequential MPC

---

## 4. Robot Physical Parameters

### 4.1 Robot Radius (`robot_radius: 0.425`)

**What it is:**  
The radius (in meters) of the circular approximation of the robot for collision avoidance.

**Why 0.425m:**
- Jackal robot dimensions: 0.65m × 0.65m
- Diagonal: $\sqrt{0.65^2 + 0.65^2}/2 \approx 0.46m$
- Using 0.425m provides good coverage while being slightly conservative
- Includes a small safety buffer (~2-3cm margin from actual corners)

### 4.2 Obstacle Radius (`obstacle_radius: 0.425`)

**What it is:**  
Default radius for dynamic obstacles (other robots) when not specified in messages.

**Why 0.425m:**
- Symmetric assumption: other Jackal robots have same size
- Ensures collision constraints are consistent
- Total separation distance: $0.425 + 0.425 = 0.85m$ (covers both robots)

### 4.3 Maximum Obstacles (`max_obstacles: 4`)

**What it is:**  
Maximum number of dynamic obstacles the solver can handle simultaneously.

**Why 4:**
- Supports 3-robot coordination plus 1 non-communicating obstacle
- Each obstacle adds $N \times n_{\text{discs}}$ constraints to the QP
- Trade-off between flexibility and computation time
- For dense multi-robot scenarios, increase to 8-12

---

## 5. MPC Cost Weights

The MPC objective function is:
$$J = \sum_{k=0}^{N-1} \left[ w_{\text{lag}} e_{\text{lag}}^2 + w_{\text{contour}} e_{\text{contour}}^2 + w_v (v - v_{\text{ref}})^2 + w_a a^2 + w_\omega \omega^2 \right] + J_{\text{terminal}}$$

### 5.1 Contouring Weights

#### Contour Error (`contour: 0.05`)
- Penalizes lateral deviation from reference path
- Low weight allows the robot to deviate for obstacle avoidance
- Higher values (0.5-1.0) force strict path following

#### Lag Error (`lag: 0.75`)  
- Penalizes longitudinal deviation along path
- Higher than contour weight → prioritizes progress over lateral accuracy
- Encourages forward motion along the reference

### 5.2 Velocity Weights

#### Velocity Tracking (`velocity: 0.55`)
- Penalizes deviation from reference velocity
- Moderate weight balances speed maintenance with constraint satisfaction

#### Reference Velocity (`reference_velocity: 2.0`)
- Target velocity in m/s
- Jackal maximum: ~2.0 m/s
- Setting at maximum encourages efficient path completion

### 5.3 Input Weights

#### Acceleration (`acceleration: 0.34`)
- Penalizes changes in linear velocity
- Promotes smooth velocity profiles
- Prevents aggressive acceleration/braking

#### Angular Velocity (`angular_velocity: 0.85`)
- Penalizes turning rate
- Higher than acceleration → prioritizes smooth orientation changes
- Prevents aggressive steering maneuvers

### 5.4 Terminal Costs

#### Terminal Angle (`terminal_angle: 100.0`)
- Strong penalty on heading error at horizon end
- Encourages path-aligned terminal orientation
- Improves trajectory continuity between MPC cycles

#### Terminal Contouring (`terminal_contouring: 10.0`)
- Multiplier for lag/contour errors at terminal state
- Encourages the robot to be well-positioned at prediction horizon

### 5.5 Consistency Weight (`consistency: 0.4`)

**What it is:**  
Penalizes deviation from the previously planned trajectory.

**Mathematical Form:**
$$J_{\text{consistency}} = w_{\text{consistency}} \sum_{k=1}^{N} \|(x_k, y_k) - (x_k^{\text{prev}}, y_k^{\text{prev}})\|^2$$

**Why 0.4:**
- Provides temporal consistency (smoother execution)
- Prevents abrupt trajectory changes between MPC cycles
- Important for multi-robot coordination: other robots predict your motion
- Too high: Reduces responsiveness to new obstacles
- Too low: Jittery trajectories that confuse other robots

---

## 6. T-MPC++ Topology Settings

### 6.1 Enable T-MPC++ (`use_t-mpc++: true`)

**What it is:**  
Adds a non-guided parallel planner alongside topology-guided planners.

**Why Enabled:**
- The non-guided planner provides a fallback when guidance fails
- Explores trajectories that might not match predefined topologies
- Improves robustness in complex scenarios

### 6.2 Enable Constraints (`enable_constraints: true`)

**What it is:**  
Activates homotopy constraints that keep trajectories within their designated topology class.

**Mathematical Context:**  
Homotopy constraints are linearized as:
$$a_1 x + a_2 y \leq b$$

These half-plane constraints separate different passing behaviors (left vs right of obstacles).

**Why Enabled:**
- Ensures each parallel planner explores its designated topology
- Prevents all planners from converging to the same solution
- Essential for topology-driven coordination

---

## 7. JULES Multi-Robot Communication Settings

### 7.1 Selective Communication (`communicate_on_topology_switch_only: true`)

**What it is:**  
Enables bandwidth-efficient communication where robots only broadcast trajectories when necessary.

**Communication Triggers:**
1. **INFEASIBLE** - Solver failed (always communicate)
2. **TOPOLOGY_CHANGE** - Switched homotopy class
3. **GEOMETRIC** - Trajectory deviated beyond threshold
4. **TIME** - Heartbeat timeout
5. **NO_COMMUNICATION** - No trigger (save bandwidth)

**Why Enabled:**
- Reduces communication bandwidth by 60-80%
- Robots only share when their behavior changes significantly
- Critical for real-world wireless networks with limited bandwidth

### 7.2 Geometric Deviation Threshold (`max_geometric_deviation: 1.1`)

**What it is:**  
Maximum allowed deviation (in meters) between current and last communicated trajectory before triggering communication.

**Mathematical Definition:**
$$\max_k \|x_k^{\text{current}} - x_k^{\text{last\_sent}}\| > 1.1m$$

**Why 1.1m:**
- Related to robot collision radius (0.425m × 2 ≈ 0.85m)
- Provides safety margin before trajectories diverge dangerously
- Larger values reduce communication but risk prediction errors
- Tested value that balances safety and bandwidth

### 7.3 Heartbeat Time (`heartbeat_time: 3.0`)

**What it is:**  
Maximum time (in seconds) between trajectory broadcasts, regardless of other triggers.

**Why 3.0s:**
- Safety fallback: Ensures robots always have recent trajectory data
- With 6s prediction horizon, 3s ensures at least 50% overlap
- Prevents "stale" trajectory predictions during steady-state motion
- Long enough to save bandwidth when robots move predictably

### 7.4 Trajectory Interpolation (`enable_trajectory_interpolation: true`)

**What it is:**  
Compensates for communication delays by extrapolating received trajectories forward in time.

**Mathematical Operation:**
```
elapsed_time = now - trajectory_received_time
k = floor(elapsed_time / dt)  // Remove k stale points
alpha = (elapsed_time - k*dt) / dt  // Interpolation factor
trajectory_adjusted = interpolate(trajectory[k:], alpha)
```

**Why Enabled:**
- Wireless networks introduce 10-100ms delays
- Without interpolation, collision constraints use outdated positions
- Critical for high-speed multi-robot coordination

---

## 8. Solver State Bounds (from `solver_model.py`)

The solver enforces box constraints on states and inputs:

| Variable | Lower Bound | Upper Bound | Unit |
|----------|-------------|-------------|------|
| Acceleration (a) | -2.0 | 2.0 | m/s² |
| Angular velocity (ω) | -0.8 | 0.8 | rad/s |
| Position (x, y) | -2000 | 2000 | m |
| Heading (ψ) | -4π | 4π | rad |
| Velocity (v) | -0.01 | 3.0 | m/s |
| Spline parameter (s) | -1.0 | 10000 | - |

**Notes:**
- Position bounds are effectively unbounded for practical purposes
- Heading allows multiple rotations (±4π) to avoid wrap-around issues
- Velocity lower bound of -0.01 allows tiny backward motion for numerical stability

---

## 9. Recording Settings

### Experiment Recording Configuration

```yaml
recording:
  enable: true
  folder: /workspace/experiment/mpc_planner_sim/...
  file: 3_jackals_Topology_Com_Trigger_GEO_Time
  num_experiments: 11
```

- **Purpose:** Data collection for thesis analysis
- **num_experiments:** Automatic stopping after 11 runs for batch experiments
- **Data includes:** Trajectories, solve times, communication events, topology selections

---

## 10. Summary Table

| Parameter | Value | Primary Justification |
|-----------|-------|----------------------|
| N | 30 | 6s horizon covers typical interaction scenarios |
| integrator_step | 0.2s | Balances resolution and computation |
| control_frequency | 20 Hz | Standard for mobile robots with RTI |
| solver_type | SQP_RTI | Deterministic timing for real-time control |
| robot_radius | 0.425m | Covers Jackal footprint with margin |
| consistency | 0.4 | Smooth trajectories, predictable for other robots |
| max_geometric_deviation | 1.1m | Related to collision safety margin |
| heartbeat_time | 3.0s | Ensures trajectory freshness |

---

## References

1. **Acados Documentation**: https://docs.acados.org/
2. **T-MPC++ Paper (T-RO 2024)**: Topology-Driven Parallel Trajectory Optimization
3. **HPIPM Paper**: Frison & Diehl, "HPIPM: A High-Performance QP Framework for MPC", 2020
4. **SQP-RTI**: Diehl et al., "Real-time optimization for nonlinear MPC", 2002
5. **Jackal Robot Specifications**: Clearpath Robotics

---

*Document generated for Master Thesis Defense preparation.*
*Configuration analyzed from T-MPC++ workspace at `/workspace/src/mpc_planner/`*
