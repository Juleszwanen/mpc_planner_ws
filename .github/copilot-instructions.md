# MPC Planner Workspace - AI Agent Instructions

## Project Overview

**T-MPC++ (Topology-Driven Model Predictive Control++)** - A ROS1 (Noetic) workspace for **multi-robot coordination** in dynamic environments using MPC and guidance planning. The system enables decentralized collision avoidance where each robot computes multiple topology-distinct trajectories and coordinates through selective communication.

### Primary Focus: Multi-Robot Coordination
- Decentralized planning: Each robot runs independent MPC
- Topology-based communication: Robots share trajectories only when needed
- Real-world + simulation: Code works on physical Jackal robots and Gazebo
- State machine architecture: Robust coordination through explicit state transitions

### Key Publications
- **T-RO 2024**: Topology-Driven Parallel Trajectory Optimization
- **ICRA 2023**: Globally Guided Trajectory Optimization
- **IJRR 2024**: Scenario-Based Trajectory Optimization with Bounded Probability of Collision (SH-MPC)

## Architecture

### Three-Tier System
1. **MPC Planner** (`src/mpc_planner/`) - Low-level trajectory optimization with modular cost/constraint components
2. **Guidance Planner** (`src/guidance_planner/`) - High-level sampling-based planner computing topology-distinct paths
3. **Robot-Specific Packages** - System implementations:
   - `mpc_planner_jackalsimulator/` - Multi-robot simulation (`jules_ros1_jackalplanner.cpp`) - uses ROS namespaces
   - `mpc_planner_jackal/` - Real-world multi-robot (`jules_ros1_real_jackalplanner.cpp`) - uses ZeroMQ middleware
   - `mpc_planner_rosnavigation/` - Single-robot ROS Navigation integration

### Deployment Architecture Differences

**Simulation (`jules_ros1_jackalplanner.cpp`)**:
- All robots in single ROS master
- ROS namespace-based topic isolation: `/<robot_ns>/robot_to_robot/output/...`
- Direct ROS pub/sub for robot-to-robot communication

**Real-World (`jules_ros1_real_jackalplanner.cpp`)**:
- Each robot runs planner in **separate container** with its own ROS master
- **No ROS namespacing** - each robot's planner uses absolute topic names
- **ZeroMQ middleware** (`guidance_planner_multi_robot_nodes`) enables direct container-to-container communication
- Central aggregator only handles synchronization (resetting robots when all reach objectives)
- Trajectory communication: Direct between robot containers via ZeroMQ pub/sub

### Core Components

```
mpc_planner/
├── mpc_planner/           # Core planning logic (planner.cpp)
├── mpc_planner_modules/   # Modular costs/constraints (auto-generated modules.h)
├── mpc_planner_solver/    # Solver interface (Acados/Forces Pro)
├── mpc_planner_types/     # Shared data structures
├── mpc_planner_util/      # YAML loading, parameters
└── mpc_planner_<system>/  # System-specific ROS wrappers
    ├── config/settings.yaml  # MPC configuration
    └── src/ros1_<system>.cpp # ROS node implementation
```

**Module System**: Modules are **auto-generated** by Python scripts during solver generation. The `modules.h` header is created by `generate_cpp_files.py` and includes `initializeModules()` factory function. Available modules:
- Contouring (MPCC/CA-MPC), Goal tracking, State/input costs
- Dynamic obstacle avoidance (ellipsoidal, linearized, chance-constrained)
- Static obstacle avoidance (decomp_util)
- T-MPC guidance constraints, Consistency tracking

## Critical Workflows

### Build Process (MUST follow order)
```bash
# 1. Generate solver (Python) - REQUIRED before C++ build
./build.sh <system> true  # e.g., jackalsimulator, jackal, rosnavigation

# 2. Build only (when solver exists)
./build.sh <system>

# 3. Build other packages
./build_other_packages.sh <package_name>
```

**Why**: Solver generation creates C++ code (`acados_solver_Solver.h`, `mpc_planner_parameters.h`, `modules.h`) that must exist before compilation. Use `Ctrl+Shift+B` in VSCode to access pre-configured tasks.

### Solver Generation
- **Acados** (open-source, default): Poetry environment in container handles dependencies
- **Forces Pro** (licensed, optional): Requires floating license for container use, regular license outside container
- Configuration: `settings.yaml` → `solver_settings/solver: "acados"` or `"forces"`
- Scripts: `mpc_planner_<system>/scripts/generate_<system>_solver.py`
- Output: `mpc_planner_solver/acados/` or `forces/` directories

### Running Simulations
```bash
# Via VSCode tasks (Ctrl+Shift+B):
- "JackalSimulator: Run Simulator"
- "ROS Navigation: Run Simulator"
- "Jackal: Run Real-World Jackal"

# Or manually:
source devel/setup.bash && roslaunch mpc_planner_jackalsimulator ros1_jackalsimulator.launch
```

## Configuration Patterns

### YAML Settings (`mpc_planner_<system>/config/settings.yaml`)
```yaml
N: 30                      # MPC horizon length
integrator_step: 0.2       # Time step (s)
control_frequency: 20      # Control loop Hz

solver_settings:
  solver: "acados"         # or "forces"
  acados:
    iterations: 10
    solver_type: SQP_RTI   # Real-time iteration

t-mpc:
  use_t-mpc++: true        # Enable non-guided parallel planner
  enable_constraints: true  # Homotopy constraints
  
weights:                   # Must match solver-declared weights
  velocity: 0.55
  acceleration: 0.34
  contour: 0.05
```

**Critical**: Weight names in YAML must exactly match those declared in solver generation scripts.

### Multi-Robot Communication Architecture

**Topic Naming Convention**:
```
# Simulation (namespaced):
/<robot_ns>/robot_to_robot/output/pose                    # Real-time pose updates
/<robot_ns>/robot_to_robot/output/current_trajectory      # Trajectory predictions

# Real-world (absolute paths, ZeroMQ-bridged):
/robot_to_robot/output/pose                               # Published by each robot
/robot_to_robot/output/current_trajectory                 # Published by each robot
```

**Communication Infrastructure**:
- **Simulation**: Direct ROS pub/sub within single ROS master, central aggregator for reset coordination
- **Real-World**: 
  - Direct container-to-container trajectory communication via ZeroMQ pub/sub
  - Each robot publishes trajectories that other robots subscribe to directly
  - Central aggregator (separate container) coordinates simultaneous resets only
  - See `guidance_planner_multi_robot_nodes` package for middleware implementation

**Communication Triggers** (topology-based filtering):
1. `INFEASIBLE` - Solver failed, must alert others
2. `TOPOLOGY_CHANGE` - Switched homotopy class (guided topologies only)
3. `GEOMETRIC` - Trajectory deviated beyond threshold from last communicated
4. `TIME` - Heartbeat timeout (fallback safety)
5. `CHOOSE_NON_GUIDED_MAPPING_HOMOLOGY_FAIL` - Fell back to non-guided planner
6. `NO_COMMUNICATION` - No trigger activated (saves bandwidth)

**Key Configuration** (`settings.yaml`):
```yaml
JULES:
  communicate_on_topology_switch_only: true  # Enable selective communication
  max_geometric_deviation: 0.5               # [m] Geometric trigger threshold
  heartbeat_time: 1.0                        # [s] Maximum silence duration
  enable_trajectory_interpolation: true      # Compensate for network delay
```

**Data Flow**:
- Other robots tracked as `DynamicObstacle` in `_data.trajectory_dynamic_obstacles` (keyed by namespace)
- Predictions interpolated forward in time to compensate for communication delays
- Ego trajectory published as `mpc_planner_msgs::ObstacleGMM` (encoded as moving obstacle)

## Project Conventions

### Code Organization
- **ROS abstraction**: `ros_tools` wraps ROS1/ROS2 for portability - use its interfaces, not direct ROS calls
- **Data flow**: `RealTimeData` (input) → `Planner::solveMPC()` → `PlannerOutput` (output)
- **Module pattern**: Inherit from `ControllerModule`, implement `update()` and `setParameters()`
- **Solver parameters**: Use auto-generated `setSolverParameter*()` functions in `mpc_planner_parameters.h`

### ROS1 Callback Patterns (Critical for Multi-Robot)

**State Machine Architecture** (`MPCPlanner::PlannerState`):
```cpp
UNINITIALIZED → TIMER_STARTUP → WAITING_FOR_FIRST_EGO_POSE 
  → INITIALIZING_OBSTACLES → WAITING_FOR_TRAJECTORY_DATA 
  → PLANNING_ACTIVE ⟷ GOAL_REACHED → RESETTING
```

**Callback Categories**:
1. **Ego State Updates** (`statePoseCallback`):
   - Direct `_state` update (x, y, psi, v)
   - Triggers transition from `WAITING_FOR_FIRST_EGO_POSE`
   
2. **Other Robot Pose** (`poseOtherRobotCallback`):
   - Updates `_data.trajectory_dynamic_obstacles[ns].position`
   - Immediate position updates (high frequency)
   
3. **Other Robot Trajectory** (`trajectoryCallback`):
   - Updates full prediction: `trajectory_dynamic_obstacles[ns].prediction`
   - Triggers state transition to `PLANNING_ACTIVE` when first meaningful data arrives
   - Uses `ros::Time::now()` for interpolation timestamps
   
4. **Main Control Loop** (`loop` - timer-driven):
   - Runs at `control_frequency` Hz (typically 20 Hz)
   - State-based dispatch: different behavior per `_current_state`
   - Calls `prepareObstacleData()` → `generatePlanningCommand()` → `publishCmdAndVisualize()`

**Thread Safety**: All callbacks run on ROS spinner thread - use `ros::spinOnce()` or `ros::spin()`

**Critical Pattern - Namespace-Based Indexing**:
```cpp
// Other robots stored by namespace string as key
std::map<std::string, DynamicObstacle> trajectory_dynamic_obstacles;

// Callbacks receive namespace parameter for lookup
void trajectoryCallback(const ObstacleGMM::ConstPtr& msg, const std::string ns) {
    auto& robot_obs = _data.trajectory_dynamic_obstacles[ns];  // Direct access
    // Update robot_obs.prediction...
}
```

### Naming Conventions
- System packages: `mpc_planner_<system>` (lowercase)
- Config files: `settings.yaml`, `<system>_params.yaml`
- Launch files: `ros1_<system>.launch`
- Scripts: `generate_<system>_solver.py`

### File Locations
- Launch files: `mpc_planner_<system>/launch/`
- Config: `mpc_planner_<system>/config/`
- ROS wrappers: `mpc_planner_<system>/src/ros1_*.cpp`
- Solver gen: `mpc_planner_<system>/scripts/generate_*.py`

## Development Environment

### Containerized Setup
- **Base**: Ubuntu 20.04, ROS Noetic
- **Setup**: `./setup.sh` - clones repos, installs dependencies, builds Acados
- **Poetry**: Manages Python dependencies for solver generation (auto-configured)
- **Environment vars**: `ACADOS_SOURCE_DIR=/workspace/acados`, `LD_LIBRARY_PATH` includes Acados libs

### Key Dependencies
- Acados solver (installed in `/workspace/acados/`)
- Eigen3 (C++ linear algebra)
- CasADi (Python symbolic math for solver generation)
- Catkin (ROS1 build system)

### VSCode Integration
- Tasks defined in `.vscode/tasks.json` (build, run, solver gen)
- `catkin config` sets: `RelWithDebInfo` build, Python 3, compile commands export
- Terminal: Source `/workspace/devel/setup.bash` before ROS commands

## Multi-Robot Coordination Patterns

### Initializing Multi-Robot System

**Simulation (namespaced topics)**:
```cpp
// 1. Extract other robot namespaces (excluding self)
std::set<std::string> other_robot_nss = 
    MultiRobot::extractOtherRobotNamespaces(_robot_ns_list, _ego_robot_ns);

// 2. Create trajectory obstacles for each robot
for (const auto& ns : other_robot_nss) {
    _data.trajectory_dynamic_obstacles.emplace(
        ns, 
        DynamicObstacle(MultiRobot::extractRobotIdFromNamespace(ns),
                       Eigen::Vector2d(100.0, 100.0),  // Far away initially
                       0.0, robot_radius));
}

// 3. Subscribe to each robot's topics (WITH namespace prefix)
for (const auto& ns : other_robot_nss) {
    std::string pose_topic = ns + "/robot_to_robot/output/pose";
    auto sub = nh.subscribe<PoseStamped>(pose_topic, 1,
        boost::bind(&YourClass::poseOtherRobotCallback, this, _1, ns));
    _pose_sub_list.push_back(sub);
}
```

**Real-World (absolute topics, ZeroMQ middleware)**:
```cpp
// Same initialization for trajectory obstacles
// But topics are absolute paths - ZeroMQ enables direct container-to-container pub/sub

// Subscribe to absolute topics (NO namespace prefix)
std::string pose_topic = "/robot_to_robot/output/pose";
auto sub = nh.subscribe<PoseStamped>(pose_topic, 1,
    boost::bind(&YourClass::poseOtherRobotCallback, this, _1, ns));

// Each robot container publishes on absolute topics
// ZeroMQ middleware bridges topics between containers (direct peer-to-peer)
// Robot filters messages by namespace parameter passed to callback
```

### Trajectory Interpolation (Compensating Network Delay)
```cpp
// Key: Use ros::Time for delay compensation
ros::Time now = ros::Time::now();
ros::Duration elapsed = now - traj_obs.last_trajectory_update_time;
double dt_interp = elapsed.toSec();

// Calculate shift indices: k full steps + fractional alpha
int k = static_cast<int>(std::floor(dt_interp / dt));
double alpha = (dt_interp - k * dt) / dt;  // [0, 1)

// Apply: Remove first k points, extrapolate new points, interpolate by alpha
// See jules_ros1_jackalplanner.cpp::interpolateTrajectoryPredictionsByTime()
```

### Communication Decision Logic
```cpp
bool shouldCommunicate(const PlannerOutput& output, const RealTimeData& data) {
    // Priority order matches enum values:
    if (CommunicationTriggers::checkInfeasible(output)) 
        return true;  // Always communicate failures
    
    if (CommunicationTriggers::checkTopologyChange(output, n_paths))
        return true;  // Switched homotopy class
    
    if (CommunicationTriggers::checkGeometricDeviation(
        output.trajectory, data.last_communicated_trajectory, threshold))
        return true;  // Deviated from last sent
    
    if (CommunicationTriggers::checkTime(
        data.last_send_trajectory_time, ros::Time::now(), heartbeat))
        return true;  // Heartbeat timeout
    
    return false;  // Stay silent (save bandwidth)
}
```

### State Transition Pattern
```cpp
// Use helper to log transitions consistently
MultiRobot::transitionTo(_current_state, _previous_state, 
                         MPCPlanner::PlannerState::PLANNING_ACTIVE, 
                         _ego_robot_ns);

// Check in main loop with switch statement
switch (_current_state) {
    case WAITING_FOR_TRAJECTORY_DATA:
        if (first_meaningful_data_received)
            MultiRobot::transitionTo(/*...PLANNING_ACTIVE*/);
        break;
    case PLANNING_ACTIVE:
        // Normal operation
        break;
}
```

## Common Patterns

### Adding a New Module
1. Create header/source in `mpc_planner_modules/`
2. Inherit from `ControllerModule`, set `ModuleType` (COST/CONSTRAINT)
3. Implement `update()` (called each control iteration), `setParameters()` (per MPC stage)
4. Register in solver generator (`control_modules.py`)
5. Rebuild solver: `./build.sh <system> true`

### Debugging MPC
- Enable in `settings.yaml`: `debug_output: true`, `debug_visuals: true`
- Visualization: Published on topics like `/mpc_planner/visualization/trajectory`
- Data recording: Set `recording/enable: true` → saves to `/workspace/experiment/`
- Logging: Use `LOG_INFO()`, `LOG_WARN()`, `LOG_ERROR()` from `ros_tools/logging.h`

### Modifying Robot Systems
- Create new package: `mpc_planner_<newsystem>/`
- Copy structure from `mpc_planner_jackalsimulator/` (multi-robot sim) or `mpc_planner_jackal/` (real-world)
- Update: `CMakeLists.txt`, `package.xml`, config YAML, launch file, ROS wrapper
- Create solver script: `scripts/generate_<newsystem>_solver.py`
- Add build task to `.vscode/tasks.json`

### Multi-Robot Launch Files

**Simulation Launch** (with namespaces):
```xml
<!-- Launch multiple robots with unique namespaces -->
<group ns="$(arg robot_ns)">
    <param name="ego_robot_ns" value="$(arg robot_ns)"/>
    <param name="robot_ns_list" value="[robot1, robot2, robot3]"/>
    <node pkg="mpc_planner_jackalsimulator" type="jules_jackalplanner" 
          name="jules_jackalplanner" output="screen">
        <!-- Topic remapping happens in launch file -->
        <remap from="input/state_pose" to="/$(arg robot_ns)/odometry/filtered_map_pose"/>
        <remap from="output/command" to="/$(arg robot_ns)/jackal_velocity_controller/cmd_vel"/>
    </node>
</group>
```

**Real-World Launch** (without namespaces, separate containers):
```xml
<!-- Each robot launches independently in its own container -->
<launch>
    <arg name="jackal_name" default="jackal1"/>
    <arg name="laptop_name" default="tu_delft"/>
    
    <param name="ego_robot_ns" value="$(arg jackal_name)"/>
    <param name="robot_ns_list" value="[jackal1, jackal2, jackal3]"/>
    
    <!-- NO namespace wrapping - absolute topic names -->
    <node pkg="mpc_planner_jackal" type="jules_real_jackalplanner" 
          name="jules_real_jackalplanner" output="screen">
        <remap from="/input/state" to="/odometry/filtered"/>
        <remap from="/output/command" to="/cmd_vel"/>
    </node>
    
    <!-- ZeroMQ middleware nodes for inter-robot communication -->
    <include file="$(find guidance_planner_multi_robot_nodes)/launch/launch_central_agg_middleware.launch">
        <arg name="laptop_name" value="$(arg laptop_name)"/>
    </include>
</launch>
```

## Important Notes

1. **Never edit generated files**: `modules.h`, `mpc_planner_parameters.h`, `acados_solver_*.{h,c}` are auto-generated
2. **Solver must be regenerated** when changing: modules, cost/constraint definitions, horizon length, state/input dims
3. **Switch ROS version**: Each repo has `switch_to_ros.py {1|2}` - modifies build files
4. **Multi-robot communication patterns differ by platform**:
   - **Simulation**: ROS namespaces (`/<robot_ns>/robot_to_robot/output/...`), single ROS master
   - **Real-world**: Absolute topics (`/robot_to_robot/output/...`), separate containers, ZeroMQ middleware
   - Both use same `_ego_robot_ns` variable for robot identification in message content
5. **Real robots deployment**:
   - Each robot runs in separate Docker container with own ROS master
   - Connect scripts in `connect_to_jackals/` configure `ROS_MASTER_URI` and network
   - ZeroMQ middleware enables direct container-to-container trajectory sharing
   - Central aggregator (separate container) coordinates simultaneous resets only
6. **Non-communicating objects**: Real-world code supports tracking non-communicating dynamic obstacles (e.g., humans) via Vicon
7. **Data saving**: Controlled per-robot with namespace-prefixed filenames when `_safe_extra_data=true`
8. **ROS timing**: Always use `ros::Time::now()` for timestamps (not `std::chrono`) - ensures simulation time compatibility
9. **Callback binding**: Use `boost::bind(&Class::callback, this, _1, extra_param)` to pass extra parameters to callbacks

## Troubleshooting

### Build Issues
- **"No rule to make target acados_solver"**: Run `./build.sh <system> true` to generate solver first
- **Poetry not found**: Re-run `./setup.sh`, answer 'y' when prompted to install
- **Solver interface mismatch**: Regenerate solver after changing modules or weights
- **Forces Pro license error**: Verify floating proxy running and `floating_license: true` in settings
- **Catkin build fails**: Check that `source /opt/ros/noetic/setup.sh` was run
- **Module not found**: Verify module registered in `control_modules.py` and solver regenerated

### Multi-Robot Runtime Issues
- **Robot stuck in WAITING_FOR_TRAJECTORY_DATA**: 
  - **Simulation**: Check other robots publishing to `/<their_ns>/robot_to_robot/output/current_trajectory`
  - **Real-world**: Verify ZeroMQ middleware running (`launch_central_agg_middleware.launch`)
  - Verify namespaces in `robot_ns_list` parameter match actual robot namespaces
  - Look for "trajectory_dynamic_obstacles.find(ns) == end()" warnings
  
- **Robots colliding despite coordination**:
  - Verify `communicate_on_topology_switch_only: true` config matches desired behavior
  - Check `max_geometric_deviation` threshold (increase = less communication = more risk)
  - Enable `debug_output: true` and check obstacle predictions being used
  
- **Network delay causing collisions**:
  - Enable `enable_trajectory_interpolation: true` in JULES config
  - Reduce `heartbeat_time` to force more frequent updates
  - Check actual message delays in logs: "rx_from_<ns>_delay_sec"
  - **Real-world**: Check ZeroMQ message routing latency with `rostopic hz`
  
- **State machine transitions failing**:
  - Use `LOG_INFO` with state names to trace transitions
  - Check `_validated_trajectory_robots.size()` matches expected robot count
  - Verify `_startup_timer->hasFinished()` before expecting transitions

- **Real-world specific issues**:
  - **No messages between robots**: Check ZeroMQ middleware running in each robot container
  - **ZeroMQ connection failed**: Verify network configuration in `network.yaml` and port forwarding
  - **Container communication**: Ensure containers can reach each other (same network or proper routing)
  - **Reset not triggering**: Verify central aggregator container running and publishing to `/all_robots_reached_objective`
  - **Vicon tracking lost**: Check `obstacleCallback` for non-communicating object updates

## References

- Main workspace: https://github.com/tud-amr/mpc_planner_ws
- MPC planner: https://github.com/tud-amr/mpc_planner  
- Guidance planner: https://github.com/tud-amr/guidance_planner
- Acados docs: https://docs.acados.org/
- Paper website: https://autonomousrobots.nl/paper_websites/topology-driven-mpc
