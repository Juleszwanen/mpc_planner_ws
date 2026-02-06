# Guidance Planner Algorithm Documentation

## Table of Contents
1. [Overview](#overview)
2. [Algorithm Summary](#algorithm-summary)
3. [Visibility-PRM: Graph Construction](#visibility-prm-graph-construction)
   - [First Iteration](#first-iteration)
   - [Consecutive Iterations](#consecutive-iterations)
4. [Depth-First Search (DFS)](#depth-first-search-dfs)
5. [FilterAndSelect](#filterandselect)
6. [IdentifyAndPropagate](#identifyandpropagate)
7. [Topology Comparison Methods](#topology-comparison-methods)
8. [Data Structures](#data-structures)
9. [Configuration Parameters](#configuration-parameters)

---

## Overview

The **Guidance Planner** is a high-level sampling-based planner that computes multiple **topology-distinct trajectories** through space-time (x, y, t). Unlike traditional path planners that find a single optimal path, the Guidance Planner explicitly explores different ways of navigating around dynamic obstacles (different "homotopy classes") and provides multiple candidate trajectories to the downstream MPC solver.

### Why Topology-Distinct Paths?

In multi-robot coordination and dynamic environments, there are often multiple valid ways to reach a goal:
- Pass **left** of an obstacle
- Pass **right** of an obstacle
- Pass **before** an obstacle (temporally)
- Pass **after** an obstacle (temporally)

Each of these represents a different **topology class**. The Guidance Planner finds paths from each distinct topology class, enabling:
1. **Parallel trajectory optimization**: MPC can optimize trajectories for multiple topologies simultaneously
2. **Robust planning**: If one topology becomes infeasible, the system can switch to another
3. **Deadlock avoidance**: Multi-robot systems can coordinate by selecting compatible topologies

---

## Algorithm Summary

The Guidance Planner follows this high-level algorithm (Algorithm 1 from the T-MPC++ paper):

```
Algorithm 1: Guidance Planner
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Input: Configuration space C, start x₀, goal(s) xₙ, previous graph G⁻, 
       previous trajectories T_p⁻

1. G ← Visibility-PRM(C, x₀, xₙ, G⁻)     // Build/update space-time graph
2. {τ₀, ..., τ_nas} ← DepthFirstSearch(G) // Find all paths through graph
3. T_p = {τ₀, ..., τ_p} ← FilterAndSelect({τ₀, ..., τ_nas})  // Keep distinct topologies
4. G⁻ ← IdentifyAndPropagate({τ₀, ..., τ_p}, T_p⁻)  // Prepare for next iteration

Output: T_p (topology-distinct trajectories)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### Implementation Flow

The main entry point is `GlobalGuidance::Update()` in [global_guidance.cpp](../../src/guidance_planner/src/global_guidance.cpp):

```cpp
bool GlobalGuidance::Update() {
    // 1. Load obstacle data into PRM
    prm_.LoadData(obstacles_, static_obstacles_, start, orientation, velocity, goals_);
    
    // 2. Build Visibility-PRM graph
    Graph &graph = prm_.Update();
    
    // 3. Depth-First Search for each goal
    for (auto &goal : goals) {
        graph_search_.Search(graph, config_->n_paths_, L, cur_paths[g], goal_node);
        // ... collect paths from different goals
    }
    
    // 4. Sort paths by cost (heuristic)
    std::sort(paths.begin(), paths.end(), [](a, b) { return cost(a) < cost(b); });
    
    // 5. Keep topology-distinct paths
    KeepTopologyDistinctPaths(paths_);
    
    // 6. Propagate graph for next iteration
    prm_.PropagateGraph(paths_);
    
    // 7. Identify previous homologies for consistency
    IdentifyPreviousHomologies(outputs_);
    
    return true;
}
```

---

## Visibility-PRM: Graph Construction

### Space-Time Graph Concept

The Visibility-PRM constructs a graph in **3D space-time** (x, y, t) where:
- **x, y**: Spatial coordinates
- **t**: Discrete time index k ∈ [0, N] where N is the MPC horizon

This space-time representation is crucial because it captures the **temporal nature** of dynamic obstacle avoidance. A path through this graph represents not just *where* to go, but *when* to be at each location.

### Node Types

The graph contains three types of nodes (defined in [node.h](../../src/guidance_planner/include/guidance_planner/types/node.h)):

| Node Type | Description | Visual Color |
|-----------|-------------|--------------|
| **GUARD** | Isolated samples that "see" no other guards. Forms the skeleton of the graph. | Orange |
| **CONNECTOR** | Bridges two guards that cannot directly see each other. Creates alternative paths. | Colored by path ID |
| **GOAL** | Terminal nodes representing goal positions at the end of the horizon (k = N). | Orange |

```cpp
enum class NodeType {
    NONE = 0,
    GUARD = 1,
    CONNECTOR = 2,
    GOAL = 3
};
```

### Visibility Definition

Two points are **visible** to each other if the line connecting them does not collide with any obstacle trajectory in space-time. This is checked by `Environment::IsVisible()`:

```
Visibility(p₁, p₂) = True iff the line segment (x₁,y₁,t₁) → (x₂,y₂,t₂) 
                      is collision-free in space-time
```

### First Iteration

In the **first iteration**, the graph is built from scratch:

#### Step 1: Initialize Graph
```cpp
void PRM::Update() {
    // Create start node (id = -1, special case)
    graph_->start_node_ = graph_->AddNode(Node(-1, start_, NodeType::GUARD));
    
    // Create goal nodes (id = -2, -3, ... for multiple goals)
    for (int g = 0; g < goals_.size(); g++) {
        SpaceTimePoint goal_point(goals_[g].pos(0), goals_[g].pos(1), Config::N);
        Node *goal_node = graph_->AddNode(Node(-2 - g, goal_point, NodeType::GOAL));
        graph_->goal_nodes_.push_back(goal_node);
    }
}
```

#### Step 2: Sample Points
The sampler draws `n_samples` points uniformly in the space-time region between start and goals:

```cpp
Sample &Sampler::SampleUniformly(int sample_index) {
    Sample &sample = samples_[sample_index];
    
    // Sample x, y uniformly in [min, max] range
    for (int i = 0; i < SpaceTimePoint::numPositions(); i++)
        sample.point(i) = min_(i) + random_generator_.Double() * range_(i);
    
    // Sample discrete time k ∈ [1, N-1] (excluding endpoints)
    sample.point.SetTime(random_generator_.Int(Config::N - 2) + 1);
    
    return sample;
}
```

#### Step 3: Visibility-PRM Classification

For each sample, the algorithm classifies it as a **Guard** or **Connector**:

```cpp
void PRM::SampleNewPoints() {
    for (int i = 0; i < num_samples; i++) {
        SpaceTimePoint sample = sampler_->DrawSample(i).point;
        
        if (!environment_->InCollision(sample)) {
            // Find all guards visible from this sample
            std::vector<Node *> visible_guards;
            FindVisibleGuards(sample, visible_guards);
            
            AddSample(i, sample, visible_guards, false);
        }
    }
}
```

The `AddSample()` function applies the Visibility-PRM rules:

```cpp
void PRM::AddSample(int i, SpaceTimePoint &sample, 
                    const std::vector<Node *> guards,
                    bool sample_is_from_previous_iteration) {
    
    // CASE 1: No guards visible → This is a new GUARD
    if (guards.empty()) {
        AddGuard(i, sample);
    }
    // CASE 2: Exactly one guard visible → Not useful, discard
    else if (guards.size() == 1) {
        // Discard - doesn't add topological information
    }
    // CASE 3: Two or more guards visible → Potential CONNECTOR
    else {
        Node new_node(graph_->GetNodeID(), sample, NodeType::CONNECTOR);
        
        // Check if guards can already see each other
        std::vector<Node *> shared = graph_->GetSharedNeighbours(guards);
        
        if (shared.empty()) {
            // Guards can't see each other → ADD new connector
            AddNewConnector(new_node, guards);
        } else {
            // Guards already connected → REPLACE if this path is better
            ReplaceConnector(new_node, shared[0], guards);
        }
    }
}
```

#### Visual Representation of Classification

```
    G₁ ─────────── G₂        G₁           G₂        G₁ ─── C₁ ─── G₂
                              \          /              \    |    /
         No sample               Sample S                  Sample S
         needed                  (new connector)           (better connector)
         
    Case: Guards visible      Case: Guards NOT          Case: Guards connected
    to each other             visible to each other     but new path is shorter
```

#### Guard Addition
```cpp
void PRM::AddGuard(int i, SpaceTimePoint &sample) {
    Node &new_guard = *graph_->AddNode(
        Node(graph_->GetNodeID(), sample, NodeType::GUARD)
    );
    
    // Check if this guard can see any goals
    for (int g = 0; g < goals_.size(); g++) {
        if (IsGoalVisible(sample, g)) {
            // Connect guard to goal
            new_guard.neighbours_.push_back(graph_->goal_nodes_[g]);
            graph_->goal_nodes_[g]->neighbours_.push_back(&new_guard);
        }
    }
}
```

#### Connector Addition
```cpp
void PRM::AddNewConnector(Node &new_node, 
                          const std::vector<Node *> &visible_guards) {
    Node *added_node = graph_->AddNode(new_node);
    
    // Connect to all visible guards
    for (auto &guard : visible_guards) {
        added_node->neighbours_.push_back(guard);
        guard->neighbours_.push_back(added_node);
    }
}
```

#### Connector Replacement Logic

When a sample sees guards that are already connected, the algorithm checks if the new path is **better** (shorter/smoother):

```cpp
void PRM::ReplaceConnector(Node &new_node, Node *existing_connector,
                           const std::vector<Node *> &visible_guards) {
    // Check if new connector provides a better path
    if (prm_.FirstPathIsBetter(new_path, existing_path)) {
        // Mark old connector as replaced
        existing_connector->replaced_ = true;
        
        // Add new connector
        AddNewConnector(new_node, visible_guards);
    }
}
```

### Consecutive Iterations

In subsequent iterations, the algorithm **preserves and propagates** the graph from the previous iteration to ensure **temporal consistency**. This is critical because:

1. The robot has moved forward in time
2. Obstacles have new predictions
3. Previously found topologies should be preserved if still valid

#### Graph Propagation Overview

```
Iteration t:                    Iteration t+1:
                               
  k=0  k=1  k=2  k=3  k=N        k=0  k=1  k=2  k=3  k=N
   S────●────●────●────G          S────●────●────●────G
        │    │    │                    │    │    │
   ─────●────●────●─────     →    ─────●────●────●─────
        │    │    │                    │    │    │
   ─────●────●────●────G          ─────●────●────●────G
   
   Nodes shift left (time decreases)
   Nodes at k<0 are re-sampled or discarded
```

#### PropagateGraph Implementation

After the filtering step, the algorithm propagates selected paths:

```cpp
void PRM::PropagateGraph(const std::vector<GeometricPath> &paths) {
    if (do_not_propagate_nodes_)
        return;
    
    previous_nodes_.clear();
    
    // Mark which nodes belong to which path (for visualization)
    for (size_t p = 0; p < paths.size(); p++) {
        for (auto &node : paths[p].GetNodes()) {
            node->belongs_to_path_ = p;
        }
    }
    
    // Propagate all nodes in the graph
    for (auto &node : graph_->nodes_) {
        // Find which path this node belongs to (if any)
        const GeometricPath *belonging_path = nullptr;
        for (auto &path : paths) {
            if (path.ContainsNode(node)) {
                belonging_path = &path;
                break;
            }
        }
        
        PropagateNode(node, belonging_path);
    }
}
```

#### PropagateNode: Time Shifting

Each node is shifted backward in time by the elapsed time step:

```cpp
void PRM::PropagateNode(const Node &node, const GeometricPath *path) {
    // Shift time index: k_new = k_old - 1
    double new_time = node.point_.Time() - 1.0;
    
    // CASE 1: Node still in future (k >= 1)
    if (new_time >= 1.0) {
        SpaceTimePoint new_point(node.point_.Pos(), new_time);
        
        // Check if still collision-free
        if (!environment_->InCollision(new_point)) {
            previous_nodes_.emplace_back(node.id_, new_point, node.type_);
        }
    }
    // CASE 2: Node in past or present (k < 1)
    else if (path != nullptr) {
        // Re-sample along the path at k = 1
        SpaceTimePoint new_point = (*path)(1.0 / Config::N);  // s = 1/N maps to k=1
        previous_nodes_.emplace_back(node.id_, new_point, node.type_);
    }
    // CASE 3: Node expired and not on any path → Discard
}
```

#### Using Previous Nodes in Update

At the start of `PRM::Update()`, previous nodes are added first:

```cpp
Graph &PRM::Update() {
    graph_->Clear();
    
    // Add start and goals
    InitializeStartAndGoals();
    
    // FIRST: Add nodes from previous iteration
    for (auto &prev_node : previous_nodes_) {
        if (!environment_->InCollision(prev_node.point_)) {
            std::vector<Node *> visible_guards;
            FindVisibleGuards(prev_node.point_, visible_guards);
            
            // Reuse previous node classification
            AddSample(-1, prev_node.point_, visible_guards, 
                      true);  // sample_is_from_previous_iteration = true
        }
    }
    
    // THEN: Sample new points to fill gaps
    SampleNewPoints();
    
    return *graph_;
}
```

This two-phase approach ensures:
1. **Consistency**: Nodes from previous paths are prioritized
2. **Completeness**: New samples explore areas not covered by previous nodes
3. **Efficiency**: Less sampling needed when previous graph is still valid

---

## Depth-First Search (DFS)

After the graph is constructed, the algorithm finds all paths from start to each goal using **Depth-First Search**. The implementation is in [graph_search.cpp](../../src/guidance_planner/src/graph_search.cpp).

### Algorithm Description

```
DFS(graph, max_paths, L, T, goal):
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Input: graph G, maximum paths to find, current path L, found paths T, goal node

1. l ← last node in L
2. FOR each neighbour n of l:
   a. IF n.time < l.time: CONTINUE  // Enforce forward-in-time direction
   b. IF n ∈ L: CONTINUE            // Avoid cycles
   c. IF n == goal:
      - L.push(n)
      - T.push(L)                   // Found a complete path!
      - L.pop()
      - BREAK                       // Only one path per goal from this branch
   
3. FOR each neighbour n of l:
   a. IF n.time < l.time: CONTINUE
   b. IF n ∈ L OR n == goal: CONTINUE
   c. L.push(n)
   d. DFS(graph, max_paths, L, T, goal)  // Recursive call
   e. L.pop()
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### Key Properties

1. **Directed Edges**: Only follows edges where `neighbour.time > current.time`, making the graph a **DAG** (Directed Acyclic Graph) in time
2. **No Cycle Detection Needed**: The time-ordering naturally prevents cycles
3. **Early Termination**: Stops when `max_paths` are found
4. **Path Collection**: Each complete path (start → goal) is stored as a `GeometricPath`

### Implementation

```cpp
void GraphSearch::Search(const Graph &graph, unsigned int max_paths,
                         std::vector<Node *> &L, 
                         std::vector<GeometricPath> &T, 
                         const Node *goal) {
    // Stop if enough paths found
    if (T.size() >= max_paths)
        return;
    
    Node *l = L.back();  // Current node
    
    // Check for goal among neighbours
    for (auto &neighbour : l->neighbours_) {
        if (neighbour->point_.Time() < l->point_.Time())  // Time direction
            continue;
        if (HasBeenVisited(L, neighbour))
            continue;
        
        if (goal->id_ == neighbour->id_) {
            L.push_back(neighbour);
            T.emplace_back(L);  // Store complete path
            L.pop_back();
            break;
        }
    }
    
    // Recursive exploration
    for (auto &neighbour : l->neighbours_) {
        if (neighbour->point_.Time() < l->point_.Time())
            continue;
        if (HasBeenVisited(L, neighbour) || goal->id_ == neighbour->id_)
            continue;
        
        L.push_back(neighbour);
        Search(graph, max_paths, L, T, goal);  // Recurse
        L.pop_back();
    }
}
```

### Example DFS Execution

```
Graph:                    DFS Execution (start=S, goal=G):
                          
     G                    L = [S]
    /|\                   L = [S, A] → explore A's neighbours
   A B C                  L = [S, A, G] → FOUND PATH 1: S→A→G
    \|/                   L = [S, A]
     S                    L = [S, B] → explore B's neighbours
                          L = [S, B, G] → FOUND PATH 2: S→B→G
                          L = [S, B]
                          L = [S, C] → explore C's neighbours
                          L = [S, C, G] → FOUND PATH 3: S→C→G
                          
Result: T = [{S→A→G}, {S→B→G}, {S→C→G}]
```

---

## FilterAndSelect

After DFS finds multiple paths, many may be **topologically equivalent** (same homotopy class). The `KeepTopologyDistinctPaths()` function filters these to keep only distinct topologies.

### Algorithm

```
FilterAndSelect(paths):
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Input: Sorted list of paths (by cost/heuristic)
Output: Up to n_paths topology-distinct paths

1. distinct_paths ← [paths[0]]     // First path is always kept (best cost)

2. FOR i = 1 to len(paths):
   a. candidate ← paths[i]
   b. is_distinct ← TRUE
   
   c. FOR each path p in distinct_paths:
      - IF AreHomotopicEquivalent(candidate, p):
        - is_distinct ← FALSE
        - BREAK
   
   d. IF is_distinct:
      - distinct_paths.push(candidate)
   
   e. IF len(distinct_paths) >= n_paths:
      - BREAK

3. RETURN distinct_paths
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### Implementation

```cpp
void GlobalGuidance::KeepTopologyDistinctPaths(std::vector<GeometricPath> &paths) {
    if (paths.size() == 0)
        return;
    
    // Paths are pre-sorted by cost - first occurrence is the best
    std::vector<GeometricPath> topology_distinct_paths;
    topology_distinct_paths.emplace_back(paths.front());  // Best path always kept
    
    for (size_t i = 1; i < paths.size() && 
         topology_distinct_paths.size() < config_->n_paths_; i++) {
        
        auto &candidate_path = paths[i];
        bool distinct = true;
        
        // Check against all already-selected paths
        for (auto &path : topology_distinct_paths) {
            if (prm_.AreHomotopicEquivalent(path, candidate_path)) {
                distinct = false;
                break;
            }
        }
        
        if (distinct)
            topology_distinct_paths.emplace_back(paths[i]);
    }
    
    paths = topology_distinct_paths;
}
```

### Path Sorting (Cost Heuristic)

Before filtering, paths are sorted by a **cost function** that prefers:
1. Paths reaching lower-cost goals
2. Shorter path length (3D through space-time)

```cpp
double GlobalGuidance::PathSelectionCost(const GeometricPath &path) {
    // Goal cost (priority) + path smoothness
    return 1000.0 * Goal::FindGoalWithNode(*prm_.GetGoals(), path.GetEnd()).cost 
           - path.Length3D();
}
```

The heuristic-based ordering ensures:
```cpp
std::sort(paths.begin(), paths.end(), [&](const GeometricPath &a, const GeometricPath &b) {
    return PathSelectionCost(a) < PathSelectionCost(b);
});
```

### Example Filter Operation

```
Input paths (sorted by cost):
  1. S → A → G  (cost: 5.2)  - Pass LEFT of obstacle
  2. S → B → G  (cost: 5.5)  - Pass LEFT of obstacle (same topology as #1)
  3. S → C → G  (cost: 6.0)  - Pass RIGHT of obstacle
  4. S → D → G  (cost: 6.5)  - Pass RIGHT of obstacle (same topology as #3)

After FilterAndSelect (n_paths = 2):
  1. S → A → G  - Pass LEFT (kept: best path)
  2. S → C → G  - Pass RIGHT (kept: distinct topology)
  
Paths 2 and 4 are discarded as homotopically equivalent to better paths.
```

---

## IdentifyAndPropagate

The final step ensures **temporal consistency** by identifying which outputs correspond to previously tracked topologies.

### Purpose

Between iterations:
1. New paths might be found
2. Previous paths might become invalid
3. The MPC solver needs consistent topology IDs to track trajectories

`IdentifyPreviousHomologies()` maps current outputs to previous ones using homotopy equivalence.

### Algorithm

```
IdentifyPreviousHomologies(outputs):
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Input: Current outputs, previous_outputs (from last iteration)
Output: outputs with assigned topology_class IDs and consistency flags

1. id_assigner ← new IDAssigner(n_paths)
2. FOR each prev_output in previous_outputs:
   - id_assigner.MarkIDAsUsed(prev_output.topology_class)

3. FOR each output in outputs:
   a. output.is_new_topology ← TRUE
   
   b. FOR each prev_output in previous_outputs:
      - IF AreHomotopicEquivalent(output.path, prev_output.path):
        - output.topology_class ← prev_output.topology_class
        - output.previously_selected ← prev_output.previously_selected
        - output.color ← prev_output.color
        - output.is_new_topology ← FALSE
        - BREAK
   
   c. IF output.is_new_topology:
      - output.topology_class ← id_assigner.GetID()  // Assign new ID

4. previous_outputs ← outputs  // Store for next iteration
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### Implementation

```cpp
void GlobalGuidance::IdentifyPreviousHomologies(
    std::vector<GlobalGuidance::OutputTrajectory> &outputs) {
    
    // Set up ID assigner - mark previous IDs as used
    IDAssigner id_assigner_(config_->n_paths_);
    for (auto &previous_output : previous_outputs_)
        id_assigner_.MarkIDAsUsed(previous_output.topology_class);
    
    int previous_outputs_identified = 0;
    
    for (auto &output : outputs) {
        output.is_new_topology_ = true;
        
        if (previous_outputs_identified < previous_outputs_.size()) {
            for (auto &previous_output : previous_outputs_) {
                
                // Check homotopy equivalence
                if (prm_.AreHomotopicEquivalent(output.path, previous_output.path)) {
                    
                    // Inherit properties from previous output
                    output.previously_selected_ = previous_output.previously_selected_;
                    output.is_new_topology_ = false;
                    output.topology_class = previous_output.topology_class;
                    output.color_ = previous_output.color_;
                    
                    previous_outputs_identified++;
                    break;
                }
            }
        }
        
        // Assign new ID for new topologies
        if (output.is_new_topology_) {
            output.topology_class = id_assigner_.GetID();
        }
    }
}
```

### OutputTrajectory Structure

```cpp
struct OutputTrajectory {
    StandaloneGeometricPath path;    // The geometric path through space-time
    CubicSpline3D spline;            // Smooth spline fit to the path
    int topology_class;               // Unique ID for this topology (persistent across iterations)
    bool is_new_topology_;            // True if first time seeing this topology
    bool previously_selected_;        // True if MPC was tracking this topology
    RosTools::Color color_;           // Visualization color (consistent per topology)
};
```

### Example: Topology Tracking Across Iterations

```
Iteration t:
  Output 0: topology_class=0, path=LEFT,  selected=TRUE
  Output 1: topology_class=1, path=RIGHT, selected=FALSE

Iteration t+1 (obstacle moved, new path found):
  Raw outputs: [PathA, PathB, PathC]
  
  Identification:
    PathA ≡ LEFT  → topology_class=0, previously_selected=TRUE
    PathB ≡ RIGHT → topology_class=1, previously_selected=FALSE
    PathC is NEW  → topology_class=2, is_new_topology=TRUE
  
  Final outputs:
    Output 0: topology_class=0, path=PathA (LEFT),  is_new=FALSE, selected=TRUE
    Output 1: topology_class=1, path=PathB (RIGHT), is_new=FALSE
    Output 2: topology_class=2, path=PathC (NEW),   is_new=TRUE
```

---

## Topology Comparison Methods

The Guidance Planner supports multiple methods for determining if two paths are **topologically equivalent**.

### Available Methods

| Method | Description | Use Case |
|--------|-------------|----------|
| **Homology** | H-signature based comparison using line integrals around obstacles | Dynamic obstacles, most accurate |
| **UVD** | Visibility-based comparison (can points along paths see each other) | Static obstacles, fast |
| **Winding Angle** | Compares accumulated angles around obstacles | Simple scenarios |

### Homology (H-Signature)

The most sophisticated method, based on complex analysis. For each obstacle, compute an **H-value** integral along the path:

$$
H(\gamma) = \oint_\gamma \frac{1}{z - z_o(t)} dz
$$

where $z_o(t)$ is the obstacle position in complex plane at time $t$.

Two paths $\gamma_a$ and $\gamma_b$ are homologous iff:
$$
|H(\gamma_a) - H(\gamma_b)| < \epsilon \quad \forall \text{ obstacles}
$$

Implementation in [homology.cpp](../../src/guidance_planner/src/homotopy_comparison/homology.cpp):

```cpp
bool Homology::AreEquivalent(const GeometricPath &a, const GeometricPath &b,
                             Environment &environment, bool compute_all) {
    
    for (size_t obstacle_id = 0; obstacle_id < obstacles.size(); obstacle_id++) {
        double h = 0;
        
        h += PathHValue(a, cached_a, obstacle_id);  // Integral over path A
        h += ConnectingSegmentHValue(a.end, b.end); // Connect endpoints
        h -= PathHValue(b, cached_b, obstacle_id);  // Integral over path B (reversed)
        
        // If closed loop has non-zero winding number → different topology
        if (std::abs(h) >= 0.1) {
            return false;
        }
    }
    
    return true;  // All obstacles have zero winding → same topology
}
```

### UVD (Visibility Decomposition)

A simpler method that samples points along both paths and checks if corresponding points can "see" each other:

```cpp
bool UVD::AreEquivalent(const GeometricPath &a, const GeometricPath &b,
                        Environment &environment, bool compute_all) {
    
    Eigen::VectorXd path_indices = Eigen::VectorXd::LinSpaced(20, 0., 1.);
    
    for (int i = 0; i < path_indices.size(); i++) {
        // Check visibility between corresponding points on paths
        if (!environment.IsVisible(a(path_indices(i)), b(path_indices(i))))
            return false;  // Obstacle blocks visibility → different topology
    }
    
    return true;  // All corresponding points visible → same topology
}
```

### Configuration

```yaml
global_guidance:
  homotopy_comparison: "UVD"  # Options: "UVD", "Homology", "WindingAngle"
```

---

## Data Structures

### Graph

```cpp
class Graph {
    Node *start_node_;
    std::vector<Node *> goal_nodes_;
    std::list<Node> nodes_;  // List for stable pointers
    
    Node *AddNode(const Node &node);
    std::vector<Node *> GetSharedNeighbours(const std::vector<Node *> &nodes);
};
```

### Node

```cpp
struct Node {
    int id_;                      // Unique identifier
    SpaceTimePoint point_;        // (x, y, k) position in space-time
    NodeType type_;               // GUARD, CONNECTOR, or GOAL
    bool replaced_;               // True if superseded by better connector
    int belongs_to_path_;         // Path ID for visualization
    std::vector<Node *> neighbours_;
};
```

### GeometricPath

```cpp
struct GeometricPath {
    std::vector<std::shared_ptr<Connection>> connections_;
    
    SpaceTimePoint operator()(double s) const;  // Evaluate at s ∈ [0,1]
    double Length3D() const;
    double RelativeSmoothness() const;
    Node *GetStart() const;
    Node *GetEnd() const;
};
```

### SpaceTimePoint

```cpp
class SpaceTimePoint {
    Eigen::Vector2d pos_;  // (x, y) spatial position
    double time_;          // Discrete time index k
    
    Eigen::Vector3d MapToTime() const {
        return Eigen::Vector3d(pos_(0), pos_(1), time_ * dt_);
    }
};
```

---

## Configuration Parameters

### Key Parameters in `settings.yaml`

```yaml
global_guidance:
  # Sampling
  n_samples: 100           # Number of samples per iteration
  sample_margin: 2.0       # Extra sampling margin around start-goal region [m]
  seed: 1                  # Random seed for reproducibility
  
  # Output
  n_paths: 3               # Maximum topology-distinct paths to output
  
  # Topology comparison
  homotopy_comparison: "UVD"  # "UVD", "Homology", or "WindingAngle"
  
  # Selection weights
  selection_weight_length: 1.0
  selection_weight_velocity: 0.0
  selection_weight_acceleration: 0.0
  selection_weight_consistency: 1.5  # Prefer previously selected path
  
  # Visualization
  visualize_all_samples: false
  visualize_homology: false
```

### Parameter Effects

| Parameter | Effect |
|-----------|--------|
| `n_samples` | More samples → denser graph → finds more topologies, but slower |
| `n_paths` | Maximum parallel topologies for MPC to optimize |
| `selection_weight_consistency` | Higher → more likely to keep tracking same topology |
| `homotopy_comparison` | Trade-off between accuracy (Homology) and speed (UVD) |

---

## Summary: Complete Pipeline

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        GUIDANCE PLANNER PIPELINE                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  INPUT: Start pose, Goals, Obstacles (with predictions), Previous Graph    │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │ 1. VISIBILITY-PRM                                                    │   │
│  │    ├── Load previous nodes (shifted in time)                        │   │
│  │    ├── Sample new points uniformly in space-time                    │   │
│  │    ├── Classify: GUARD (isolated) or CONNECTOR (bridges guards)     │   │
│  │    └── Output: Graph G with nodes and visibility edges              │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                      │                                      │
│                                      ▼                                      │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │ 2. DEPTH-FIRST SEARCH                                               │   │
│  │    ├── For each goal: DFS from start to goal                        │   │
│  │    ├── Collect all paths (forward in time only)                     │   │
│  │    └── Output: {τ₀, τ₁, ..., τₙ} paths                              │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                      │                                      │
│                                      ▼                                      │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │ 3. FILTER AND SELECT                                                │   │
│  │    ├── Sort paths by cost heuristic                                 │   │
│  │    ├── For each path: check homotopy against kept paths             │   │
│  │    ├── Keep only topology-distinct paths (up to n_paths)            │   │
│  │    └── Output: Tₚ = {τ₀, ..., τₚ} distinct topologies               │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                      │                                      │
│                                      ▼                                      │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │ 4. IDENTIFY AND PROPAGATE                                           │   │
│  │    ├── Match current paths to previous topology classes             │   │
│  │    ├── Assign topology IDs (persistent across iterations)           │   │
│  │    ├── Mark previously_selected for consistency bonus               │   │
│  │    ├── Propagate graph nodes for next iteration (time shift)        │   │
│  │    └── Output: Annotated trajectories with topology metadata        │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                      │                                      │
│                                      ▼                                      │
│  OUTPUT: Topology-distinct guidance trajectories with:                     │
│          - Smooth cubic spline representation                              │
│          - Topology class ID (for MPC tracking)                            │
│          - Consistency flag (previously selected)                          │
│          - Visualization color                                             │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## References

1. **T-MPC++ Paper**: "Topology-Driven Parallel Trajectory Optimization in Dynamic Environments" - TRO 2024
2. **Visibility-PRM**: Siméon et al., "Visibility-based Probabilistic Roadmaps"
3. **Homology for Path Planning**: Bhattacharya et al., "Search-based Path Planning with Homotopy Class Constraints" - IJRR 2012
4. **H-Signature**: [Springer Article](https://link.springer.com/article/10.1007/s10514-012-9304-1)
