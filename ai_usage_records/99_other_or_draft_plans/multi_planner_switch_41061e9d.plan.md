---
name: Multi planner switch
overview: Keep your plant fixed (dynamics/controller). Formalize a planner backend switch via launch, then add GCOPTER/MINCO as Path C — the most practical recent planner with a real ROS2 route — behind the same command topics you already use for Path A/B.
todos: []
isProject: false
---

# Multi-planner interface + GCOPTER (Path C)

## Verdict on “newer planners you can transfer”

Your controller already defines the stable plant contract (do **not** change this for each planner):

| Role | Topic | Type |
|------|--------|------|
| Pose goal | `/drone/goal` | `PoseStamped` |
| Tracking setpoint | `/planner/local_goal` | `PoseStamped` |
| Feedforward | `/planner/trajectory_cmd` | [`TrajectoryCommand.msg`](src/drone_msgs/msg/TrajectoryCommand.msg) (p/v/a/yaw) |
| Yellow path | `/planner/trajectory` | `Path` |
| State | `/drone/odom` | `Odometry` |
| Map | PointCloud2 | Path A `/map/obstacles`, Path B `/map_generator/global_cloud` |

Anything new is: **planner stack + thin bridge → those topics**. Same pattern as [`ego_cmd_bridge.py`](src/drone_bringup/drone_bringup/ego_cmd_bridge.py).

### Candidate ranking (ROS2 Humble + your sim)

| Planner | Year / idea | ROS2 reality | Fit for you |
|---------|-------------|--------------|-------------|
| **Path A homemade** | EGO-style simplified | Done | Keep |
| **Path B ego-planner-swarm** | 2021 EGO B-spline | Official `ros2_version` | Done |
| **GCOPTER / MINCO** (recommend Path C) | 2022+ T-RO, successor trajectory form to EGO-v2 | Real paths: [yuwei-wu/GCOPTER `ros2`](https://github.com/yuwei-wu/GCOPTER/tree/ros2), [aerostack2 `gcopter_trajectory_generator_lib`](https://github.com/aerostack2/gcopter_trajectory_generator_lib) | **Best next port** — newer math, no SO3/fake_drone needed |
| EGO-Planner-v2 Swarm Playground | Science Robotics swarm | Heavy ROS1 playground, not a clean Humble drop-in | Skip as full tree; MINCO overlaps GCOPTER |
| Fast-Planner (HKUST) | ESDF classic | Mainly ROS1 | High port cost, older than MINCO |
| Nav2 / PX4 Avoidance | Ground/mission stacks | Wrong problem (not SE(3) local quadrotor) | Skip |
| RL planners (NavRL etc.) | 2024–25 | Models + training, poor for assignment “core formulas” | Skip |

**Chosen next backend:** GCOPTER/MINCO via a vendored ROS2-capable tree (prefer `yuwei-wu/GCOPTER` ros2 branch if it builds on Humble; else aerostack2 lib + custom ROS2 node). Keep dynamics/controller/self-developed plant.

```mermaid
flowchart LR
  subgraph plant [Fixed plant]
    dyn[drone_dynamics]
    ctrl[drone_controller]
    dyn --> ctrl
  end
  subgraph backends [Switchable planners]
    A[homemade drone_planner]
    B[official ego_planner]
    C[GCOPTER MINCO]
  end
  goal[/drone/goal]
  mapCloud[PointCloud2]
  odom[/drone/odom]
  goal --> A & B & C
  mapCloud --> A & B & C
  odom --> A & B & C
  A & B & C -->|bridge if needed| cmds["/planner/local_goal + trajectory_cmd"]
  cmds --> ctrl
  ctrl --> dyn
```

## Implementation plan

### 1. Unified launch switch (Path A / B / C)

Add [`src/drone_bringup/launch/planner_sim.launch.py`](src/drone_bringup/launch/planner_sim.launch.py) (name can be `avoidance_multi.launch.py`):

- Arg: `planner:=homemade|ego|gcopter` (default `ego` or `homemade` — pick `homemade` for course demos, `ego` if you want official default).
- Shared: `dynamics_node`, `controller_node`, `visualization_node`, RViz config per planner.
- Branch:
  - `homemade` → existing map_node + planner_node ([`avoidance.launch.py`](src/drone_bringup/launch/avoidance.launch.py) stack)
  - `ego` → include / reuse [`ego_avoidance.launch.py`](src/drone_bringup/launch/ego_avoidance.launch.py) body
  - `gcopter` → new stack below
- Keep current per-path launches so old commands still work.

### 2. Standardize the bridge contract

Extract a small shared pattern (Python node or one file per backend):

- **Inputs:** odom, goal, obstacle cloud (remap names only).
- **Outputs:** always `/planner/local_goal` + `/planner/trajectory_cmd` (+ Path for RViz).
- Path A already speaks this natively.
- Path B: keep [`ego_cmd_bridge`](src/drone_bringup/drone_bringup/ego_cmd_bridge.py).
- Path C: new `gcopter_cmd_bridge` (or node that publishes TrajectoryCommand directly if GCOPTER eval loop is ours).

Do not force every planner onto `quadrotor_msgs/PositionCommand`; bridge to **your** msgs.

### 3. Path C: vendor + wire GCOPTER

- Vendor under `src/gcopter_vendor/` (or `ego_vendor`-style) from the chosen ROS2 source; **exclude** any SO3/fake quadrotor plant.
- Map: reuse official `map_generator` cloud (same as Path B) or Path A dense cloud — start with **official random_forest** so A/B/C compare fairly.
- Planning loop: goal + odom + cloud → corridor/optimize → sample p/v/a at control rate → `TrajectoryCommand`.
- Launch: `gcopter_avoidance.launch.py` + RViz topic for GCOPTER path/markers.
- Smoke success: `EXEC`-like steady `trajectory_cmd`, drone tracks through forest like Path B.

### 4. Docs

Update [`README.md`](README.md) table:

```bash
ros2 launch drone_bringup planner_sim.launch.py planner:=homemade
ros2 launch drone_bringup planner_sim.launch.py planner:=ego
ros2 launch drone_bringup planner_sim.launch.py planner:=gcopter
```

Short note: why GCOPTER (MINCO successor to EGO-style B-spline era); why not full EGO-v2 playground / Fast-Planner / RL.

## Explicit non-goals

- Do not replace drone_dynamics / drone_controller with GCOPTER or EGO sim.
- Do not pull entire Swarm Playground / CUDA sim.
- Do not require Aerostack2 as a whole stack — only the trajectory library if we go that route.

## Success

- One arg selects planner; plant unchanged.
- Path A and B still work.
- Path C GCOPTER flies the same style mission (-15→15 forest) via `/planner/trajectory_cmd`.
