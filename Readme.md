# Assignment 1 — The Turtlebot

**Course:** Intelligent Robotics 2025/2026  
**Package:** `lorandi_assignament_1`  
**Student:** Lorandi Enzo  
**Repository:** https://github.com/Maxz0l/lorandi_assignement_1  
**Video:** *(link to be added)*

---

## Project Structure

```
lorandi_assignement_1/
├── scripts/                          # ROS2 nodes (Python executables)
│   ├── tag_detector.py               # Logs AprilTag detections
│   ├── go_to_tags.py                 # Hybrid navigation to midpoint
│   ├── table_detector.py             # LiDAR cylinder detection
│   └── table_publisher.py            # Detection aggregation (clustering)
├── launch/                           # Launch files
│   └── lorandi_assignment_1.launch.py
├── config/                           # Node parameters (YAML)
│   └── apriltag_params.yaml
├── package.xml                       # Package manifest (dependencies)
├── CMakeLists.txt                    # Build system instructions
└── Readme.md
```

### ROS2 Package — `package.xml` and `CMakeLists.txt`

A **ROS2 package** is the basic unit of software organization. Any directory containing a `package.xml` is a package. The manifest declares:
- The package name, version, and maintainer
- All dependencies — packages that must be available to build and run this code

`rosdep` reads `package.xml` to install missing dependencies automatically:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

`CMakeLists.txt` tells `colcon` (the ROS2 build tool) how to build the package and where to install executables so that `ros2 run` and `ros2 launch` can find them.

### `scripts/` — ROS2 Nodes

A **node** is a single process in the ROS2 computational graph. Nodes are the basic execution units: each node runs independently, has a unique name, and communicates exclusively through the middleware (topics, services, actions).

Key properties:
- **Decoupled**: nodes do not call each other directly — they publish and subscribe to named channels called **topics**
- **Typed**: every topic carries messages of a specific type (`LaserScan`, `Twist`, `PoseArray`…)
- **Asynchronous**: a subscriber's callback is called automatically when a message arrives, driven by the executor (`rclpy.spin`)

This package has four nodes, each in its own Python file. Each file contains exactly one class inheriting from `Node`, and a `main()` function that calls `rclpy.spin()`.

### `launch/` — Launch Files

Without a launch file, starting this project would require running five separate `ros2 run` commands in five terminals, in the right order, with the right parameters. A **launch file** replaces all of that with a single command.

In ROS2, launch files are plain Python scripts that return a `LaunchDescription` — a list of **actions** to execute:

| Action | What it does |
|---|---|
| `Node(...)` | Starts a ROS2 node with specified parameters, remappings, and log level |
| `IncludeLaunchDescription(...)` | Includes and executes another launch file (here: the Gazebo simulation) |
| `SetEnvironmentVariable(...)` / `os.environ` | Sets an environment variable before any process starts |

The launch file `lorandi_assignment_1.launch.py` starts six processes in one command:

1. **Gazebo simulation** — `ir_launch/assignment_1.launch.py` (TurtleBot3 + environment)
2. **`apriltag_node`** — detects AprilTags from the camera feed; log level set to `error` to suppress synchronization warnings
3. **`tag_detector`** — subscribes to detections and logs each new tag
4. **`go_to_tags`** — navigates the robot to the midpoint between the two tags
5. **`table_detector`** — detects cylindrical tables with LiDAR once the robot arrives
6. **`table_publisher`** — aggregates raw detections into stable final positions

**Topic remapping** is another key feature of the launch system. Instead of hardcoding topic names in the node code, you can remap them at launch time:
```python
remappings=[('image_rect', '/rgb_camera/image')]
```
This tells `apriltag_node` to subscribe to `/rgb_camera/image` even though it internally uses `image_rect`. The node code never changes.

### `config/` — YAML Parameters

ROS2 nodes can expose parameters (tag family, detection thresholds…) that are set at startup without recompiling. `apriltag_params.yaml` configures the AprilTag detector: tag family (`tag36h11`), tag size (0.050 m), and detection thresholds. The `Node` action in the launch file loads this file via `parameters=[path_to_yaml]`.

---

## Prerequisites

**ROS2 Humble** must be installed and sourced first (Ubuntu 22.04):
follow https://docs.ros.org/en/humble/Installation.html, then `source /opt/ros/humble/setup.bash`.

The `ir_launch` simulation package is provided by the course and must be present in the same colcon workspace.

All other dependencies are installed by the provided script:
```bash
chmod +x install_deps.sh
./install_deps.sh
```

`install_deps.sh` installs the required apt packages (`ros-humble-ros-gz`, `ros-humble-apriltag-ros`, `ros-humble-tf2-geometry-msgs`) and then calls `rosdep` to resolve everything declared in `package.xml` automatically.

---

## How to Launch

```bash
# 1. Build the workspace
cd ~/ws_assignments
colcon build --packages-select lorandi_assignament_1
source install/setup.bash

# 2. Launch the full pipeline (simulation + all nodes)
ros2 launch lorandi_assignament_1 lorandi_assignment_1.launch.py
```

`colcon build` compiles the package and installs its executables under `install/`. `source install/setup.bash` adds them to the ROS2 discovery path so that `ros2 launch` can find them by package name.

---

## Pipeline Overview

The assignment requires four capabilities, each handled by a dedicated ROS2 node:

| Consigne | Node | Input topic | Output topic |
|---|---|---|---|
| Detect AprilTags | `tag_detector.py` | `/apriltag/detections` | `/tags_poses_camera` |
| Navigate to position between tags | `go_to_tags.py` | `/apriltag/detections`, `/odom`, `/scan` | `/cmd_vel`, `/goal_reached` |
| Detect cylindrical tables (LiDAR) | `table_detector.py` | `/scan`, `/goal_reached` | `/detected_tables` |
| Return table positions in odom | `table_publisher.py` | `/detected_tables` | `/final_tables` |

The `/goal_reached` Bool topic acts as a sequencer signal: table detection only activates once the robot has reached its destination.

---

## 1. AprilTag Detection

`apriltag_ros` detects the tags and publishes them on `/apriltag/detections`. It also automatically publishes a **TF frame** named `tag36h11:<id>` for each detected tag (family: `tag36h11`, size: 0.050 × 0.050 m).

`tag_detector.py` subscribes to this topic with a **BEST_EFFORT QoS profile**. QoS (Quality of Service) defines how reliably messages are delivered. The publisher (`apriltag_ros`) uses `BEST_EFFORT` — it sends and forgets, with no delivery guarantee. A subscriber must declare a **compatible** QoS; using the default `RELIABLE` profile would cause ROS2 to silently refuse the connection.

`go_to_tags.py` watches the same topic and accumulates tag IDs. Once **two distinct tags** are seen, it queries TF2 to get both positions in the `odom` frame and computes their midpoint as the navigation goal.

---

## 2. Navigation to the Goal (Hybrid Architecture)

### Design Choice

Rather than a purely reactive algorithm (Bug2) or a purely deliberative planner (A*), this implementation uses the **Hybrid Deliberative/Reactive architecture** (slides 18-BIS, 19 — Arkin 1989, Brooks 1986). The two layers run concurrently:

- **Deliberative layer** (global): computes the desired heading from the AprilTag positions retrieved via TF2.
- **Reactive layer** (local): reads the LiDAR scan in real time and overrides the deliberative command when an obstacle is detected.

### TF2 — The Coordinate Frame System

TF2 is the ROS2 library for managing **coordinate frames**. Every physical element has a frame: `base_link` (robot body), `odom` (fixed world origin), `camera_link`, `tag36h11:0`…

TF2 maintains a **tree of transforms** published on `/tf` and `/tf_static`. Any node can query: *"what is the position of frame A expressed in frame B at time T?"*

```
odom
 └── base_link
      └── camera_link
           └── tag36h11:0   ← published by apriltag_ros on each detection
```

`go_to_tags.py` calls `lookup_transform('odom', 'tag36h11:0', ...)` to get the tag's world position — stable regardless of robot movement, because `odom` is fixed.

### Priority Layers (high → low)

| Priority | Condition | Behaviour |
|---|---|---|
| **0 — Corridor** *(extra points)* | Both lateral LiDAR readings < 0.60 m | Lateral centering between walls |
| **1 — Emergency** | Front obstacle < 0.30 m | Back up + pivot toward open side |
| **2 — Reactive** | Front obstacle < 0.55 m | LiDAR guides direction; speed reduced |
| **3 — Deliberative** | Open space | Proportional heading control toward goal + soft repulsion |

The key property of the deliberative layer: **`linear.x` is always positive** — the robot always moves forward. Heading correction and obstacle repulsion are both applied as angular corrections that sum together (Motor Schema). This avoids the spin-in-place behaviour of Bug2.

### Goal Localization

The goal is computed once at navigation start, via TF2:
```
goal = (pos_tag_A + pos_tag_B) / 2   [in odom frame]
```
The `odom` frame is fixed, so the goal coordinates remain stable throughout navigation regardless of robot motion.

### Key Parameters

| Parameter | Value | Role |
|---|---|---|
| `k_att` | 1.0 | Heading error gain (deliberative) |
| `max_linear` | 0.4 m/s | Forward speed cap |
| `d_obstacle` | 0.55 m | Reactive layer activation threshold |
| `d_emergency` | 0.30 m | Emergency backup threshold |
| `distance_tolerance` | 0.25 m | Goal arrival criterion |
| Control frequency | 10 Hz | `create_timer(0.1, navigate)` |

---

## 3. Extra Points — Corridor Navigation

When the robot detects a wall on both its left **and** right sides within 0.60 m (measured by narrow ±20° LiDAR windows), it enters **corridor mode** (Priority 0 — highest priority).

In corridor mode:
- The robot ignores the global goal direction.
- A P-controller on the **lateral error** (left distance − right distance) centres the robot between the walls.
- Forward speed is reduced to 0.30 m/s for safe traversal.
- On exit, normal goal-directed navigation resumes.

The narrow ±20° windows prevent false activations in an open room where distant walls or tables might fall within a wider cone.

---

## 4. Table Detection

### Sensor and Method

Tables are detected using the **LiDAR** (`/scan`), activated once `/goal_reached` is received. The `LaserScan` message contains a `ranges[]` array: one distance measurement per ray, from `angle_min` to `angle_max` in steps of `angle_increment`.

Detection uses 1D range segmentation on each full scan rotation.

### Algorithm

**Step 1 — Segmentation:** `ranges[]` is split into clusters. Two consecutive rays belong to the same object if their range difference is below 0.20 m; a larger jump indicates an object boundary.

**Step 2 — Filtering:** Each cluster is validated against three criteria:
- ≥ 5 rays (rejects single-point noise)
- Aspect ratio < 3.0 (rejects walls and elongated structures)
- Apparent diameter ∈ [0.15, 0.80] m (rejects objects too small or too large)

**Step 3 — Centre estimation:** The centroid of the visible arc underestimates the cylinder's distance because the LiDAR only sees the front face. The correct estimate is:

```
center = (r_mid + radius_est) × [cos(θ_mid), sin(θ_mid)]
```

where `r_mid` is the range at the arc midpoint and `radius_est = apparent_diameter / 2`.

**Step 4 — Frame transformation:** Positions are converted from `base_link` (robot-fixed) to `odom` (world-fixed) via TF2, so they remain stable as the robot moves.

### Aggregation (`table_publisher.py`)

`table_detector.py` publishes raw detections at ~10 Hz. `table_publisher.py` maintains a rolling history of the last 50 scans using a `deque(maxlen=50)` (O(1) circular buffer) and applies **greedy single-linkage clustering** (0.50 m threshold) every second. The three most reliably detected positions are published on `/final_tables`.

---

## 5. ROS2 Architecture

```
/apriltag/detections ──┬──► tag_detector.py ──► /tags_poses_camera
                       │
                       └──► go_to_tags.py ──────► /cmd_vel
                                │                 /goal_reached
                                │ (TF2: odom ← tag36h11:X)
/odom ─────────────────────────►│
/scan ─────────────────────────►│
                                          │
/goal_reached ◄─────────────────────────────
      │
      ▼
table_detector.py ◄── /scan
      │              (TF2: base_link → odom)
      ▼ /detected_tables
table_publisher.py ──► /final_tables
```

All inter-node communication uses standard ROS2 message types (`LaserScan`, `Odometry`, `Twist`, `PoseArray`, `Bool`, `AprilTagDetectionArray`), ensuring full modularity and compatibility with any ROS2-compliant tool.

---

## 6. Acknowledgments

### AI-Assisted Development

This project was developed with assistance from **Claude AI (Anthropic)**. AI contributions include: architecture design, navigation algorithm selection and implementation, LiDAR clustering pipeline, TF2 integration, debugging, code review, and documentation.

All AI-generated content was reviewed, validated, and directed by the student. The overall approach, design decisions (hybrid architecture, corridor detection strategy, cylinder centre estimation method), parameter tuning, and integration testing were performed by the student.

### Human Contribution
- Conceptualization and problem analysis
- Selection of the hybrid deliberative/reactive architecture
- Integration and simulation testing
- Parameter calibration
- ROS2 pipeline design and supervision

---

## References

1. R. C. Arkin, *Motor Schema-Based Mobile Robot Navigation*, The International Journal of Robotics Research, 1989.
2. R. A. Brooks, *A Robust Layered Control System for a Mobile Robot*, IEEE Journal on Robotics and Automation, 1986.
3. E. Olson, *AprilTag: A robust and flexible visual fiducial system*, ICRA, 2011.
4. ROS 2 Documentation — https://docs.ros.org/
5. Course slides: *Architectures for Autonomous Robots — Reactive* (18-BIS) and *Hybrid* (19), Intelligent Robotics 2025/2026.
