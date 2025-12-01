# Assignment 1: Turtlebot Navigation Pipeline  
## AprilTag Detection & Table Localization  
**Package:** `lorandi_assignament_1`  
**Date:** December 01, 2025  
**Course:** Intelligent Robotics 2025/2026

---

## 1. Introduction & System Overview

This assignment develops an autonomous navigation pipeline for TurtleBot3.  
The robot navigates from the lab entrance to a goal position between two AprilTags, then detects cylindrical tables using LiDAR.

The solution implements a modular **ROS2** architecture combining:

- AprilTag-based positioning  
- Bug2 obstacle avoidance  
- LiDAR-based table detection  
- TF2 transformations for consistent coordinate handling  

---

## 2. System Architecture

The system is composed of **four ROS2 nodes**, each responsible for a specific stage of the perception–navigation pipeline.

| Node | Function | Topics |
|------|----------|--------|
| **tag_detector.py** | AprilTag pose extraction | `/apriltag/detections → /tags_poses_camera` |
| **go_to_tags.py** | Bug2 navigation control | `/scan, /odom → /cmd_vel` |
| **table_detector.py** | LiDAR clustering for table detection | `/scan → /detected_tables` |
| **table_publisher.py** | Detection filtering and publishing | `/detected_tables → /final_tables` |

Each node communicates using standard ROS2 messages, ensuring modularity and maintainability.

---

## 3. Implementation & Navigation Strategy

### Bug2 Algorithm
The navigation core implements **Bug2**, a classical reactive algorithm guaranteeing convergence in unknown environments.

Key behaviors:

- **GO_TO_GOAL mode:** the robot moves directly toward the target when the path is clear.  
- **WALL_FOLLOWING mode:** left-wall tracking when obstacles block the path.  
- Exit criteria:
  - robot crosses the **m-line** (start→goal line),
  - robot is **closer to the goal** than at the obstacle encounter,
  - front direction is **clear**,  
  - heading error is within acceptable bounds.

### Key Parameters
- Obstacle threshold: **0.75 m**  
- Wall-follow distance: **0.45 m**  
- Control frequency: **5 Hz**  
- Distance tolerance: **0.25 m**

### Table Detection
LiDAR data is converted to Cartesian space and clustered. Validation uses:

- Aspect ratio < **3.0**  
- Size between **0.15–0.80 m**  
- Minimum cluster size: **≥5 points**

Detections are transformed to the `odom` frame and temporally filtered to reduce noise.

### TF2 Integration
All detected positions (AprilTags and tables) are consistently transformed between frames using ROS2 TF2.

---

## 4. Results, Limitations & Conclusion

### Performance
- The robot reliably reaches goals in cluttered environments **without global planning**.  
- Table detection successfully identifies the three cylindrical objects with minimal false positives.  
- Architecture is modular, allowing easy debugging and extension.

### Strengths
- Robust obstacle avoidance using Bug2  
- Real-time execution  
- Correct TF frame handling  
- Noise-resistant table detection pipeline  

### Limitations & Future Work
- Fixed **left-wall strategy** (not adaptive)  
- L-shaped corridors may cause extended wall-following  
- Potential improvements:
  - adaptive wall-side selection,  
  - potential field layer,  
  - RRT* fallback planner,  
  - learning-based parameter tuning  

---

## 5. Acknowledgments

### AI-Assisted Development
This project was developed with assistance from artificial intelligence systems:

- **Claude AI (Anthropic)**  
  Contributed to architecture design, Bug2 implementation, LiDAR clustering pipeline, TF2 transforms, debugging, optimization, and code review.

- **ChatGPT (OpenAI)**  
  Assisted with modular design, ROS2 best practices, parameter tuning, algorithm clarification, and debugging support.

### Human Contribution
Team members performed:

- Conceptualization  
- Integration testing  
- Parameter calibration  
- Simulation experiments  
- ROS2 development  
- Project supervision and direction  

AI tools acted as accelerators, while humans maintained full scientific and engineering oversight.

---

## Submission Information

- **Group Members:** Lorandi Enzo 
- **Git Repository:**  https://github.com/Maxz0l/lorandi_assignement_1
- **Video Demonstration:** 

---

## References

1. V. Lumelsky & A. Stepanov, *Path-Planning Strategies for a Point Mobile Automaton*, Algorithmica, 1987.  
2. E. Olson, *AprilTags: A robust and flexible visual fiducial system*, ICRA, 2011.  
3. ROS 2 Documentation: https://docs.ros.org/
