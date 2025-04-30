# RoboDefuse: Robot Bomb Zone Navigator with Obstacle Avoidance

A Unity-based robot car controller that autonomously navigates through multiple bomb zone waypoints while avoiding obstacles in its path. The robot detects bombs, marks visited zones, and dynamically reroutes itself using local obstacle avoidance strategies.

---

## Features

- **Autonomous Navigation**, through a sequence of bomb zone waypoints.
- **Dynamic Obstacle Avoidance**, using raycasting and reactive movement logic.
- **Stuck Detection & Recovery**, with automatic reverse and turning strategies.
- **Bomb Detection**, via trigger checks around bomb zones using physics overlap.
- **Dead-End Memory**, remembers previously stuck positions to avoid re-visiting.
- **No Manual Intervention Needed**, starts automatically and never stops until all waypoints are visited.

---

## How It Works

1. The robot initializes a list of bomb zone targets (waypoints).
2. It moves toward each target in order using basic forward motion and smooth turning.
3. While moving, it continuously checks for obstacles using forward raycasts.
4. If stuck (not moving for a few seconds), it initiates reverse-and-turn maneuvers.
5. When a bomb is detected near the target zone, it logs the detection and continues to the next waypoint.
6. After the last waypoint is visited, the robot stops (or can be made to loop).

---

## Important Configuration Parameters

| Variable                    | Description                                                   |
|----------------------------|---------------------------------------------------------------|
| `bombZones`                | Array of target locations to visit                            |
| `obstacleLayer`            | Layer mask used for detecting walls or obstacles              |
| `detectionDistance`        | Range of raycast for obstacle detection                       |
| `targetReachThreshold`     | Distance to consider a waypoint as "reached"                  |
| `reverseTime`, `rotateInPlaceTime` | Time spent during recovery maneuvers          |
| `raycastAngleStep`         | Angle between ray directions in obstacle detection sweep      |
| `stuckCheckInterval`       | How often the script checks if the robot is stuck             |
