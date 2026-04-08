# Robomaze

A 2D maze game for learning robotics programming. Spawn robots, navigate them
through a maze, and race to the goal!

Robomaze offers two modes:

- **REST API mode**: a simple HTTP interface for beginners. Control robots with
  basic HTTP requests (move North/South/East/West), view the maze in a web
  browser. Great for an engaging first introduction to programming robots.
- **ROS2 mode**: a full robotics simulation with continuous velocity control,
  laser scans, odometry, and TF frames. For a more advanced, gamified
  introduction to ROS2 and robotics programming.

Both modes share the same maze engine and tileset.

![Screenshot](doc/screenshot.jpg)

## Prerequisites

- Python 3
- `pip install flask opencv-python numpy` (or system packages: `sudo apt install python3-opencv python3-flask`)
- For ROS2 mode: ROS2 (tested with Jazzy)

---

## REST API mode

The REST mode is the simplest way to get started. No ROS2 required -- just
Python, Flask, and a web browser.

### Start the server

```bash
python -m robomaze.rest_server
```

Options:

```bash
# Random maze with goal enabled
python -m robomaze.rest_server --random --goal

# Custom size and seed
python -m robomaze.rest_server --random --width 31 --height 25 --seed 42 --goal

# Change host/port
python -m robomaze.rest_server --host 0.0.0.0 --port 8080
```

### View the maze

Open [http://localhost:5000/live](http://localhost:5000/live) in a browser.
The maze renders with pan (drag) and zoom (scroll).

### Control robots

Robots are created automatically on their first move:

```bash
# Move a robot (N/S/E/W)
curl http://localhost:5000/api/move/WallE/E
# Returns: [true, [false, true, false, true]]
#          (success, [obstacle_N, obstacle_S, obstacle_E, obstacle_W])

# Check remaining life
curl http://localhost:5000/api/life/WallE

# List all robots
curl http://localhost:5000/api/robots
```

### REST API reference

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/move/<name>/<dir>` | GET | Move robot one tile (N/S/E/W). Auto-creates on first call. Returns `[success, [N,S,E,W obstacles]]` |
| `/api/life/<name>` | GET | Remaining life points (0-10) |
| `/api/robots` | GET | All robots with position, life, and age |
| `/api/map` | GET | Maze data (dimensions, tile array, goal) |
| `/api/render_map` | GET | Pre-computed tile types for rendering |
| `/live` | GET | Web visualization page |

### Game rules (REST mode)

- Robots start with **10 life points**
- Each collision (moving into a wall) costs **1 life**
- Minimum **200ms** between moves (rate-limited)
- Robots are removed after **10 minutes** of inactivity
- When life reaches 0, the robot is removed

### Writing a REST client

Any language with HTTP support works. A minimal Python example:

```python
import requests, time

name = "MyBot"
base = "http://localhost:5000"

for direction in ["E", "E", "N", "N", "E"]:
    r = requests.get(f"{base}/api/move/{name}/{direction}").json()
    success, obstacles = r
    print(f"Move {direction}: {'OK' if success else 'BLOCKED'}, obstacles: {obstacles}")
    time.sleep(0.3)
```

---

## ROS2 mode

The ROS2 mode provides a full robotics simulation with continuous physics,
laser sensors, odometry, and TF -- suitable for learning navigation, SLAM, and
autonomous robot programming.

### Build

```bash
# From your colcon workspace src/ directory:
cd ~/ros2_ws/src
ln -s /path/to/robomaze .

# Build
cd ~/ros2_ws
colcon build --packages-select robomaze
source install/setup.bash
```

### Start the simulator

```bash
ros2 run robomaze simulator
```

An OpenCV window opens showing the maze. Press **f** to toggle fullscreen.

Options:

```bash
# Random maze with goal enabled
ros2 run robomaze simulator --ros-args -p random_maze:=true -p enable_goal:=true

# Reproducible random maze
ros2 run robomaze simulator --ros-args -p random_maze:=true -p seed:=1234

# Custom size
ros2 run robomaze simulator --ros-args -p random_maze:=true -p map_width:=31 -p map_height:=25
```

Parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `random_maze` | `false` | Generate a random maze instead of the built-in one |
| `seed` | `-1` | Random seed (-1 = auto) |
| `enable_goal` | `false` | Enable goal marker and race mode |
| `map_width` | `20` | Maze width in tiles (only with `random_maze`) |
| `map_height` | `20` | Maze height in tiles (only with `random_maze`) |

### Spawn a robot

```bash
ros2 topic pub --once /create_robot std_msgs/msg/String "data: 'WallE'"
```

Robots spawn at a random open position away from the goal and other robots.

### Delete a robot

```bash
ros2 topic pub --once /delete_robot std_msgs/msg/String "data: 'WallE'"
```

### Control with keyboard

```bash
ros2 run robomaze keyboard_teleop --ros-args -p robot_name:=WallE
```

- Arrow up/down: increase/decrease linear speed
- Arrow left/right: increase/decrease angular speed
- `q`: quit

### Topic interface

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/create_robot` | `std_msgs/String` | Subscribe | Send robot name to spawn |
| `/delete_robot` | `std_msgs/String` | Subscribe | Send robot name to remove |
| `/<name>/cmd_vel` | `geometry_msgs/Twist` | Subscribe | `linear.x` and `angular.z` |
| `/<name>/scan` | `sensor_msgs/LaserScan` | Publish | Lidar: -45 to +45 deg, 2 deg steps |
| `/<name>/odom` | `nav_msgs/Odometry` | Publish | Robot pose and velocity |
| `/goal_reached` | `std_msgs/String` | Publish | Robot that reached the goal |
| `/winners` | `std_msgs/String` | Publish | Ranking: `#1 WallE (12.3s), #2 Bot2 (14.8s)` |

### TF frames

The simulator broadcasts: `<name>_odom` -> `<name>_base_link`

### Visualize in RViz2

```bash
rviz2
```

- Add a `LaserScan` display, topic: `/WallE/scan`
- Add a `TF` display to see the robot frame
- Set fixed frame to `WallE_odom`
