# maze_bot

A four wheel skid steer robot that explores the Pista A maze in Gazebo Harmonic with ROS 2 Jazzy.
It maps walls from a 2D lidar, reads floor colours and an ArUco marker, finds the goal tile and walks
its path back to the start.

## Build

```bash
mkdir -p ~/ws/src && cd ~/ws/src
git clone https://github.com/rickyegl/maze-bot.git maze_bot
cd ~/ws && colcon build --symlink-install && source install/setup.bash
```

## Run

```bash
ros2 launch maze_bot sim.launch.py strategy:=flood
ros2 service call /explorer/start std_srvs/srv/Trigger
```

Strategies: `flood`, `dijkstra`, `dfs`, `right_hand`, `left_hand`, `random`.

## Nodes

| Node | Does |
|---|---|
| `base` | speed limits and a watchdog on `/cmd_vel`, masks the lidar arc hidden by the gripper |
| `wall_map` | matches each scan to the 30 cm grid, votes every edge wall or open, publishes `/maze/walls` and `maze -> base_footprint` |
| `explorer` | picks the next cell with a strategy, saves the red goal tile for last, then retraces the path |
| `color_sensor` | nearest calibrated colour under the car on `/color` |
| `aruco_detector` | 4x4_50 marker id on `/aruco/id` |
| `display` | 16x2 I2C LCD with the colour and marker id |

## Track

`worlds/maze.sdf` is a 5 x 5 grid of 30 cm cells with a ramp, escaleras, speed bumps, four colour
tiles and one ArUco marker on a wall. The robot spawns on the start tile.

## Tests

```bash
colcon test --packages-select maze_bot && colcon test-result --verbose
```
