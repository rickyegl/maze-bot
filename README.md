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
| `wall_map` | matches each scan to the 30 cm grid, votes every edge wall or open, publishes `/maze/walls` and `maze -> base_footprint`. Heading comes from the gyro, and scans are dropped while the IMU says the car is tilted |
| `explorer` | picks the next cell with a strategy, saves the red goal tile for last, then retraces the path. Backs out of dead ends instead of turning round in them |
| `color_sensor` | nearest calibrated colour under the car on `/color`, raw reading on `/color/rgb` |
| `aruco_detector` | 4x4_50 marker id on `/aruco/id` |
| `display` | 16x2 I2C LCD with the colour and marker id |
| `joy_drive` | gamepad driving from `/joy` |

## Track

`worlds/maze.sdf` is a 5 x 5 grid of 30 cm cells with a ramp, escaleras, speed bumps, four colour
tiles and one ArUco marker on a wall. The robot spawns on the start tile.

## On the real car

```bash
ros2 launch maze_bot robot.launch.py button_gpio:=17
```

This starts the C1 through `sllidar_ros2` (clone it into the workspace, it publishes on `/scan_raw`),
the camera through `v4l2_camera` at 640 x 480 and 5 fps, the LCD at `0x27` (`lcd_address:=0x3F` if
`i2cdetect -y 1` finds it there) and the explorer waiting for the button. No wheel encoders are needed:
without `/odom` the wall map dead reckons from `/cmd_vel` and the gyro between scans.

Still needed from the car itself:

- an IMU on `/imu` (`angular_velocity` at least, orientation if the driver fuses it)
- the colour sensor as a small rgb8 image on `/color_sensor/image`
- the motor driver taking `/drive/cmd_vel`

`teleop:=true` swaps the explorer for `joy_node` and `joy_drive`. Left stick drives, right trigger
is turbo, left trigger is precision, B stops.

### Colours

The real track never reads like the rulebook colours, so record each one with the sensor parked on it:

```bash
ros2 run maze_bot calibrate_color.py white
ros2 run maze_bot calibrate_color.py goal_red
ros2 run maze_bot calibrate_color.py
```

Readings go into the `robot` profile in `config/colors.json` and the sensor reloads them straight away.
The last line prints what is recorded.

## Tests

```bash
colcon test --packages-select maze_bot && colcon test-result --verbose
```
