# maze_bot

A four wheel skid steer robot that explores the Pista A maze in Gazebo Harmonic with ROS 2 Jazzy.
It maps walls from a 2D lidar, reads floor colours and an ArUco marker, finds the goal tile and walks
its path back to the start. On Pista B it picks up the golf ball, carries it through the gaps in the
white lines and follows the coloured tiles to FIN.

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
| `display` | 128x64 SSD1306 OLED, the colour and the marker id taking turns in big letters |
| `motors` | `/drive/cmd_vel` to the four N20s on the TB6612s, speed loop on the encoders, STOP on GPIO 26 |
| `imu` | MPU6050 on `/imu`, gyro bias taken at start, roll and pitch from gravity |
| `tcs34725` | the colour sensor as a 1x1 image on `/color_sensor/image` |
| `pista_b_map` | finds the start unit and tracks the pose against the Pista B walls |
| `pista_b` | the Pista B run: ball, lines, colours |
| `drive_test` | straight, square or spin on the gyro to check the wheels |
| `joy_drive` | gamepad driving from `/joy` |

## Track

`worlds/maze.sdf` is a 5 x 5 grid of 30 cm cells with a ramp, escaleras, speed bumps, four colour
tiles and one ArUco marker on a wall. The robot spawns on the start tile.

## On the real car

```bash
ros2 launch maze_bot robot.launch.py button_gpio:=4
```

This starts the C1 through `sllidar_ros2` (clone it into the workspace, it publishes on `/scan_raw`),
the camera through `v4l2_camera` at 640 x 480 and 5 fps, the motors, the IMU, the colour sensor, the
OLED at `0x3C` and the explorer waiting for the button. Keep the car still for a second while the
IMU measures its bias. The drivers need `lgpio` and `gpiozero` on the Pi and I2C turned on.

Check which wheel each motor turns with the car lifted:

```bash
ros2 run maze_bot motors.py --test
ros2 run maze_bot motors.py --test 3
```

Then drive it on the floor:

```bash
ros2 launch maze_bot drive_test.launch.py
ros2 launch maze_bot drive_test.launch.py square:=true side:=0.5
ros2 launch maze_bot drive_test.launch.py spin:=10
```

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

For Pista B launch with `track:=b`. The sensor then names the Pista B tiles (`field_green`,
`checkpoint_red`, `finish_lime`, ...) and logs the direction a cyan, yellow, orange or magenta tile
stands for: derecha, izquierda, arriba, abajo. Calibrate those tiles the same way.

## Pista B

```bash
ros2 launch maze_bot robot.launch.py track:=b button_gpio:=4
```

`pista_b_map` replaces the wall map and `pista_b` the explorer. On the button:

1. The lidar votes on each side of the ball's unit until one reads open. If it can't tell from the
   start it goes round the units around the ball looking. Then it parks in front of the open side
   with its back to the ball, lowers the arm with the claws open, backs up until the ball is in the
   claws, closes them and lifts the ball. It carries it round to the checkpoint and backs in.
2. It turns its camera to the field, pans a little either way, and finds the gap in each white line.
   Then it backs through the gaps one by one to the second checkpoint.
3. It drives unit to unit through the last room, reading each tile's colour for the next step, and
   stops on FIN.

The arm and claws are commanded on `gripper/arm_cmd` and `gripper/claw_cmd` (radians). There is
no servo driver for them on the car yet.

## Tests

```bash
colcon test --packages-select maze_bot && colcon test-result --verbose
```
