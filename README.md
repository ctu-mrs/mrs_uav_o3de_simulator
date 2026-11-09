# o3de_drone_sim

A multirotor simulator in [O3DE](https://o3de.org) with a ROS 2 interface. An x500 quadrotor flies with a forward RGB camera and a Livox Mid-360-style lidar, in a warehouse or in a Gaussian splat world.

The dynamics and the attitude and rate controllers run inside O3DE at 500 Hz. ROS 2 carries attitude or attitude-rate commands in and sensor data out, using standard messages only. To fly it with the [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system), use [mrs_uav_o3de_api](https://github.com/ctu-mrs/mrs_uav_o3de_api), as described under [Flying with the MRS UAV System](#flying-with-the-mrs-uav-system).

## Installation on Ubuntu 24.04

### 1. ROS 2 Jazzy

Install ROS 2 Jazzy as described in the [ROS 2 documentation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), then the message packages that the `ROS2` Gem needs:

```sh
sudo apt install ros-jazzy-desktop ros-jazzy-control-msgs ros-jazzy-ackermann-msgs ros-jazzy-vision-msgs ros-jazzy-rmw-zenoh-cpp
```

### 2. O3DE 26.05

Download the Linux `.deb` package of O3DE 26.05 from [o3de.org/download](https://o3de.org/download/) and install it. It goes to `/opt/O3DE/26.05` and pulls in Clang, Ninja, and the other build dependencies:

```sh
sudo apt install ./o3de_26.05_amd64.deb   # the file you downloaded
sudo apt install cmake git
/opt/O3DE/26.05/scripts/o3de.sh register --this-engine
```

The last command registers the engine in `~/.o3de/o3de_manifest.json`, and the first run of `o3de.sh` also downloads O3DE's own Python, which takes a few minutes.

### 3. The external Gems

The project needs `ROS2` 4.3.0, `ROS2Sensors` 1.1.0, `LevelGeoreferencing` 1.0.0, and `WarehouseAssets` 2.0.4 from [o3de-extras](https://github.com/o3de/o3de-extras). Download and register them:

```sh
O3DE=/opt/O3DE/26.05/scripts/o3de.sh
URL=https://github.com/o3de/o3de-extras/releases/download/2.0
for gem in ROS2:ros2:4.3.0 ROS2Sensors:ros2sensors:1.1.0 LevelGeoreferencing:levelgeoreferencing:1.0.0 WarehouseAssets:warehouseassets:2.0.4; do
  IFS=: read -r name file version <<< "$gem"
  dir=~/O3DE/Gems/$name/$version
  mkdir -p "$dir"
  curl -L "$URL/$file-$version-gem.zip" -o /tmp/$file.zip && unzip -oq /tmp/$file.zip -d "$dir"
  $O3DE register --gem-path "$dir"
done
```

The Project Manager's **Gem Catalog** can download the same Gems instead.

### 4. This project

Clone it, register it, and build it. ROS 2 must be sourced for every configure and build:

```sh
git clone https://github.com/ctu-mrs/mrs_uav_o3de_simulator.git ~/git/o3de_drone_sim
cd ~/git/o3de_drone_sim
/opt/O3DE/26.05/scripts/o3de.sh register --project-path .

source /opt/ros/jazzy/setup.bash
CC=clang CXX=clang++ cmake -B build/linux -S . -G "Ninja Multi-Config"
cmake --build build/linux --config profile --target Editor DroneSim.GameLauncher -j 8
```

The first build takes a long time. Keep `-j` at about one job per 2 GB of RAM, or the compiler runs out of memory. Keep the project in `~/git/o3de_drone_sim`, where the MRS UAV System session looks for it, or set `O3DE_PROJECT` to its path.

## First test in the Editor

1. Start the Editor with `Tools/run_editor.sh`, never directly, as explained under [Run](#run).
2. On the first start, the Asset Processor compiles all assets, which takes a long time. Its icon in the system tray shows the progress. Wait until it is done.
3. Open a level with **File > Open Level**: `FlatWorld` first, because it is small and loads fast, then `DemoLevel`, the warehouse.
4. Press **Ctrl+G** to play. The drone sits on the ground, and the game view follows it. Press **Esc** to stop.
5. While it plays, check the ROS 2 interface from another terminal:
   ```sh
   source /opt/ros/jazzy/setup.bash
   ros2 topic list                 # /clock and the /uav1 topics
   ros2 topic hz /uav1/odom        # about 250 Hz in sim time
   ```
   To see it fly without a controller, command a zero body rate with a throttle above hover. It climbs, then drifts, because nothing holds its position. Stop it with Ctrl+C; the throttle is cut 0.5 s later:
   ```sh
   ros2 topic pub -r 50 /uav1/cmd/attitude_rate geometry_msgs/msg/TwistStamped "{twist: {linear: {z: 0.6}}}"
   ```

Without the Editor, `Tools/run_game.sh flatworld` and `Tools/run_game.sh` (the warehouse) run the same levels in the game launcher, after the Asset Processor has compiled them once.

## Flying with the MRS UAV System

The [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system) flies the drone through the HW API plugin [mrs_uav_o3de_api](https://github.com/ctu-mrs/mrs_uav_o3de_api), which also has an example tmux session that starts the simulator, the MRS UAV System, and RViz.

1. Install the MRS UAV System for ROS 2 Jazzy, as described in its README, and `tmuxinator`.
2. Build the plugin in a workspace:
   ```sh
   mkdir -p ~/ws_o3de/src && cd ~/ws_o3de/src
   git clone https://github.com/ctu-mrs/mrs_uav_o3de_api.git
   cd ~/ws_o3de
   source /opt/ros/jazzy/setup.bash
   colcon build
   ```
3. Start the session:
   ```sh
   source ~/ws_o3de/install/setup.bash
   cd ~/ws_o3de/src/mrs_uav_o3de_api/tmux/one_drone
   ./start.sh
   ```
   The drone takes off by itself once the MRS UAV System is ready. It loads `DemoLevel` unless `O3DE_LEVEL` names another level, in lowercase, for example `export O3DE_LEVEL=flatworld` before `./start.sh`. `./start_headless.sh` runs without rendering, so without the camera, and faster than real time. The plugin's README describes the session's flags and its limitations.

The game launcher in the session does not connect to the Asset Processor, so compile the assets first, with the Editor or `AssetProcessorBatch`, and again after every change to a level or the drone.

## Run

**Start the Editor and Project Manager only with this script.** It finds the engine from the O3DE manifest, or from `O3DE_ENGINE_PATH`, sources ROS 2, and uses `rmw_fastrtps_cpp` unless `RMW_IMPLEMENTATION` is set:

```sh
Tools/run_editor.sh              # Editor
Tools/run_editor.sh --manager    # Project Manager
```

Without ROS 2 sourced, the ROS and Multirotor Gems fail to load, without an error that stops anything. The Asset Processor then compiles prefabs without their components, and saving in the Editor removes the components from the prefabs for good. If the drone has no topics, delete the `.spawnable` files under `Cache/linux` and run `AssetProcessorBatch` with ROS 2 sourced.

To run the game, process the assets once by opening the Editor or running `AssetProcessorBatch`, then:

```sh
Tools/run_game.sh                # the default level
Tools/run_game.sh flatworld      # another level
Tools/run_game.sh --list         # the compiled levels
Tools/run_game.sh --headless     # without rendering, so without the camera
```

Other arguments go to the launcher. Set `RMW_IMPLEMENTATION` to match your other ROS nodes; the default is `rmw_fastrtps_cpp`. With `rmw_zenoh_cpp`, also start a router with `ros2 run rmw_zenoh_cpp rmw_zenohd`.

The script passes these flags:

- `--t_simulationTickDeltaOverride=17000` advances the simulation 17 ms per rendered frame, in microseconds. Slow frames then slow the simulation down, instead of looking like data dropouts to ROS. Without it, flying at low frame rates is unstable. All ROS nodes need `use_sim_time:=true`.
- `--r_renderScale=0.5` lowers the game view's resolution, not the camera's.
- `--rhi=null`, with `--headless`, runs without rendering.

In the game, press **C** to switch between following the drone and a free camera: WASD to move, Q and E for down and up, the right mouse button to look around, and left shift to go faster. In the free camera, **T** keeps the view aimed at the drone while you move. **H** hides the guide in the game view. **V** stops rendering the game view while the drone's camera keeps running, and `--drone_viewport=false` starts that way. It raised the frame rate by 5 to 20%, less than `--r_renderScale=0.5` does, and adds nothing on top of it.

## ROS 2 interface

Topics are under the drone's namespace, `/uav1`.

| Topic | Type | Notes |
|---|---|---|
| `cmd/attitude_rate` | `geometry_msgs/TwistStamped` | `twist.angular`: body rate [rad/s], `twist.linear.z`: throttle [0, 1] |
| `cmd/attitude` | `geometry_msgs/PoseStamped` | `pose.orientation`: attitude, `pose.position.z`: throttle [0, 1] |
| `odom` | `nav_msgs/Odometry` | 250 Hz, in the world frame, twist in the body frame |
| `imu` | `sensor_msgs/Imu` | 250 Hz |
| `camera_image_color`, `color_camera_info` | `sensor_msgs/Image`, `CameraInfo` | 1280x720, 30 Hz |
| `rangefinder` | `sensor_msgs/Range` | Ray cast down, 100 Hz, 40 m, 41 m when nothing is in range |
| `on_ground` | `std_msgs/Bool` | From the contacts of the rigid body, 100 Hz, not published until the drone is released at startup |
| `lidar_points` | `sensor_msgs/PointCloud2` | 28800 points, 10 Hz, NaN where a ray hits nothing |

Rates are in sim time, on `/clock`. Commands must keep streaming: the throttle is cut 0.5 s after the last one. The drone hovers at a throttle of about 0.5. The body frame is `uav1/fcu`, front-left-up, and `uav1/camera`, `uav1/lidar`, and `uav1/garmin` publish static transforms to it.

## Levels

- `DemoLevel`, the default, is a warehouse.
- `FlatWorld` is a template: a flat 2 x 2 km ground, a sky, a sun, and the drone. It renders the fastest, which is about 1.1 times real time on an integrated GPU.

To load another level, pass its name to `Tools/run_game.sh`. To change the default, edit `Registry/loadlevel.setreg` and run the Asset Processor, because the game reads a compiled copy of the file.

### Creating your own level

1. Copy the template: copy `Levels/FlatWorld` to `Levels/MyLevel`, and rename `FlatWorld.prefab` inside it to `MyLevel.prefab`. Or start from **File > New Level** in the Editor, opened with `Tools/run_editor.sh`.
2. Build your world on it. Everything the drone should touch needs a PhysX collider, because the lidar and the rigid body only see colliders. The drone waits for ground below it, up to 120 s, and then moves to just above it. If it finds none, it falls.
3. In a new level, add the drone: right-click, choose **Instantiate Prefab**, and pick `Assets/Drone/Multirotor.prefab`.
4. Move the `Multirotor` entity to the spawn point, about 0.155 m above the ground. Do not move the `uav1` entity inside it, because the sensors and the follow camera hang off it.
5. Press **Ctrl+G** to try it.

With the MRS UAV System, select the level with `export O3DE_LEVEL=mylevel`.

Avoid `Tools/create_drone_prefab.py` for this: it edits `DemoLevel`, and it overwrites changes made to the drone prefab in the Editor.

### Gaussian splat worlds (optional)

To fly in a Gaussian splat exported from ModelForge, you need its `ModelForgeSplat` Gem, version 0.12.0 or later, which is not part of this project and is not public:

1. Unpack the Gem anywhere, then register and enable it:
   ```sh
   /opt/O3DE/26.05/scripts/o3de.sh register --gem-path /path/to/ModelForgeSplat
   /opt/O3DE/26.05/scripts/o3de.sh enable-gem -gn ModelForgeSplat -pp ~/git/o3de_drone_sim
   ```
   This adds the Gem to `project.json`. Do not commit that change, because the project does not build without the Gem; `git update-index --skip-worktree project.json` keeps it out of `git status`.
2. Build again, as above.
3. Open a level with `Tools/run_editor.sh`, create an entity at the origin, and add a **ModelForge Splat** component to it.
4. In the component, click the browse button next to **Export folder** and select the ModelForge export: the folder that holds `splat.json`.
5. Move the drone's `Multirotor` entity just above the ground of the splat.

The drone waits for the splat's collision mesh, which can take seconds to load, before it is released. `Tools/bake_sky.py` bakes a sky without a horizon, so that nothing from a photo shows around the splat's edge.

## Repository

- `Gems/Multirotor`: the dynamics, controllers, and ROS interface.
- `Assets/Drone`: the drone prefab, and the x500 meshes from [mrs_uav_gazebo_simulation](https://github.com/ctu-mrs/mrs_uav_gazebo_simulation), BSD 3-Clause.
- `Tools`: `create_drone_prefab.py` generates the prefab, and `convert_x500_meshes.py` the meshes. Rerunning the former overwrites changes saved in the Editor.
- `CLAUDE.md`: development notes.
