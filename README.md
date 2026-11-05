# o3de_drone_sim

A multirotor simulator in [O3DE](https://o3de.org) with a ROS 2 interface. An x500 quadrotor flies with a forward RGB camera and a Livox Mid-360-style lidar, in a warehouse or in a Gaussian splat world.

The dynamics and the attitude and rate controllers run inside O3DE at 500 Hz. ROS 2 carries attitude or attitude-rate commands in and sensor data out, using standard messages only. To fly it with the [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system), use [mrs_uav_o3de_api](https://github.com/klaxalk/mrs_uav_o3de_api).

## Requirements

- O3DE 26.05 SDK, registered in `~/.o3de/o3de_manifest.json`, on Linux
- ROS 2 Jazzy, with `control_msgs` and `ackermann_msgs`, which the `ROS2` Gem requires
- Clang
- These Gems, registered with `o3de.sh register --gem-path`: `ROS2` 4.3.0, `ROS2Sensors`, `LevelGeoreferencing`, and `WarehouseAssets`

## Build

ROS 2 must be sourced:

```sh
source /opt/ros/jazzy/setup.zsh
CC=clang CXX=clang++ cmake -B build/linux -S . -G "Ninja Multi-Config"
cmake --build build/linux --config profile --target Editor DroneSim.GameLauncher -j 8
```

## Run

**Start the Editor and Project Manager only with this script.** It finds the engine from the O3DE manifest, or from `O3DE_ENGINE_PATH`, sources ROS 2, and uses `rmw_fastrtps_cpp` unless `RMW_IMPLEMENTATION` is set:

```sh
Tools/run_editor.sh              # Editor
Tools/run_editor.sh --manager    # Project Manager
```

Without ROS 2 sourced, the ROS and Multirotor Gems fail to load, without an error that stops anything. The Asset Processor then compiles prefabs without their components, and saving in the Editor removes the components from the prefabs for good. If the drone has no topics, delete the `.spawnable` files under `Cache/linux` and run `AssetProcessorBatch` with ROS 2 sourced.

To run the game, process the assets once by opening the Editor or running `AssetProcessorBatch`, then run the launcher. Set `RMW_IMPLEMENTATION` to match your other ROS nodes. With `rmw_zenoh_cpp`, also start a router with `ros2 run rmw_zenoh_cpp rmw_zenohd`.

```sh
source /opt/ros/jazzy/setup.zsh
cd build/linux/bin/profile
./DroneSim.GameLauncher --project-path=$HOME/git/o3de_drone_sim --bg_ConnectToAssetProcessor=0 \
  --t_simulationTickDeltaOverride=17000 --r_renderScale=0.5
```

`--t_simulationTickDeltaOverride=17000` advances the simulation 17 ms per rendered frame, in microseconds. Slow frames then slow the simulation down, instead of looking like data dropouts to ROS. Without it, flying at low frame rates is unstable. All ROS nodes need `use_sim_time:=true`. `--r_renderScale` lowers the game view's resolution, not the camera's. Use `--rhi=null` to run without rendering, and so without the camera.

In the game, press **C** to switch between following the drone and a free camera: WASD to move, Q and E for down and up, the right mouse button to look around, and left shift to go faster. In the free camera, **T** keeps the view aimed at the drone while you move. **H** hides the guide in the game view.

## ROS 2 interface

Topics are under the drone's namespace, `/uav1`.

| Topic | Type | Notes |
|---|---|---|
| `cmd/attitude_rate` | `geometry_msgs/TwistStamped` | `twist.angular`: body rate [rad/s], `twist.linear.z`: throttle [0, 1] |
| `cmd/attitude` | `geometry_msgs/PoseStamped` | `pose.orientation`: attitude, `pose.position.z`: throttle [0, 1] |
| `odom` | `nav_msgs/Odometry` | 250 Hz, relative to the spawn point, twist in the body frame |
| `imu` | `sensor_msgs/Imu` | 250 Hz |
| `camera_image_color`, `color_camera_info` | `sensor_msgs/Image`, `CameraInfo` | 1280x720, 30 Hz |
| `lidar_points` | `sensor_msgs/PointCloud2` | 28800 points, 10 Hz, NaN where a ray hits nothing |

Rates are in sim time, on `/clock`. Commands must keep streaming: the throttle is cut 0.5 s after the last one. The drone hovers at a throttle of about 0.5. The body frame is `uav1/fcu`, front-left-up, and `uav1/camera` and `uav1/lidar` publish static transforms to it.

## Levels

- `DemoLevel`, the default, is a warehouse.
- `FlatWorld` is a template: a flat 2 x 2 km ground, a sky, a sun, and the drone. It renders the fastest, which is about 1.1 times real time on an integrated GPU.

To load another level, pass it to the game in lowercase, as in `--loadlevel=levels/flatworld/flatworld.spawnable`. To change the default, edit `Registry/loadlevel.setreg` and run the Asset Processor, because the game reads a compiled copy of the file.

### Creating your own level

1. Copy the template: copy `Levels/FlatWorld` to `Levels/MyLevel`, and rename `FlatWorld.prefab` inside it to `MyLevel.prefab`. Or start from **File > New Level** in the Editor, opened with `Tools/run_editor.sh`.
2. Build your world on it. Everything the drone should touch needs a PhysX collider, because the lidar and the rigid body only see colliders. The drone waits for ground below it, up to 120 s, and then falls.
3. In a new level, add the drone: right-click, choose **Instantiate Prefab**, and pick `Assets/Drone/Multirotor.prefab`.
4. Move the `Multirotor` entity to the spawn point, about 0.155 m above the ground. Do not move the `uav1` entity inside it, because the sensors and the follow camera hang off it.
5. Press **Ctrl+G** to try it.

With the MRS UAV System, select the level with `export O3DE_LEVEL=mylevel`, and if the drone does not spawn at 0.155 m above the ground, set `rangefinder/ground_z` in `mrs_uav_o3de_api` to minus the spawn height. The odometry starts at the spawn point. The rangefinder assumes a flat floor there, so it is only approximate on real terrain.

Avoid `Tools/create_drone_prefab.py` for this: it edits `DemoLevel`, and it overwrites changes made to the drone prefab in the Editor.

### Gaussian splat worlds (optional)

To fly in a splat exported from [ModelForge](https://github.com/klaxalk/modelforge), add its `ModelForgeSplat` Gem, which is not part of this project:

1. Download the Gem, then register and enable it:
   ```sh
   <engine>/scripts/o3de.sh register --gem-path /path/to/ModelForgeSplat
   <engine>/scripts/o3de.sh enable-gem -gn ModelForgeSplat -pp /path/to/o3de_drone_sim
   ```
2. Build again, as above.
3. Open a level with `Tools/run_editor.sh`, add a **ModelForge Splat** component to an entity at the origin, and set its **Export folder** to the export.
4. Set the drone's position just above the ground.

The drone waits for the collision mesh, which can take seconds to load, before it is released. The HW API's rangefinder assumes a flat floor, so it is only approximate on real terrain.

## Repository

- `Gems/Multirotor`: the dynamics, controllers, and ROS interface.
- `Assets/Drone`: the drone prefab, and the x500 meshes from [mrs_uav_gazebo_simulation](https://github.com/ctu-mrs/mrs_uav_gazebo_simulation), BSD 3-Clause.
- `Tools`: `create_drone_prefab.py` generates the prefab, and `convert_x500_meshes.py` the meshes. Rerunning the former overwrites changes saved in the Editor.
- `CLAUDE.md`: development notes.
