# Farm-NG Amiga + Kinova Gen3 Gazebo Sim (Jetty)

This repository bundles a Gazebo Sim (Jetty) model of the Farm-NG Amiga mobile base with a Kinova Kortex (Gen3) manipulator mounted on top. The goal is to offer a ROS 2 friendly simulation that exposes control topics for the differential drive base and the 7-DOF arm through `ros2_control`.

## Repository layout

```
models/
  amiga_kinova/
    model.sdf            # Combined robot definition for Gazebo Sim
    model.config         # Metadata for Gazebo Sim
    meshes/              # Simplified STL meshes for base, wheels and arm links
worlds/
  amiga_world.sdf        # Minimal flat world with the robot spawned
config/
  ros2_control.yaml      # Controller manager and ros2_control configuration
ros2/
  launch/amiga_sim.launch.py   # High-level launch entry point
  templates/gz_sim.launch.py   # Reusable Gazebo Sim launcher
```

## Usage

1. Install ROS 2 Humble (or newer) together with Gazebo Sim Jetty and the `ros_gz` bridges.
2. Clone this repository into a ROS 2 workspace and export the `models/` folder to Gazebo Sim via `GZ_SIM_RESOURCE_PATH`.
3. Launch the simulation:

   ```bash
   export GZ_SIM_RESOURCE_PATH=$(pwd)/models:$GZ_SIM_RESOURCE_PATH
   ros2 launch ros2/launch/amiga_sim.launch.py headless:=true
   ```

   The launch file starts Gazebo Sim, the controller manager with `ros2_control`, and a bridge exposing the differential drive and joint command topics.

4. Control the base with `/model/amiga_kinova/cmd_vel` (`geometry_msgs/Twist`) and the arm with the `kinova_arm_controller` trajectory interface.

## Testing

Run the static validation checks with:

```bash
pip install -r requirements.txt
pytest
```

The tests do not require ROS 2; they validate the SDF model, control configuration, and launch files structurally so that integration errors are caught early.

## Limitations

* The meshes are simplified geometric approximations meant to give the robot a clear footprint and collision geometry in simulation.
* Real Kinova dynamics and constraints are approximated; the configuration provides a functional set-up for control experiments but may require tuning for production scenarios.
