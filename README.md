# Obstacle Avoidance using PX4 and AirSim

Integration of some algorithms with PX4 and AirSim for real-time collision-free and obstacle-free trajectories in bounded environment.

This repo is only tested in Ubuntu18.04 with ROS Melodic.

## Dependences

[AirSim](https://github.com/microsoft/AirSim) 1.8.1

- airsim_ros_pkgs is need as a ROS wrapper over the AirSim C++ client library.
- Please follow [this tutorial](https://microsoft.github.io/AirSim/airsim_ros_pkgs/) to build and use AirSim ROS wrapper.

[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) 1.13.0

- Clone PX4-Autopilot

[PX4-Avoidance](https://github.com/PX4/PX4-Avoidance)

[Fast-planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)

## Running in simulation

1. Start AirSim environment

   1. Download AirSim linux build from [here](https://github.com/microsoft/AirSim/releases/tag/v1.8.1-windows).
   2. Start environment.
2. Start simulation
   1. `roslaunch px4_avoidance_airsim start_simulation.launch`
   2. Note: The local-planner in PX4-avoidance and fast-planner using different config file. Please select the right config file when you start simulation using `roslaunch px4_avoidance_airsim start_simulation.launch`.
3. Launch planning algorithms
   1. Such as `roslaunch px4_avoidance_airsim start_fast_planner.launch`
4. Have fun

## UAV settings

MPC_XY_CRUISE 3m/s

MC_PITCHRATE_MAX 60 deg/s

MC_ROLLRATE_MAX 60 deg/s

MC_YAWRATE_MAX 60 deg/s
