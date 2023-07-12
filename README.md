# Obstacle Avoidance using PX4 and AirSim with ROS

Integration of some algorithms with PX4 and AirSim for real-time collision-free and obstacle-free trajectories in bounded environment.

This repo is only tested in Ubuntu18.04 with ROS Melodic.

<p align="center">
  <img src="demo_gif\demo_fast_planner.gif" width = "400" height = "225"/>
  <img src="demo_gif\demo_px4_avoidance.gif" width = "400" height = "225"/>
</p>

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

   1. Such as `roslaunch px4_avoidance_airsim start_fast_planner.launch` or `roslaunch px4_avoidance_airsim start_fast_planner.launch`
4. Have fun

## Speed up the mavros topics

Sometimes the communication between PX4 SITL and the AirSim is not fast enough. It leads to the very slow mavros topics, normally 4Hz for local_pose.

 You can modify the MAVLINK data rate in
 `ROMFS/px4fmu_common/init.d-posix/rcS`
 Line 274 to
 `mavlink start -x -u $udp_onboard_payload_port_local -r 4000000 -f -m onboard -o $udp_onboard_payload_port_remote`
(in v1.11.3 version).
In latest v1.13, it should be at `ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink` Line 29.

## UAV settings

MPC_XY_CRUISE 3m/s

MC_PITCHRATE_MAX 60 deg/s

MC_ROLLRATE_MAX 60 deg/s

MC_YAWRATE_MAX 60 deg/s

## Change Fast-planner goal height

method 1:

in `fast_planner/plan_manage/src/kino_replan_fsm.cpp`

```
if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {  
	end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;
}
```

method 2:

in `uav_simulator/Utils/waypoint_generator/src/waypoint_generator.cpp` line 146

```
voidgoal_callback(constgeometry_msgs::PoseStamped::ConstPtr&msg) 
```
