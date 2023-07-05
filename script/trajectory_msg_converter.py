#!/usr/bin/env python

"""@trajectory_msg_converter.py
This node converts Fast-Planner reference trajectory message to mavros message
Authors: Lei He
"""

# Imports
import math
import rospy
from quadrotor_msgs.msg import PositionCommand   # for Fast-Planner
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from tf.transformations import quaternion_from_euler


class MessageConverter:
    def __init__(self):
        rospy.init_node('trajectory_msg_converter')

        self.flag_use_velocity_contorl = True

        fast_planner_traj_topic = "/planning/ref_traj"
        traj_pub_topic = "/mavros/setpoint_position/local"
        traj_pub_vel_topic = "/mavros/setpoint_raw/local"

        # Publisher, you can choose position control or velocity control
        self.pose_sp_pub = rospy.Publisher(traj_pub_topic, PoseStamped, queue_size=10)
        self.vel_sp_pub = rospy.Publisher(traj_pub_vel_topic, PositionTarget, queue_size=10)

        # Subscriber for Fast-Planner reference trajectory
        rospy.Subscriber(fast_planner_traj_topic, PositionCommand, self.fastPlannerTrajCallback, tcp_nodelay=True)

        rospy.spin()

    def fastPlannerTrajCallback(self, msg):

        if self.flag_use_velocity_contorl:

            # transfer fast-planner msg to velocity control msg
            control_msg = PositionTarget()
            control_msg.header.stamp = rospy.Time.now()
            control_msg.header.frame_id = 'local_origin'
            # LOCAL_NED
            control_msg.coordinate_frame = 1
            # use x, y, z, vx, vz, yaw, yaw_rate
            control_msg.type_mask = int('001111000000', 2)
            
            control_msg.position.x = msg.position.x
            control_msg.position.y = msg.position.y
            control_msg.position.z = msg.position.z

            local_vx = msg.velocity.x
            local_vy = msg.velocity.y
            local_vz = msg.velocity.z
            yaw_dot = msg.yaw_dot
    
            control_msg.velocity.x = local_vx
            control_msg.velocity.y = local_vy
            control_msg.velocity.z = local_vz
            
            control_msg.yaw = msg.yaw
            
            control_msg.yaw_rate = yaw_dot
            self.vel_sp_pub.publish(control_msg)
        else:
            # transfer fast-planner msg to position control msg
            pose_msg = PoseStamped()
            pose_msg.pose.position.x = msg.position.x
            pose_msg.pose.position.y = msg.position.y
            pose_msg.pose.position.z = msg.position.z

            q = quaternion_from_euler(0, 0, msg.yaw)  # RPY
            pose_msg.pose.orientation.x = q[0]
            pose_msg.pose.orientation.y = q[1]
            pose_msg.pose.orientation.z = q[2]
            pose_msg.pose.orientation.w = q[3]

            pose_msg.header = msg.header

            self.pose_sp_pub.publish(pose_msg)


if __name__ == '__main__':
    obj = MessageConverter()
