#!/usr/bin/env python

"""@trajectory_msg_converter.py
This node converts Fast-Planner reference trajectory message to MultiDOFJointTrajectory which is accepted by geometric_controller
Authors: Mohamed Abdelkader
"""

# Imports
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint # for geometric_controller
from quadrotor_msgs.msg import PositionCommand # for Fast-Planner
from geometry_msgs.msg import Transform, Twist, PoseStamped, Pose
from tf.transformations import quaternion_from_euler

class MessageConverter:
    def __init__(self):
        rospy.init_node('trajectory_msg_converter')

        fast_planner_traj_topic = "/planning/ref_traj"
        traj_pub_topic = "/mavros/setpoint_position/local"

        # Publisher for geometric_controller
        self.traj_pub = rospy.Publisher(traj_pub_topic, PoseStamped, queue_size=10)

        # Subscriber for Fast-Planner reference trajectory
        rospy.Subscriber(fast_planner_traj_topic, PositionCommand, self.fastPlannerTrajCallback, tcp_nodelay=True)

        rospy.spin()

    def fastPlannerTrajCallback(self, msg):
        
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = msg.position.x
        pose_msg.pose.position.y = msg.position.y
        pose_msg.pose.position.z = msg.position.z
        
        q = quaternion_from_euler(0, 0, msg.yaw) # RPY
        pose_msg.pose.orientation.w = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        
        pose_msg.header = msg.header

        self.traj_pub.publish(pose_msg)

if __name__ == '__main__':
    obj = MessageConverter()