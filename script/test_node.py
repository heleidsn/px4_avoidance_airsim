import rospy
from mavros_msgs.msg import PositionTarget


class MavrosControlTest:
    def __init__(self) -> None:
        rospy.init_node('test_node')

        traj_pub_vel_topic = "/mavros/setpoint_raw/local"

        self.vel_sp_pub = rospy.Publisher(traj_pub_vel_topic, PositionTarget, queue_size=10)

        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            control_msg = PositionTarget()
            control_msg.header.stamp = rospy.Time.now()
            control_msg.header.frame_id = 'local_origin'
            # BODY_NED
            control_msg.coordinate_frame = 8
            # use vx, vz, yaw_rate
            control_msg.type_mask = int('011111000111', 2)

            control_msg.velocity.x = 3   # 和ground speed一致
            control_msg.velocity.y = 0
            control_msg.velocity.z = 1   # 上升为正
            control_msg.yaw_rate = 0

            self.vel_sp_pub.publish(control_msg)
            rate.sleep()


if __name__ == '__main__':
    obj = MavrosControlTest()
