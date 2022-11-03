#!/usr/bin/env python3
import rospy
import rospkg
import os
import math
import random

from cv_bridge import CvBridge, CvBridgeError
import subprocess32 as subprocess
import numpy as np

from mavros_msgs.msg import State
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import ParamSet, SetMode, CommandBool, CommandTOL
from mavros_msgs.srv import CommandHome, CommandLong
from airsim_ros_pkgs.srv import Reset

from tf.transformations import quaternion_from_euler, euler_from_quaternion


class PX4Evaluation():
    """
    px4 airsim evaluation node
    create reset_sim, log etc function
    running at 10hz
    input:
        /input_setpoint_position_local
    output:
        /mavros/setpoint_position/local
    """

    def __init__(self):
        rospy.logdebug('start px4_evaluation init node')
        rospy.init_node('px4_evaluation', anonymous=False,
                        log_level=rospy.INFO)

        self.px4_ekf2_path = os.path.join(rospkg.RosPack().get_path("px4"),
                                          "build/px4_sitl_default/bin/px4-ekf2"
                                          )
        self.bridge = CvBridge()
        self._rate = rospy.Rate(10.0)
        self.control_method = 'velocity'   # choose velocity control or position control

        # Subscribers
        rospy.Subscriber('/mavros/state', State,
                         callback=self.mav_state_cb, queue_size=10)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                         callback=self.local_position_cb, queue_size=10)
        rospy.Subscriber('/input_setpoint_position_local', PoseStamped,
                         callback=self.pose_sp_cmd_cb, queue_size=10)
        rospy.Subscriber('/input_setpoint_velocity_local', PositionTarget,
                         callback=self.vel_sp_cmd_cb, queue_size=10)
        rospy.Subscriber('/depth_clip_20', Image,
                         callback=self.depth_image_cb, queue_size=10)

        # Publishers
        self.pub_pose_sp = rospy.Publisher('/mavros/setpoint_position/local',
                                           PoseStamped, queue_size=10)
        self.pub_vel_sp = rospy.Publisher('/mavros/setpoint_raw/local',
                                          PositionTarget, queue_size=10)
        self.pub_goal_pose = rospy.Publisher('/move_base_simple/goal',
                                             PoseStamped, queue_size=10)
        self.pub_init_fast_planner = rospy.Publisher('/init_planner',
                                                     String, queue_size=10)

        # Services
        self.reset_sim_client = rospy.ServiceProxy('/airsim_node/reset',
                                                   Reset)
        self._arming_client = rospy.ServiceProxy('mavros/cmd/arming',
                                                 CommandBool)
        self._set_mode_client = rospy.ServiceProxy('mavros/set_mode',
                                                   SetMode)
        self._change_param = rospy.ServiceProxy('/mavros/param/set',
                                                ParamSet)
        self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff',
                                                 CommandTOL)
        self.set_home_client = rospy.ServiceProxy('/mavros/cmd/set_home',
                                                  CommandHome)
        self._mavros_cmd_client = rospy.ServiceProxy('/mavros/cmd/command',
                                                     CommandLong)

        # msg variables
        self.setpoint_position = PoseStamped()
        self.setpoint_velocity = PositionTarget()
        self.depth_img_msg = 0
        self.depth_image_meter = 0
        self.current_pose_local = PoseStamped()
        self.current_state = State()
        self._goal_pose = PoseStamped()

        # training info
        self.episode_num = 0
        self.step_num = 0
        self.total_step = 0
        self.cumulated_episode_reward = 0

        self.last_obs = 0
        self.previous_distance_from_des_point = 0

        self.takeoff_hight = 5
        self.random_start_direction = True
        self.goal_distance = 100
        self.goal_angle_noise_degree = 180  # random goal direction

        self.work_space_x = [-140, 140]
        self.work_space_y = [-140, 140]
        self.work_space_z = [0.5, 20]
        self.max_episode_step = 1500

        '''
        Settings for termination
        '''

        self.accept_radius = 3
        self.min_dist_to_obs_meters = 1

        rospy.logdebug('px4_evaluation node initialized...')

    # -----------------callback functions---------------------

    def pose_sp_cmd_cb(self, msg):
        self.setpoint_position = msg

    def vel_sp_cmd_cb(self, msg):
        self.setpoint_velocity = msg

    def local_position_cb(self, msg):
        self.current_pose_local = msg

    def mav_state_cb(self, msg):
        self.current_state = msg

    def depth_image_cb(self, msg):
        depth_img_msg = msg
        try:
            cv_image = self.bridge.imgmsg_to_cv2(depth_img_msg,
                                                 desired_encoding=depth_img_msg.encoding)
        except CvBridgeError as e:
            print(e)

        image = np.array(cv_image, dtype=np.float32)
        self.depth_image_meter = image
        # print(self.depth_image_meter.min())

    # -----------------openAI gym APIs-----------------------

    def step(self, action):
        rospy.logdebug('step function start')
        self.set_action(action)

        obs = self.get_obs_new()
        done = self.is_done()
        info = {
            'is_success': self.is_in_desired_pose(),
            'is_crash': self.too_close_to_obstacle(),
            'step_num': self.step_num
        }
        reward = 0

        self.cumulated_episode_reward += reward
        self.step_num += 1
        self.total_step += 1

        self.last_obs = obs
        rospy.logdebug('step function end')

        return obs, reward, done, info

    def compute_reward(self, done, action):
        reward = 0
        reward_reach = 10
        reward_crash = -20
        reward_outside = -10

        if not done:
            distance_now = self.get_distance_to_goal_3d()
            reward_distance = (self.previous_distance_from_des_point - distance_now) / \
                self.dynamic_model.goal_distance * \
                500  # normalized to 100 according to goal_distance
            self.previous_distance_from_des_point = distance_now

            reward_obs = 0
            action_cost = 0

            # add yaw_rate cost
            yaw_speed_cost = 0.1 * \
                abs(action[-1]) / self.dynamic_model.yaw_rate_max_rad

            if self.dynamic_model.navigation_3d:
                # add action and z error cost
                v_z_cost = 0.1 * abs(action[1]) / self.dynamic_model.v_z_max
                z_err_cost = 0.2 * \
                    abs(self.dynamic_model.state_raw[1]) / \
                    self.dynamic_model.max_vertical_difference
                action_cost += v_z_cost + z_err_cost

            action_cost += yaw_speed_cost

            yaw_error = self.dynamic_model.state_raw[2]
            yaw_error_cost = 0.1 * abs(yaw_error / 180)

            reward = reward_distance - reward_obs - action_cost - yaw_error_cost
        else:
            if self.is_in_desired_pose():
                reward = reward_reach
            if self.is_crashed():
                reward = reward_crash
            if self.is_not_inside_workspace():
                reward = reward_outside

        return reward

    def get_obs(self):
        # get depth image from current topic
        # Note: check image format. Now is 0-black near 255-wight far
        image = self._depth_image_gray.copy()

        # transfer image to image obs according to 0-far  255-nears
        image_obs = 255 - image

        # publish image_obs
        image_obs_msg = self.bridge.cv2_to_imgmsg(image_obs)
        self._depth_image_gray_input.publish(image_obs_msg)

        state_feature_array = np.zeros((self.screen_height, self.screen_width))

        state_feature = self._get_state_feature()

        state_feature_array[0, 0:self.state_length] = state_feature

        image_with_state = np.array([image_obs, state_feature_array])
        image_with_state = image_with_state.swapaxes(0, 2)
        image_with_state = image_with_state.swapaxes(0, 1)

        return image_with_state

    def get_obs_new(self):
        return 0

    def set_action(self, action):
        if self.control_method == 'position':
            self.pub_pose_sp.publish(self.setpoint_position)
        elif self.control_method == 'velocity':
            # need to get velocity from position and yaw error
            control_msg = PositionTarget()
            control_msg.header.stamp = rospy.Time.now()
            control_msg.header.frame_id = 'local_origin'
            # BODY_NED
            control_msg.coordinate_frame = 8
            # use vx, vz, yaw_rate
            control_msg.type_mask = int('011111000111', 2)

            x_err = self.setpoint_position.pose.position.x - \
                self.current_pose_local.pose.position.x
            y_err = self.setpoint_position.pose.position.y - \
                self.current_pose_local.pose.position.y
            z_err = self.setpoint_position.pose.position.z - \
                self.current_pose_local.pose.position.z

            control_msg.velocity.x = math.sqrt(x_err*x_err + y_err*y_err)
            control_msg.velocity.y = 0
            # control_msg.velocity.x = x_err
            # control_msg.velocity.y = y_err
            control_msg.velocity.z = z_err

            if math.sqrt(x_err*x_err + y_err*y_err) < 0.001:
                control_msg.yaw_rate = 0
            else:
                # calculate yaw according to x_err and y_err
                yaw_current = self.get_euler_from_pose(
                    self.current_pose_local.pose)[2]
                q = np.empty((4, ), dtype=np.float64)
                pose = self.setpoint_position.pose
                q[0] = pose.orientation.x
                q[1] = pose.orientation.y
                q[2] = pose.orientation.z
                q[3] = pose.orientation.w
                euler_rad = euler_from_quaternion(q)
                yaw_setpoint = euler_rad[2]
                yaw_error = yaw_setpoint - yaw_current
                if yaw_error > math.pi:
                    yaw_error -= 2*math.pi
                elif yaw_error < -math.pi:
                    yaw_error += 2*math.pi
                control_msg.yaw_rate = yaw_error * 2.8
                # print(yaw_error * 57.3)
            self.pub_vel_sp.publish(control_msg)
        else:
            print('error: bad control_method, should be position or velocity')

        self._rate.sleep()

    def reset(self):
        '''
        include reset sim and reset env
        reset_sim: reset simulation and ekf2
        reset_env: arm uav and takeoff
        '''
        rospy.logdebug('reset function in')

        self.reset_sim()

        self.episode_num += 1
        self.cumulated_episode_reward = 0
        self.step_num = 0

        obs = self.get_obs_new()
        self.last_obs = obs

        rospy.logdebug("reset function out")
        return obs

    def reset_sim(self):
        """
        Including ***ekf2 stop*** and ***ekf2 start***
        目前要解决的问题是：
        1. 无人机在空中无法disarm，尝试从固件入手
        2. 然后尝试先disarm，然后reset sim，最后reset ekf2.
        问题解决：在固件中设置使用强制arm and disarm

        遇到问题：
        stop ekf2之后出现断开连接，无法重新连接。。。。
        估计是需要旧版本固件。。。
        """
        rospy.logdebug("RESET SIM START --- DONT RESET CONTROLLERS")

        # 1. disarm
        self.DisarmForce()

        # 2. reset sim
        self.reset_airsim()

        self.DisarmForce()

        # 4. reset ekf2
        rospy.logdebug("reset ekf2 module")
        subprocess.Popen([self.px4_ekf2_path, "stop"])
        rospy.sleep(1)
        subprocess.Popen([self.px4_ekf2_path, "start"])
        rospy.sleep(2)

        # 0. stop fast planner
        init_msg = String()
        self.pub_init_fast_planner.publish(init_msg)

        # 5. arm
        self.Arm()

        self.TakeOff()

        self.update_goal_pose_rect()

        rospy.logdebug("RESET SIM END")

    def update_goal_pose(self):
        distance = self.goal_distance

        noise = np.random.random() * 2 - 1
        angle = noise * math.radians(self.goal_angle_noise_degree)

        goal_x = distance * math.cos(angle)
        goal_y = distance * math.sin(angle)

        self._goal_pose = PoseStamped()
        self._goal_pose.pose.position.x = goal_x
        self._goal_pose.pose.position.y = goal_y
        self._goal_pose.pose.position.z = self.takeoff_hight
        rospy.loginfo('change goal pose to {}, {}'.format(goal_x, goal_y))

        self.pub_goal_pose.publish(self._goal_pose)

    def update_goal_pose_rect(self):
        # if goal is given by rectangular mode
        goal_rect = [-128, -128, 128, 128]
        goal_x, goal_y = self.get_goal_from_rect(goal_rect, math.pi*2)
        self.goal_distance = math.sqrt(goal_x*goal_x + goal_y*goal_y)

        self._goal_pose = PoseStamped()
        self._goal_pose.pose.position.x = goal_x
        self._goal_pose.pose.position.y = goal_y
        self._goal_pose.pose.position.z = self.takeoff_hight
        rospy.loginfo('change goal pose to {}, {}'.format(goal_x, goal_y))

        self.pub_goal_pose.publish(self._goal_pose)

    def get_goal_from_rect(self, rect_set, random_angle_set):
        rect = rect_set
        random_angle = random_angle_set
        noise = np.random.random()  # [0,1])
        angle = random_angle * noise - math.pi   # -pi~pi
        rect = [-128, -128, 128, 128]

        if abs(angle) == math.pi/2:
            goal_x = 0
            if angle > 0:
                goal_y = rect[3]
            else:
                goal_y = rect[1]
        if abs(angle) <= math.pi/4:
            goal_x = rect[2]
            goal_y = goal_x*math.tan(angle)
        elif abs(angle) > math.pi/4 and abs(angle) <= math.pi/4*3:
            if angle > 0:
                goal_y = rect[3]
                goal_x = goal_y/math.tan(angle)
            else:
                goal_y = rect[1]
                goal_x = goal_y/math.tan(angle)
        else:
            goal_x = rect[0]
            goal_y = goal_x * math.tan(angle)

        return goal_x, goal_y

    def is_done(self):
        """
        The done can be done due to three reasons:
        1) It went outside the workspace
        2) It detected something with the sonar that is too close
        3) It flipped due to a crash or something
        4) It has reached the desired point
        5) It is too close to the obstacle
        """

        episode_done = False

        # is_inside_workspace_now = self.is_inside_workspace()
        is_inside_workspace_now = True
        has_reached_des_pose = self.is_in_desired_pose()
        # has_reached_des_pose = False
        too_close_to_obstacle = self.too_close_to_obstacle()
        # too_close_to_obstacle = False

        too_much_steps = (self.step_num > self.max_episode_step)

        # rospy.logdebug(">>>>>> DONE RESULTS <<<<<")

        if not is_inside_workspace_now:
            rospy.loginfo("is_inside_workspace_now=" +
                          str(is_inside_workspace_now))
        if has_reached_des_pose:
            rospy.loginfo("has_reached_des_pose="+str(has_reached_des_pose))
        if too_close_to_obstacle:
            rospy.loginfo("has crashed to the obstacle=" +
                          str(too_close_to_obstacle))
        if too_much_steps:
            rospy.loginfo("too much steps=" + str(too_much_steps))

        episode_done = (not(is_inside_workspace_now) or
                        has_reached_des_pose or
                        too_close_to_obstacle or
                        too_much_steps)

        return episode_done

    def is_inside_workspace(self):
        """
        Check if the Drone is inside the Workspace defined
        """
        is_inside = False
        current_position = self.current_pose_local.pose.position

        if (current_position.x > self.work_space_x_min and current_position.x <= self.work_space_x_max):
            if current_position.y > self.work_space_y_min and current_position.y <= self.work_space_y_max:
                if current_position.z > self.work_space_z_min and current_position.z <= self.work_space_z_max:
                    is_inside = True

        return is_inside

    def is_in_desired_pose(self):
        """
        It return True if the current position is similar to the desired
        position
        """
        in_desired_pose = False
        current_pose = self.current_pose_local

        if self.get_distance_from_desired_point(current_pose.pose.position) < self.accept_radius:
            in_desired_pose = True

        return in_desired_pose

    def too_close_to_obstacle(self):
        """
        Check the distance to the obstacle using the depth perception
        """
        too_close = False

        if self.depth_image_meter.min() < self.min_dist_to_obs_meters:
            print('min_dist: ', self.depth_image_meter.min())
            too_close = True
        # print(self.depth_image_meter.min())

        return too_close

    def get_distance_from_desired_point(self, current_position):
        """
        Calculates the distance from the current position to the desired point
        :param start_point:
        :return:
        """
        curr_position = np.array(
            [current_position.x, current_position.y, current_position.z])
        des_position = np.array([self._goal_pose.pose.position.x,
                                 self._goal_pose.pose.position.y,
                                 self._goal_pose.pose.position.z])
        distance = self.get_distance_between_points(
            curr_position, des_position)

        return distance

    def get_distance_between_points(self, p_start, p_end):
        """
        Given a Vector3 Object, get distance from current position
        :param p_end:
        :return:
        """
        a = np.array(p_start)
        b = np.array(p_end)

        distance = np.linalg.norm(a - b)

        return distance

    def get_euler_from_pose(self, pose):
        q = np.empty((4, ), dtype=np.float64)
        q[0] = pose.orientation.x
        q[1] = pose.orientation.y
        q[2] = pose.orientation.z
        q[3] = pose.orientation.w

        euler_rad = euler_from_quaternion(q)

        return euler_rad

    # region Methods for PX4 control

    def TakeOff(self):
        # method 2: using offboard mode to take off
        pose_setpoint = PoseStamped()
        pose_setpoint.pose.position.x = 0
        pose_setpoint.pose.position.y = 0
        pose_setpoint.pose.position.z = self.takeoff_hight + 1

        # random yaw angle at start point
        if self.random_start_direction:
            yaw_angle_rad = math.pi * (2 * random.random() - 1)
        else:
            yaw_angle_rad = 0
        orientation_setpoint = quaternion_from_euler(0, 0, yaw_angle_rad)
        pose_setpoint.pose.orientation.x = orientation_setpoint[0]
        pose_setpoint.pose.orientation.y = orientation_setpoint[1]
        pose_setpoint.pose.orientation.z = orientation_setpoint[2]
        pose_setpoint.pose.orientation.w = orientation_setpoint[3]

        for i in range(10):
            self.pub_pose_sp.publish(pose_setpoint)

        self.setMavMode('OFFBOARD', 5)

        # print(self.current_pose_local.pose.position.z)

        while (self.current_pose_local.pose.position.z <
               self.takeoff_hight + 1):
            # 1. publish topics

            self.pub_pose_sp.publish(pose_setpoint)

            # 2. check arm status, if not, arm first
            if not self.current_state.armed:
                rospy.logdebug('ARM AGAIN')
                self.Arm()

            if self.current_state.mode != 'OFFBOARD':
                rospy.logdebug('SET OFFBOARD AGAIN')
                self.setMavMode('OFFBOARD', 5)

            rospy.sleep(0.1)

        rospy.logdebug("Took off success")

    def Arm(self):
        rospy.logdebug("wait for armed")
        rospy.wait_for_service("mavros/cmd/arming")

        while not self.current_state.armed:
            self._arming_client.call(True)
            rospy.sleep(0.1)

        # rospy.wait_for_service("/mavros/cmd/command")

        # ret = self._mavros_cmd_client.call(False,
        #                                    400, 0,
        #                                    1, 21196,
        #                                    0.0, 0.0, 0.0, 0.0, 0.0)
        # while not ret.success:
        #     rospy.logdebug('repeat arm')
        #     ret = self._mavros_cmd_client.call(False,
        #                                        400, 0,
        #                                        1, 21196,
        #                                        0.0, 0.0, 0.0, 0.0, 0.0)
        #     rospy.sleep(0.1)

        rospy.logdebug("ARMED!!!")

    def Disarm(self):
        rospy.logdebug("wait for disarmed")
        rospy.wait_for_service("mavros/cmd/arming")

        while self.current_state.armed:
            self._arming_client.call(False)
            rospy.sleep(0.1)

        rospy.sleep(5)

        rospy.logdebug("DISARMED!!!")

    def DisarmForce(self):
        rospy.logdebug("wait for disarmed force")
        rospy.wait_for_service("/mavros/cmd/command")

        ret = self._mavros_cmd_client.call(False,
                                           400, 0,
                                           0.0, 21196.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0)
        while not ret.success:
            ret = self._mavros_cmd_client.call(False,
                                               400, 0,
                                               0.0, 21196.0,
                                               0.0, 0.0, 0.0, 0.0, 0.0)
            rospy.sleep(0.1)
        rospy.logdebug("DISARMED!!!")

    def SetHomePose(self):
        '''
        this function is very slow, need to be optimized
        '''
        # set home position to current gps position
        # print(self.curr_gps.latitude, self.curr_gps.longitude)
        rospy.logdebug("Setting home pose...")

        ret = self.set_home_client(curr_gps=True,
                                   latitude=self.curr_gps.latitude,
                                   longitude=self.curr_gps.longitude,
                                   altitude=self.curr_gps.altitude)

        while not ret.success:
            # rospy.logwarn("Failed to set home, try again")
            ret = self.set_home_client(curr_gps=True,
                                       latitude=self.curr_gps.latitude,
                                       longitude=self.curr_gps.longitude,
                                       altitude=self.curr_gps.altitude)

        self.home_init = True
        rospy.logdebug("Set home pose success!")

    def setMavMode(self, mode, timeout):
        """mode: PX4 mode string, timeout(int): seconds"""
        rospy.logdebug("setting FCU mode: {0}".format(mode))

        loop_freq = 5  # Hz
        rate_new = rospy.Rate(loop_freq)

        for i in range(timeout * loop_freq):
            if self.current_state.mode == mode:
                rospy.logdebug("set mode success | seconds: {0} of {1}".
                               format(i / loop_freq, timeout))
                break
            else:
                # rospy.logdebug('current mode: {0}, try to set to {1}'.
                #                format(self.current_state.mode, mode))
                try:
                    res = self._set_mode_client(0, mode)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            rate_new.sleep()

    def reset_airsim(self):
        rospy.wait_for_service('/airsim_node/reset')

        ret_reset_sim = self.reset_sim_client.call(False)
        while not ret_reset_sim:
            ret_reset_sim = self.reset_sim_client.call(False)
            rospy.sleep(0.1)
            rospy.logdebug('waiting for airsim reset')

        rospy.logdebug('airsim reset ok')

    def get_distance_to_goal_3d(self):
        current_pose = self.current_pose_local.pose.position
        goal_pose = self._goal_pose.pose.position
        relative_pose_x = current_pose.x - goal_pose.x
        relative_pose_y = current_pose.y - goal_pose.y
        relative_pose_z = current_pose.z - goal_pose.z

        return math.sqrt(pow(relative_pose_x, 2) + pow(relative_pose_y, 2) + pow(relative_pose_z, 2))
    # --------------------evaluation function-----------------------

    def evaluation(self, eval_ep_num):
        # 以10hz发布控制指令
        # 然后获取当前的obs， reward等，并判断有没有结束
        episode_num = 1

        episode_successes = []
        episode_crash = []
        step_num_list = []
        reward_sum = np.array([.0])

        _ = self.reset()

        while episode_num <= eval_ep_num:

            action = 'test action'
            _, _, done, info = self.step(action)
            if done:
                maybe_is_success = info.get('is_success')
                maybe_is_crash = info.get('is_crash')

                episode_successes.append(float(maybe_is_success))
                episode_crash.append(float(maybe_is_crash))

                if maybe_is_success:
                    step_num_list.append(info.get('step_num'))

                print('episode: ', episode_num,
                      ' reward:', 0,
                      'success:', maybe_is_success)

                _ = self.reset()
                episode_num += 1

        print('Average episode reward: ', reward_sum[:eval_ep_num].mean(),
              'Success rate:', np.mean(episode_successes),
              'Crash rate:', np.mean(episode_crash),
              'average step num: ', np.mean(step_num_list))


if __name__ == '__main__':
    eval_ep_num = 50
    try:
        eval_node = PX4Evaluation()
        eval_node.evaluation(eval_ep_num)
    except rospy.ROSInterruptException:
        pass
