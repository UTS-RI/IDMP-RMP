"""
acceleration-based control for a ur5e
"""

from rmp2.utils.robot_config_utils import get_robot_urdf_path, get_robot_eef_uid
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from ur_ros_driver.srv import StartJog
from ur_ros_driver.msg import JogControl
import time
import tf2_ros


# control modes:

# CLOSED LOOP (NOT RECOMMENDED): Both the actual
# joint angles and actual joint velocities are 
# used to compute the reference next-step joint 
# angles and velocities for the low-level pd 
# controller. This often leads to unstable behavior 
# and is hence not recommended
CLOSED_LOOP = 0
# VELOCITY OPEN LOOP (DEFAULT): The actual joint 
# angles and virtual joint velocities (computed 
# through numerically integrating the accelerations) 
# are used to compute the reference joint angles and 
# velocities
VEL_OPEN_LOOP = 1
# OPEN LOOP: The virtual joint angles and joint 
# velocities (both computed through numerical
# integration) are used to compute the reference 
# joint angles and velocities
OPEN_LOOP = 2

class UR5e(object):
    """
    acceleration-based control for real robot
    """
    def __init__(self, time_step, mode=VEL_OPEN_LOOP):

        self.time_step = time_step

        assert mode == CLOSED_LOOP or mode == VEL_OPEN_LOOP or mode == OPEN_LOOP
        self._mode = mode

        self.joint_poses = None
        self.joint_vels = None
        self.joint_torques = None

        self.tcp_pose = None
        self.joint_poses_real = None
        self.joint_vels_real = None
        self.joint_torques_real = None

        self.target_joint_vels = None
        self.target_joint_poses = None

        self._joint_lower_limit = np.array([-6.28318531, -6.28318531, -3.14159265, -6.28318531, -6.28318531, -6.28318531])
        self._joint_upper_limit = np.array([6.28318531, 6.28318531, 3.14159265, 6.28318531, 6.28318531, 6.28318531])
        self._joint_vel_limit = np.array([3.14159265, 3.14159265, 3.14159265, 6.28318531, 6.28318531, 6.28318531])

        self.cspace_dim = 6

        rospy.Subscriber("/joint_states", JointState, self.callback_joint_state)
        rospy.Subscriber("/ur_hardware_interface/tcp_pose", TransformStamped, self.callback_tcp_pose)

        self.pub_jog = rospy.Publisher('/jog_control', JogControl, queue_size=10)

        self.jog_controll = False
        self.toggle_jog(True)

        #TF2 for link states
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf)
        time.sleep(2)
        self.time_buf = []
        self.last_time = 0
        self.reset()

    def toggle_jog(self,IO):
        print("Waiting on start_jog service")
        rospy.wait_for_service('/ur_hardware_interface/start_jog')
        print("Sending messageto start_jog service")
        try:
            start_jog = rospy.ServiceProxy('/ur_hardware_interface/start_jog', StartJog)
            resp1 = start_jog(IO, 1)
            if(resp1.success):
                if(IO):
                    print("Controll conection to robot is on")
                    self.jog_controll = True
                else:
                    print("Controll conection to robot is off")
                    self.jog_controll = False
            else:
                print("Some thing went wrong with the StartJog Service")
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def callback_tcp_pose(self,data):
        """
        callback tcp_pose
        """
        self.tcp_pose = np.array([data.transform.translation.x,
                                 data.transform.translation.y,
                                 data.transform.translation.z])

    def callback_joint_state(self,data):
        """
        callback joint_state_controller
        """
        temp_pos = np.asarray(data.position,dtype=np.float32)
        temp_pos[[0,2]] = temp_pos[[2,0]]
        self.joint_poses_real = temp_pos
        temp_vel = np.asarray(data.velocity,dtype=np.float32)
        temp_vel[[0,2]] = temp_vel[[2,0]]
        self.joint_vels_real = temp_vel
        temp_tor = np.asarray(data.effort,dtype=np.float32)
        temp_tor[[0,2]] = temp_tor[[2,0]]
        self.joint_torques_real = temp_tor

    def get_link_states(self,links):
        link_positions = {}
        for link in links:
            tf = self.buf.lookup_transform("base_link", link, rospy.Time()).transform
            link_positions[link] = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
        return link_positions

    def reset(self):
        """
        reset the robot to initial configuration and velocity
        """
        self.joint_poses = self.joint_poses_real
        self.joint_vels = self.joint_vels_real
        self.joint_torques = self.joint_torques_real

        self.target_joint_poses = self.joint_poses
        self.target_joint_vels = self.joint_vels

    def step(self, action):
        """
        apply velocity control to the robot
        :param action: joint accelerations
        """
        if self.joint_poses is None or self.joint_vels is None:
            raise Exception('Error: make sure to call reset() before step!')
        
        #measure time between last execution
        if self.last_time != 0:
            d_time = time.time_ns() - self.last_time
            if(len(self.time_buf) > 100):
                self.time_buf.pop(0)
            self.time_buf.append(d_time)
            self.time_step = np.mean(self.time_buf)/1e9
        self.last_time = time.time_ns()

        # forward predict using Euler integration
        self.target_joint_poses = self.joint_poses + self.joint_vels * self.time_step
        self.target_joint_vels = self.joint_vels + action * self.time_step
        # clip to respect joint limits and joint velocity limits
        self.target_joint_poses = np.clip(self.target_joint_poses, self._joint_lower_limit, self._joint_upper_limit)
        self.target_joint_vels = np.clip(self.target_joint_vels, -self._joint_vel_limit, self._joint_vel_limit)
        
        msg = JogControl()
        msg.stamp = rospy.get_rostime()
        msg.feature = 1
        msg.vector = self.target_joint_vels
        msg.acc =  3
        msg.time = self.time_step
        self.pub_jog.publish(msg)

    def get_observation(self):
        # """
        # joint angles and velocities of the robot.
        # for CLOSED_LOOP (NOT RECOMMENDED): both joint 
        #     angles and velocities are given by the robot
        # for VEL_OPEN_LOOP: joint angles are given by the
        #     robot, yet the joint velocities 
        #     are given by numerical integration
        # for OPEN_LOOP: both joint angles and velocities
        #     are given by numerical integration
        # """
        if self._mode == OPEN_LOOP:
            self.joint_poses = self.target_joint_poses
            self.joint_vels = self.target_joint_vels
        elif self._mode == VEL_OPEN_LOOP:
            self.joint_poses = self.joint_poses_real
            self.joint_vels = self.target_joint_vels
        elif self._mode == CLOSED_LOOP:
            self.joint_poses = self.joint_poses_real
            self.joint_vels = self.joint_vels_real
        return self.joint_poses.copy(), self.joint_vels.copy(), self.joint_torques.copy()

    def stop_motion(self):
        msg = JogControl()
        msg.stamp = rospy.get_rostime()
        msg.feature = 1
        msg.acc = 0.5
        msg.vector = [0,0,0,0,0,0]
        msg.time = 0.001
        self.pub_jog.publish(msg)

    def shutdown(self):
        self.stop_motion()
        self.toggle_jog(False)

def create_robot_real(time_step, mode=VEL_OPEN_LOOP):
    """
    create a acceleration-based control robot given name
    :param robot_name: robot name, 3link or franka
    :param time_step: simulation time between steps
    :param mode: control mode (see macros)
    :return robot_real: RobotReal object for 
    acceleration-based control of the robot
    """

    robot_real = UR5e(time_step=time_step, mode=mode)
    return robot_real

