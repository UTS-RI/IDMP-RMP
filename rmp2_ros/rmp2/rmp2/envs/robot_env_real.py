"""
Base environment for robots
"""

import rospy
import time
import numpy as np
from abc import abstractmethod
from rmp2.envs import ur5e_real
from rmp2.utils.python_utils import merge_dicts
from pkg_resources import parse_version
from rmp2.utils.robot_config_utils import load_robot_config, get_robot_urdf_path
from idmp_ros.srv import GetDistanceGradient
from rmp2_ros.srv import SetCollRadii, SetCollRadiiRequest

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt

DEFAULT_CONFIG = {
    "time_step": 1/60.,
    "action_repeat": 1,
    "goal": None,
    "q_init": None,
    "acc_control_mode": ur5e_real.VEL_OPEN_LOOP,
}


class RobotEnvReal:
    """
    Base environment for robots
    """

    def __init__(self,
                robot_name,
                workspace_dim,
                config=None):
        """
        :param robot_name: str, the name of the robot, 3link or franka
        :param workspace_dim: int, workspace dimension, either 2 or 3
        :param config: dict, for overwriting the default configs
        """
        # merge config with default config
        if config is not None:
            config = merge_dicts(DEFAULT_CONFIG, config)
        else:
            config = DEFAULT_CONFIG.copy()

        self.robot_name = robot_name

        self._time_step = config['time_step']
        self._action_repeat = config['action_repeat']

        self._acc_control_mode = config['acc_control_mode']
        self._robot = None
        self.base_link = None
    
        self._robot = ur5e_real.create_robot_real(self._time_step,self._acc_control_mode)
        self.base_link = "base_link"

        self.workspace_dim = workspace_dim
        
        # set up goal
        self.goal = None
        self.current_goal = None

        _, robot_config = load_robot_config(robot_name=robot_name)
        self.arm_collision_controllers = robot_config['arm_collision_controllers']
        self.arm_collision_radii = []
        self.arm_interpolation_pts = []
        self.update_coll_radii()

        self.links = set()
        for controller in self.arm_collision_controllers:
            self.links.add(controller['segment'][0])
            self.links.add(controller['segment'][1])
        
        self.idmpQuery = rospy.ServiceProxy('query_dist_field', GetDistanceGradient)
        self.set_coll_radii_service = rospy.Service('/set_coll_radii', SetCollRadii, self.callback_radii)
        self.spherePub = rospy.Publisher('collisionSpheres', MarkerArray, queue_size=1)
    
    def update_coll_radii(self):
        arm_collision_radii = []
        arm_interpolation_pts = []
        for arm_controller in self.arm_collision_controllers:
            arm_collision_radii += [arm_controller['radius']] * arm_controller['interpolation_pts']
            arm_interpolation_pts.append(arm_controller['interpolation_pts'])
        self.arm_collision_radii = np.array(arm_collision_radii)
        self.arm_interpolation_pts = np.array(arm_interpolation_pts)

    def search_by_name(self, name):
        for index, dictionary in enumerate(self.arm_collision_controllers):
            if dictionary.get("name") == name:
                return index,dictionary
        return None,None

    def callback_radii(self,data):
        success = True
        for joint in data.radii:
            index,dic = self.search_by_name(joint.joint)
            if(dic!=None):
                dic['radius'] = joint.radius
            else:
                success = False
        self.update_coll_radii()
        return success

    def set_goal(self,data):
        self.goal = data
        self.current_goal = data
        
    def reset(self):
        """
        reset time and simulator
        """
        self.terminated = False
        self._env_step_counter = 0
        self._robot.reset()
        self._observation = self.get_extended_observation()
        return np.array(self._observation)

    def __del__(self):
        self._robot.shutdown()
        print("Goodbye")

    def shutdown(self):
        self._robot.shutdown()

    def stop_motion(self):
        self._robot.stop_motion()

    def get_control_points(self, link_positions, arm_collision_controllers):

        control_points = []
        for controller in arm_collision_controllers:
            end1, end2 = controller['segment']
            interpolation_points = controller['interpolation_pts']
            for k in range(interpolation_points):
                alpha = 1. * (k + 1) / interpolation_points
                point = alpha * link_positions[end2] + (1 - alpha) * link_positions[end1]
                control_points.append(point)
        control_points = np.array(control_points)
        return control_points
        
    def create_sphere_marker(self, id, position, scale, color):
        marker = Marker()
        marker.header.frame_id = self.base_link
        marker.header.stamp = rospy.Time.now()
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position = Point(*position)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        if(scale == -1):
            scale = 0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        marker.lifetime = rospy.Duration()

        return marker

    def get_extended_observation(self):
        joint_poses, joint_vels, _ = self._robot.get_observation()

        eef_position = self._robot.tcp_pose
        
        delta_x = self.current_goal[:self.workspace_dim] - eef_position[:self.workspace_dim]
        link_positions = self._robot.get_link_states(self.links)
        robotCollPts_ = self.get_control_points(link_positions, self.arm_collision_controllers)
        robotCollPts = robotCollPts_.reshape(-1)
        obsDistances_ = []
        try:
            query = self.idmpQuery(robotCollPts)
            obsDistances = np.array(query.distances) - self.arm_collision_radii
            obsGradients = np.array(query.gradients)

            c_dist = obsDistances.clip(0, 0.3)
            c_dist = (c_dist - c_dist.min()) / c_dist.ptp()
            c_dist = plt.cm.gist_rainbow(c_dist)
            marker_array = MarkerArray()
            for id, (pos, size, col) in enumerate(zip(robotCollPts_, self.arm_collision_radii, c_dist)):
                marker_array.markers.append(self.create_sphere_marker(id, pos, size, col))
            self.spherePub.publish(marker_array)
        except Exception as e:
            print("ERROR: ",e)
            obsDistances = np.ones(int(robotCollPts.shape[0]/3))
            obsGradients = np.zeros_like(robotCollPts)
                   
        coll_pts = self.arm_interpolation_pts
        radii = self.arm_collision_radii

        
        self._observation = np.concatenate(
            (np.sin(joint_poses),
            np.cos(joint_poses),
            joint_vels,
            delta_x,
            obsDistances,
            obsGradients)
        )
        return self._observation


    def step(self, action):
        action[np.isnan(action)] = 0.

        done = False
        for i in range(self._action_repeat):
            self._robot.step(action)
            
            # check if terminated
            if self._termination():
                done = True
                break
            self._env_step_counter += 1
          
        self._observation = self.get_extended_observation()

        return np.array(self._observation), done, {}

    def _termination(self):
        """
        check wether the current episode is terminated
        due to either out of steps or collision
        """

        eef_position = self._robot.tcp_pose
        distance_to_goal = np.linalg.norm(eef_position[:self.workspace_dim] - self.current_goal[:self.workspace_dim])

        if(distance_to_goal < 0.005):
            self.terminated = True
            return True
        return False
