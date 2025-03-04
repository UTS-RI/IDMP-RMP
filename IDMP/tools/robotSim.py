#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from ur_ros_driver.msg import JogControl
import time
import numpy as np

class JointVelocityPublisher:
    def __init__(self):
        self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        self.joint_velocities = [0.0] * len(self.joint_names)
        self.joint_positions = [1.9896753472735358, -1.5707963267948966, 1.658062789394613, -1.5707963267948966, -1.3526301702956052, 0.0]
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.velocity_sub = rospy.Subscriber('/jog_control', JogControl, self.velocity_callback)
        self.rate = rospy.Rate(30)  # 10 Hz

    def velocity_callback(self, msg):
        self.joint_velocities = np.array(msg.vector)
        # self.joint_velocities[[2,0]] = self.joint_velocities[[0,2]]

    def update_joint_states(self):
        if not self.joint_names:
            return

        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self.joint_names
        joint_state.velocity = self.joint_velocities
        joint_state.position = [0.0] * len(self.joint_names)
        joint_state.effort = [0.0] * len(self.joint_names)

        current_time = time.time()
        for i in range(len(self.joint_names)):
            self.joint_positions[i] += self.joint_velocities[i] * (1.0 / 10.0)  # Update position based on velocity

        joint_state.position = self.joint_positions
        self.joint_state_pub.publish(joint_state)

    def run(self):
        while not rospy.is_shutdown():
            self.update_joint_states()
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('joint_velocity_publisher')
    jvp = JointVelocityPublisher()
    jvp.run()
