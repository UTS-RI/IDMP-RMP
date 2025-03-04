 #!/usr/bin/env python

import rospy
import numpy as np
import tensorflow as tf
from rmp2_ros.srv import goal,goalResponse
from rmp2.rmpgraph import RobotRMPGraph
from rmp2.envs.robot_env_real import RobotEnvReal
from rmp2.utils.env_wrappers import URFullRMPWrapper
from scipy.spatial.transform import Rotation as r

dtype = "float32"
new_movement = False

robot_name = "ur5e_hande"

config = {
    "goal": [0.0, 0.45, 0.45, 0.0, 0.5, 0.5, -0.5, 1.0, 0.5],
    "q_init": [],
}

if robot_name == "ur5e_hande" or robot_name == "ur5e":
    env_wrapper = URFullRMPWrapper(dtype=dtype)
else:
    raise ValueError("Robot not supported")

rmp_graph = RobotRMPGraph(robot_name=robot_name, dtype=dtype, timed=True)

def msg_to_matrix(msg):
    matrix_4x4 = np.eye(4)
    matrix_4x4[:3,:3] = r.from_quat([msg.orientation.x, msg.orientation.y,msg.orientation.z, msg.orientation.w]).as_matrix()
    matrix_4x4[:3,3] = [msg.position.x, msg.position.y, msg.position.z]
    return tf.constant(matrix_4x4, dtype=dtype)

def calc_goal(msg):
    global config
    goal_eff = msg_to_matrix(msg)

    goal_p1 = tf.constant([[1.0, 0.0, 0.0, 0.5], 
                           [0.0, 1.0, 0.0, 0.0], 
                           [0.0, 0.0, 1.0, 0.0], 
                           [0.0, 0.0, 0.0, 1.0]])

    goal_p2 = tf.constant([[1.0, 0.0, 0.0, 0.0], 
                           [0.0, 1.0, 0.0, 0.5], 
                           [0.0, 0.0, 1.0, 0.0], 
                           [0.0, 0.0, 0.0, 1.0]])
    
    matrix_p1 = tf.matmul(goal_eff, goal_p1)
    matrix_p2 = tf.matmul(goal_eff, goal_p2)
    transfrom = tf.concat([goal_eff[0:3, -1],matrix_p1[0:3, -1],matrix_p2[0:3, -1]],0)
    config["goal"] = list(np.asarray(transfrom,dtype=np.float32))
    
def policy(state,config):
    goal = tf.convert_to_tensor([config["goal"]])
    ts_state = tf.convert_to_tensor([state])
    policy_input = env_wrapper.obs_to_policy_input(ts_state)
    policy_input['goal'] = goal
    ts_action = rmp_graph(**policy_input)
    action = ts_action[0].numpy()
    return action

def move_to_goal():
    global env
    env.set_goal(config["goal"])
    state = env.reset()
    action = policy(state,config)

    print("start to move")
    while not rospy.is_shutdown():
        env.set_goal(config["goal"])
        action = policy(state,config)
        state, done, _ = env.step(action)
    print("movment done")
    env.stop_motion()
    return False
    
def callback(data):
    global new_movement
    calc_goal(data.goal)
    new_movement = True
    return goalResponse(True)

if __name__ == "__main__":
    rospy.init_node('rmp2_ros')
    s = rospy.Service('/set_rmp_goal', goal, callback)

    global env
    env = RobotEnvReal(robot_name,3,config)
    config["q_init"] = env._robot.joint_poses_real

    print("start loop")
    while not rospy.is_shutdown():
        if(new_movement):
            new_movement = move_to_goal()

        rospy.sleep(0.5)
        continue

    env.shutdown()
    print("Done!")