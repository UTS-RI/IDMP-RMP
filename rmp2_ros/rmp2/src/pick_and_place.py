#!/usr/bin/env python

import rospy
import time
import tf2_ros
from geometry_msgs.msg import Pose, TransformStamped
from rmp2_ros.srv import goal,goalResponse
from ur_ros_driver.srv import SetGripper, SetGripperRequest
from rmp2_ros.srv import SetCollRadii, SetCollRadiiRequest
from rmp2_ros.msg import Radii

def pose_distance(pose, transform_stamped):
    # Calculate the Euclidean distance between two poses
    dx = pose.position.x - transform_stamped.transform.translation.x
    dy = pose.position.y - transform_stamped.transform.translation.y
    dz = pose.position.z - transform_stamped.transform.translation.z
    distance = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
    return distance

def compare_pose_with_transform(pose):
   
    global tf_buffer
    # Wait for the transform between "base_link" and "tcp_link"
    while not rospy.is_shutdown():
        try:
            transform_stamped = tf_buffer.lookup_transform("base_link", "tcp_link", rospy.Time(), rospy.Duration(1.0))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to lookup transform between 'base_link' and 'tcp_link'. Retrying...")

    # Calculate the distance between the original pose and the transformed pose
    distance = pose_distance(pose, transform_stamped)
    print(distance)
    return distance

def array_to_pose(array):
    if len(array) != 7:
        print("Error: Input array must have 7 elements")
        return None

    pose = Pose()
    pose.position.x = array[0]
    pose.position.y = array[1]
    pose.position.z = array[2]
    pose.orientation.x = array[3]
    pose.orientation.y = array[4]
    pose.orientation.z = array[5]
    pose.orientation.w = array[6]

    return pose

def send_pose_to_rmp_goal(pose):
    rospy.wait_for_service('set_rmp_goal')
    try:
        set_rmp_goal = rospy.ServiceProxy('set_rmp_goal', goal)
        resp = set_rmp_goal(pose)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed:", e)

def send_gripper_request(position, speed = 100, force = 100):
    rospy.wait_for_service('/ur_hardware_interface/robotiq/set_gripper')
    try:
        set_gripper = rospy.ServiceProxy('/ur_hardware_interface/robotiq/set_gripper', SetGripper)
        req = SetGripperRequest()
        req.position_unit = 0
        req.position = position
        req.speed = speed
        req.force = force
        req.asynchronous = 0
        resp = set_gripper(req)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed:", e)

def move_to_pose(pose):
    resp_rmp = send_pose_to_rmp_goal(pose)
    while(compare_pose_with_transform(pose)>0.02):
        time.sleep(0.2)

def add_radius(name,radius,srv):
    msg = Radii()
    msg.joint = name
    msg.radius = radius
    srv.radii.append(msg)

def send_srv(srv):
    rospy.wait_for_service('/set_coll_radii')
    try:
        print(srv)
        set_rmp_goal = rospy.ServiceProxy('/set_coll_radii', SetCollRadii)
        resp = set_rmp_goal(srv)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed:", e)

if __name__ == '__main__':
    rospy.init_node('pick_and_place_example')

     # Initialize TF2 listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Example array representing pose
    pose_1A = array_to_pose([-0.197, 0.523, 0.3, -0.707, 0.707, 0.0, 0.0])
    pose_1CA = array_to_pose([-0.197, 0.523, 0.1, -0.707, 0.707, 0.0, 0.0])
    pose_1 = array_to_pose([-0.197, 0.523, 0.025, -0.707, 0.707, 0.0, 0.0])
    pose_2A = array_to_pose([0.085, 0.608, 0.3, 0.0, 1.0, 0.0, 0.0])
    pose_2CA = array_to_pose([0.085, 0.608, 0.08, 0.0, 1.0, 0.0, 0.0])
    pose_2 = array_to_pose([0.085, 0.608, 0.015, 0.0, 1.0, 0.0, 0.0])

    pose_4A = array_to_pose([0.0, 0.53, 0.25, 0.0, 1.0, 0.0, 0.0])
    pose_5A = array_to_pose([-0.53, 0.03, 0.25, 0.0, 1.0, 0.0, 0.0])

    srv_1 = SetCollRadiiRequest()
    add_radius("tcp_joint",-1,srv_1)
    add_radius("hande_joint",-1,srv_1)
    add_radius("hande_base_joint",-1,srv_1)
    add_radius("wrist_3_joint",-1,srv_1)
    add_radius("wrist_2_joint",-1,srv_1)
    add_radius("shoulder_lift_joint",-1,srv_1)
    add_radius("elbow_joint",-1,srv_1)

    srv_2 = SetCollRadiiRequest()
    add_radius("tcp_joint",.05,srv_2)
    add_radius("hande_joint",.05,srv_2)
    add_radius("hande_base_joint",.07,srv_2)
    add_radius("wrist_3_joint",.1,srv_2)
    add_radius("wrist_2_joint",.1,srv_2)
    add_radius("shoulder_lift_joint",.12,srv_2)
    add_radius("elbow_joint",.12,srv_2)

    srv_3 = SetCollRadiiRequest()
    add_radius("hande_joint",.15,srv_3)
    add_radius("hande_base_joint",.15,srv_3)
    add_radius("wrist_3_joint",.15,srv_3)
    add_radius("wrist_2_joint",.15,srv_3)
    add_radius("shoulder_lift_joint",.15,srv_3)
    add_radius("elbow_joint",.15,srv_3)

    while(not rospy.is_shutdown()):
        ## Movement for Pick and Place
        # send_srv(srv_2)
        # move_to_pose(pose_1A)
        # send_srv(srv_1)
        # move_to_pose(pose_1CA)
        # rospy.sleep(1)
        # move_to_pose(pose_1)
        # rospy.sleep(1)
        # resp_gripper = send_gripper_request(100)
        # move_to_pose(pose_1A)
        # send_srv(srv_2)
        # move_to_pose(pose_2A)
        # send_srv(srv_1)
        # move_to_pose(pose_2CA)
        # rospy.sleep(1)
        # move_to_pose(pose_2)
        # rospy.sleep(3)
        # resp_gripper = send_gripper_request(0)
        # move_to_pose(pose_2A)

        ## Movement for Pick and Place with coll avoidance
        # move_to_pose(pose_5A)
        # send_srv(srv_2)
        # move_to_pose(pose_2A)
        # send_srv(srv_1)
        # move_to_pose(pose_2CA)
        # rospy.sleep(1)
        # move_to_pose(pose_2)
        # rospy.sleep(1)
        # resp_gripper = send_gripper_request(0)
        # move_to_pose(pose_2A)
        # send_srv(srv_2)
        # move_to_pose(pose_5A)
        # move_to_pose(pose_2A)
        # send_srv(srv_1)
        # move_to_pose(pose_2CA)
        # rospy.sleep(1)
        # move_to_pose(pose_2)
        # rospy.sleep(3)
        # resp_gripper = send_gripper_request(100)
        # move_to_pose(pose_2A)
        # send_srv(srv_2)
        # move_to_pose(pose_5A)

        ## Movement for reaktive control Showcase
        send_srv(srv_2)
        move_to_pose(pose_4A)
        move_to_pose(pose_5A)
