 #!/usr/bin/env python

import rospy
import tf
from rmp2_ros.srv import goal,goalResponse

from interactive_markers.interactive_marker_server import *
import tf.transformations
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarkerControl
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

import numpy as np

def process_feedback(feedback):
    global pose
    global new_data
    pose = feedback.pose 
    new_data = True

def makeBox( msg ):
    marker = Marker()
    marker.type = Marker.ARROW
    marker.scale.x = msg.scale * 0.4
    marker.scale.y = msg.scale * 0.08
    marker.scale.z = msg.scale * 0.08
    marker.pose.orientation = Quaternion(0,-0.707,0,0.707)
    marker.color.b = 1
    marker.color.a = 1.0

    return marker

def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True

    marker = Marker()
    marker.type = Marker.ARROW
    marker.scale.x = msg.scale * 0.4
    marker.scale.y = msg.scale * 0.08
    marker.scale.z = msg.scale * 0.08
    marker.pose.orientation = Quaternion(0,-0.707,0,0.707)
    marker.color.b = 1
    marker.color.a = 1.0
    control.markers.append(marker)

    marker = Marker()
    marker.type = Marker.ARROW
    marker.scale.x = msg.scale * 0.4
    marker.scale.y = msg.scale * 0.08
    marker.scale.z = msg.scale * 0.08
    marker.pose.orientation = Quaternion(0,0,0,1)
    marker.color.r = 1
    marker.color.a = 1.0
    control.markers.append(marker)

    marker = Marker()
    marker.type = Marker.ARROW
    marker.scale.x = msg.scale * 0.4
    marker.scale.y = msg.scale * 0.08
    marker.scale.z = msg.scale * 0.08
    marker.pose.orientation = Quaternion(0,0,0.707,0.707)
    marker.color.g = 1
    marker.color.a = 1.0
    control.markers.append(marker)

    msg.controls.append( control )
    return control

#####################################################################
# Marker Creation

def make6DofMarker( fixed, interaction_mode, position, frame_id, show_6dof = True):
    global pose

    int_marker = InteractiveMarker()
    int_marker.header.frame_id = frame_id
    int_marker.pose.position.x = position[0]
    int_marker.pose.position.y = position[1]
    int_marker.pose.position.z = position[2]
    int_marker.pose.orientation = Quaternion(0,1,0,0)
    int_marker.scale = 0.2

    int_marker.name = "simple_6dof"
    int_marker.description = "Simple 6-DOF Control"

    # insert a box
    makeBoxControl(int_marker)

    int_marker.controls[0].interaction_mode = interaction_mode

    if show_6dof: 
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    server = InteractiveMarkerServer("Goal_Position")
    server.insert(int_marker, process_feedback)
    server.applyChanges()

    
    pose = int_marker.pose
    new_data = True

if __name__ == "__main__":
    new_data = False
    rospy.init_node('set_goal')
    rospy.wait_for_service('/set_rmp_goal')
    goal_service = rospy.ServiceProxy('/set_rmp_goal', goal)
    make6DofMarker(False,InteractiveMarkerControl.MOVE_ROTATE_3D,[0,0.4,0.25,0,0,0],"base_link",True)

    while not rospy.is_shutdown():
        print(pose)
        if(new_data):
            try:
                resp = goal_service(pose)
                if(not resp.success):
                    print("Some thing went wrong with the StartJog Service")
            except Exception as e:
                print(e)
        rospy.Rate(30).sleep()