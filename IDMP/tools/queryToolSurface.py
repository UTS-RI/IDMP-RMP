'''
    IDMP - Interactive Distance Field Mapping and Planning to Enable Human-Robot Collaboration
    Copyright (C) 2024 Usama Ali

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License v3 as published by
    the Free Software Foundation.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License v3 for more details.

    You should have received a copy of the GNU General Public License v3
    along with this program.  If not, see https://www.gnu.org/licenses/gpl-3.0.html.

    Authors: Usama Ali <usama.ali@thws.de>
             Adrian Mueller <adrian.mueller@thws.de>
             Lan Wu <Lan.Wu-2@uts.edu.au>
'''

import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from idmp_ros.srv import GetDistanceGradient
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import numpy as np

import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib

from scipy.spatial.transform import Rotation as R
import time
from tf import transformations

matplotlib.use("Qt5Agg")

def pose_to_numpy(msg):
	return np.dot(
        transformations.translation_matrix(np.array([msg.position.x, msg.position.y, msg.position.z])),
        transformations.quaternion_matrix(np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]))
    )

def numpy_to_point(arr):
	if arr.shape[-1] == 4:
		arr = arr[...,:-1] / arr[...,-1]

	if len(arr.shape) == 1:
		return Point(*arr)
	else:
		return np.apply_along_axis(lambda v: Point(*v), axis=-1, arr=arr)

def dtype_to_fields(dtype):
    '''Convert a numpy record datatype into a list of PointFields.
    '''

    # mappings between PointField types and numpy types
    type_mappings = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')), (PointField.INT16, np.dtype('int16')),
                    (PointField.UINT16, np.dtype('uint16')), (PointField.INT32, np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')),
                    (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]
    pftype_to_nptype = dict(type_mappings)
    nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)

    fields = []
    for field_name in dtype.names:
        np_field_type, field_offset = dtype.fields[field_name]
        pf = PointField()
        pf.name = field_name
        if np_field_type.subdtype:
            item_dtype, shape = np_field_type.subdtype
            pf.count = np.prod(shape)
            np_field_type = item_dtype
        else:
            pf.count = 1

        pf.datatype = nptype_to_pftype[np_field_type]
        pf.offset = field_offset
        fields.append(pf)
    return fields

def array_to_pointcloud2(cloud_arr, stamp=None, frame_id=None):
    '''Converts a numpy record array to a sensor_msgs.msg.PointCloud2.
    '''
    # make it 2d (even if height will be 1)
    cloud_arr = np.atleast_2d(cloud_arr)

    cloud_msg = PointCloud2()

    if stamp is not None:
        cloud_msg.header.stamp = stamp
    if frame_id is not None:
        cloud_msg.header.frame_id = frame_id
    cloud_msg.height = cloud_arr.shape[0]
    cloud_msg.width = cloud_arr.shape[1]
    cloud_msg.fields = dtype_to_fields(cloud_arr.dtype)
    cloud_msg.is_bigendian = False # assumption
    cloud_msg.point_step = cloud_arr.dtype.itemsize
    cloud_msg.row_step = cloud_msg.point_step*cloud_arr.shape[1]
    cloud_msg.is_dense = all([np.isfinite(cloud_arr[fname]).all() for fname in cloud_arr.dtype.names])
    cloud_msg.data = cloud_arr.tostring()
    return cloud_msg

def process_feedback(feedback):
    global pose
    global new_data
    min_dist = 0.005 #move 5mm to get new measurement
    new_pose = pose_to_numpy(feedback.pose)
    # if(np.linalg.norm(pose[:3,3]-new_pose[:3,3]) >= min_dist):
    pose = new_pose
    new_data = True

def numpy_to_msg(arr, intensity, frame):
    data = np.zeros(len(arr), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32),('Intensity', np.float32)])
    data['x'] = arr[:, 0]
    data['y'] = arr[:, 1]
    data['z'] = arr[:, 2]
    data['Intensity'] = intensity
    return array_to_pointcloud2(data, stamp=rospy.Time.now(), frame_id=frame)

def create_arrow(scale, start, end, idnum, color=None):
    # make a visualization marker array for the occupancy grid
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = 'base_link'
    m.header.stamp = rospy.Time.now()
    m.id = idnum
    m.type = Marker.ARROW
    m.pose.orientation.x = 0
    m.pose.orientation.y = 0
    m.pose.orientation.z = 0
    m.pose.orientation.w = 1
    m.scale.x = scale*0.5
    m.scale.y = scale
    m.scale.z = 0
    if color is None:
        m.color.r = 1
        m.color.g = 0
        m.color.b = 0
        m.color.a = 1
    else:
        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = color[3]

    m.points = [numpy_to_point(start), numpy_to_point(end)]
    return m

def wave_func(x, y, phase=0):
    return np.sin(5*(np.sqrt(x**2 + y**2) + phase/15))*0.1 * 1/np.sqrt(x**2 + y**2)

if __name__=="__main__":
    rospy.init_node("dynQuery")
    plt.ion()
    # create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer("dist_field")
    pcl_pub = rospy.Publisher("distfield", PointCloud2, queue_size=0)
    marker_pub = rospy.Publisher("grad_array", MarkerArray, queue_size=0)
    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "dist_field_center"
    int_marker.description = "Distance and Gradient to Collision for Safe Human-Robot Interactions"

    # create a grey box marker
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.45
    box_marker.scale.y = 0.45
    box_marker.scale.z = 0.45
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 0.2

    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
    box_control.markers.append( box_marker )
    int_marker.controls.append( box_control )

    pose = pose_to_numpy(box_marker.pose)
    new_data = True

    server.insert(int_marker, process_feedback)
    # server.applyChanges()

    # rospy.wait_for_service('query_dist_field')
    query = rospy.ServiceProxy('query_dist_field', GetDistanceGradient)

    gpCldMsg = rospy.wait_for_message("/gp_pcl", PointCloud2)
    queryGrid = []
    for p in point_cloud2.read_points(gpCldMsg, field_names = ("x", "y", "z"), skip_nans=True):
        queryGrid.append(p)
    queryGrid=np.array(queryGrid)
    
    # noPadMask = np.logical_and(np.round(queryGrid[:,0],4)>=-1,
    #             np.logical_and(np.round(queryGrid[:,0],4)<1,
    #             np.logical_and(np.round(queryGrid[:,1],4)>=-1,
    #             np.logical_and(np.round(queryGrid[:,1],4)<1,
    #             np.logical_and(np.round(queryGrid[:,2],4)>=-0.5,
    #                            np.round(queryGrid[:,2],4)<1)))))
    # queryGrid = queryGrid[noPadMask]

    ser_grid = np.reshape(queryGrid, -1)
    try:
        res = query(ser_grid)
    except Exception as e:
        print(e)

    dists = np.array(res.distances)
    
    grads = (np.array(res.gradients).reshape((len(res.gradients)//3,3)))
    curvature = np.array(res.curvature)
    #new_data = False
    # exit()
    # plt.cla()
    c_dist = dists
    c_dist = (c_dist - c_dist.min()) / c_dist.ptp()
    
    # c_dist = np.concatenate((c_dist, np.repeat(c_dist, 2)))
    c_dist = plt.cm.gist_rainbow(c_dist)
    mArr = MarkerArray()
    for idx, (pos, grad, col) in enumerate(zip(queryGrid, grads, c_dist)):
        mArr.markers.append(create_arrow(0.01,pos,pos+grad*0.15,idx, None))
    while(not rospy.is_shutdown()):
        pcl_pub.publish(numpy_to_msg(queryGrid, curvature, "base_link"))
        marker_pub.publish(mArr)
        rospy.Rate(1).sleep()
    # rospy.spin()

        
