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
             Fouad Sukkar <fouad.sukkar@uts.edu.au>
'''

import rospy
from idmp_ros.srv import GetDistanceGradient
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import numpy as np
import time
import traceback

CURVATURE_THRESH = 0.9


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

def numpy_to_msg(arr, intensity, frame):
    data = np.zeros(len(arr), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32),('intensity', np.float32)])
    data['x'] = arr[:, 0]
    data['y'] = arr[:, 1]
    data['z'] = arr[:, 2]
    data['intensity'] = intensity
    return array_to_pointcloud2(data, stamp=rospy.Time.now(), frame_id=frame)
     
def normalize_to_hemisphere(normal):
    """
    Normalize a normal vector to the hemisphere (positive z-component).
    """
    if normal[2] < 0:
        return -normal
    return normal

def normal_to_spherical(normal):
    """
    Convert a 3D normal vector to spherical coordinates.
    theta: inclination angle (0 to pi), azimuth angle (phi) is 0 to 2pi
    """
    x, y, z = normal
    r = np.linalg.norm(normal)
    theta = np.arccos(z / r)  # Inclination angle
    phi = np.arctan2(y, x)    # Azimuth angle
    return theta, phi

def quantize_normal(theta, phi, num_bins_theta, num_bins_phi):
    """
    Quantize the spherical coordinates into discrete bins.
    num_bins_theta: Number of bins for inclination angle (theta).
    num_bins_phi: Number of bins for azimuth angle (phi).
    """
    # Map theta to [0, num_bins_theta)
    theta_bin = int((theta / np.pi) * num_bins_theta)
    # Map phi to [0, num_bins_phi), handling the range of atan2
    phi_bin = int(((phi + np.pi) / (2 * np.pi)) * num_bins_phi)
    return theta_bin, phi_bin

def quantize_normals(normals, num_bins_theta, num_bins_phi):
    """
    Quantize a list of surface normals into bins.
    """
    bins = {}
    for idx, normal in enumerate(normals):
        if(np.any(np.isnan(normal))):
            continue
        # Step 1: Normalize normal to the hemisphere
        hemi_normal = normalize_to_hemisphere(normal)
        
        # Step 2: Convert the normal to spherical coordinates
        theta, phi = normal_to_spherical(hemi_normal)
        if(np.isnan(theta)):
            continue
        # Step 3: Quantize the normal into bins
        theta_bin, phi_bin = quantize_normal(theta, phi, num_bins_theta, num_bins_phi)
        
        # Step 4: Store in bins (using bin as key)
        bin_key = (theta_bin, phi_bin)
        if bin_key not in bins:
            bins[bin_key] = []
        bins[bin_key].append(idx)  # You can store the original or hemisphere-adjusted normal
        
    return bins

if __name__=="__main__":
    rospy.init_node("query_curvature_node")

    pcl_pub = rospy.Publisher("grasppoints", PointCloud2, queue_size=0)
    query = rospy.ServiceProxy('query_dist_field', GetDistanceGradient)

    while (not rospy.is_shutdown()):
        try:
            stime = time.time()
            ## Query gp
            gpCldMsg = rospy.wait_for_message("/gp_pcl", PointCloud2)
            queryGrid = []
            for p in point_cloud2.read_points(gpCldMsg, field_names = ("x", "y", "z"), skip_nans=True):
                queryGrid.append(p)
            queryGrid=np.array(queryGrid)
            centroid = np.mean(queryGrid, axis=0)
            ser_grid = np.reshape(queryGrid, -1)
            try:
                res = query(ser_grid)
            except Exception as e:
                print(e)
            
            ## Postprocessing
            dists = np.array(res.distances)
            grads = (np.array(res.gradients).reshape((len(res.gradients)//3,3)))
            curvature = np.array(res.curvature)
            curvFilt = curvature>=CURVATURE_THRESH

            filtPts = queryGrid[curvFilt]
            filtGrads = grads[curvFilt]
            filtCurv = curvature[curvFilt]

            if len(filtCurv) < 2:
                raise Exception("less than 2 candidate grasp points after curvature filtering, exiting...")
            st  = time.time()
            qGrads = quantize_normals(filtGrads, 8,8)
            print("Time taken for quantizing: ", time.time()-st)

            tmp = np.ones_like(filtCurv)
            for idx, k in enumerate(qGrads.keys()):
                    for p in qGrads[k]:
                        tmp[p] = idx
            pcl_pub.publish(numpy_to_msg(filtPts, tmp, "base_link"))
            # pcl_pub.publish(numpy_to_msg(filtPts, filtCurv, "base_link"))
        except KeyboardInterrupt:
            sys.exit()

        except Exception as ex:
            print(traceback.format_exc())
