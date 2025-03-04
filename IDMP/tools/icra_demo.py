import rospy
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from idmp_ros.srv import GetDistanceGradient


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
    data = np.zeros(len(arr), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32),('Intensity', np.float32)])
    data['x'] = arr[:, 0]
    data['y'] = arr[:, 1]
    data['z'] = arr[:, 2]
    data['Intensity'] = intensity
    return array_to_pointcloud2(data, stamp=rospy.Time.now(), frame_id=frame)

class Ball:
    def __init__(self, pos, goal):
        
        self.position = np.array(pos)
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.goal = np.array(goal)

    def update(self, distance, gradient):
        # Attract to goal
        attraction = self.goal - self.position
        attraction_norm = np.linalg.norm(attraction)
        if attraction_norm > 0.01:
            attraction = attraction / attraction_norm
        else:
            attraction*=0

        if(distance < 0.4):
            gradient[2] = 0
        else:
            gradient = np.array([0,0,0])
        repulsion = gradient / max(distance, 1e-5)
        
        # Update velocity and position
        damping = 0.6
        self.velocity += 0.8 * (attraction + repulsion)  # Adjust coefficients as needed
        self.velocity *= damping
        self.position += self.velocity * 0.1


class BallSimulation:
    def __init__(self):
        rospy.init_node('ball_simulation')
        xg = np.linspace(-1, 1, 10)
        yg = np.linspace(-1, 1, 10)
        zg = 0.1
        grid = np.array(np.meshgrid(xg,yg,zg)).reshape(3,-1).T
        self.balls = []
        for p in grid:
            self.balls.append(Ball(p,p))
        
        self.marker_pub = rospy.Publisher('/ball_marker', Marker, queue_size=10)
        self.pcl_pub = rospy.Publisher("balls", PointCloud2, queue_size=0)
        self.distance_service = rospy.ServiceProxy('query_dist_field', GetDistanceGradient)

        rospy.Timer(rospy.Duration(0.1), self.update)

    def update(self, event):
        pts = []
        for b in self.balls:
            pts.append(b.position)
        pts = np.array(pts).reshape(-1)
        res = self.distance_service(pts)
        distances = np.array(res.distances)
        gradients = np.array(res.gradients).reshape(len(self.balls),3)
        for i in range(len(self.balls)):
            self.balls[i].update(distances[i],gradients[i])
        # Publish marker
        self.publish()

    def publish(self):
        pts = []
        for b in self.balls:
            pts.append(b.position)
        pts = np.array(pts)
        self.pcl_pub.publish(numpy_to_msg(pts, [i for i in range(len(self.balls))], "base_link"))
        

if __name__ == "__main__":
    BallSimulation()
    rospy.spin()
