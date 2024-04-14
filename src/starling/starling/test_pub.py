# import rclpy
# import math
# import time
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# from px4_msgs.msg import VehicleOdometry

# class StarlingDataNode(Node):
#     """Node for reading some starling data as a test."""
#     def __init__(self) -> None:
#         super().__init__('starling_data_node')

#         self.get_logger().info("Starling data test node alive!")

#         # Configure QoS profile for publishing and subscribing
#         qos_profile = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             durability=DurabilityPolicy.TRANSIENT_LOCAL,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=1
#         )

#         self.vehicle_status_subscriber = self.create_subscription( #creating subscriber, create subscription with Vodometry, call odo callback
#             VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odo_callback, qos_profile) #fmu/out/vehicle odometry -> topic

#     def vehicle_odo_callback(self, vehicle_odometry):
#         """Callback function for vehicle_status topic subscriber."""
#         print(vehicle_odometry.position)

# def main(args=None) -> None:
#     rclpy.init(args=args)
#     starling_data_node = StarlingDataNode()
#     rclpy.spin(starling_data_node)
#     starling_data_node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


    #Steps for meshing and visualization
    
    # Merge multiple point clouds(e.g., registration, alignment)
    # Convert point cloud to 3D mesh (surface reconstruction) Using pointcloud2 data
    # Poisson surface reconstruction & ball pivoting algorithm look promising
        
    # Visualize the 3D mesh or print the object
    # This step depends on whether you want to visualize the object or physically print it
    # For visualization, you can use Open3D
    # For 3D printing, you can export the mesh to a format suitable for 3D printing software

    #Merging point clouds with registration
    # 1. Acquire Point Clouds: Capture multiple point cloud scans of the object from different viewpoints using a camera or a 3D sensor. 
    # You can move the camera or sensor around the object to capture different perspectives.
    # 2. Registration: Align the acquired point clouds into a common coordinate system. 
    # This step involves estimating the transformation (translation and rotation) between pairs of overlapping point clouds.
    # 3. Merge Point Clouds: Once the point clouds are registered, 
    # merge them into a single global point cloud representing the complete 3D model of the object.

    #Meshing point cloud w/o ros_numpy 


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
import numpy as np

class StarlingDataNode(Node):
    def __init__(self) -> None:
        super().__init__('starling_data_node')
        self.get_logger().info("Starling data test node alive!")

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to the desired point cloud topic (/tof_pc or /voa_pc_out)
        self.point_cloud_subscriber = self.create_subscription(
            PointCloud2, '/tof_pc', self.point_cloud_callback, qos_profile)

        # Array for saving point cloud data
        #self.point_cloud_data = []


        # Other method of saving point cloud 
        self.all_point_cloud_data = np.empty((0, 3))



    def point_cloud_callback(self, point_cloud_msg):
        # MSG Data
        self.get_logger().info("Point cloud msg: %s", str(point_cloud_msg))
        
        # Convert PointCloud2 message to numpy array
        points = read_points(point_cloud_msg)
        point_cloud_np = np.array(list(points))

        # Store the numpy array in the list
        #self.point_cloud_data.append(point_cloud_np)

        # Other method
        self.all_point_cloud_data = np.vstack((self.all_point_cloud_data, point_cloud_np))

    
    def save_point_cloud_data(self, file_path):
        # Save the accumulated point cloud data to a file
        np.save(file_path, self.point_cloud_data)

        # Try and mesh here
        mesh_numpy_data(self, self.point_cloud_data)

        # clear point cloud data
        self.all_point_cloud_data = np.empty((0, 3))

    
    def mesh_numpy_data(self, numpy_file):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        #Poisson Meshing
        poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8, width=0, scale=1.1, linear_fit=False)[0]
        #Cropping
        bbox = pcd.get_axis_aligned_bounding_box()
        p_mesh_crop = poisson_mesh.crop(bbox)

        #visualization
        o3d.visualization.draw_geometries_with_editing([p_mesh_crop])



import sys
from collections import namedtuple
import ctypes
import math
import struct
from sensor_msgs.msg import PointCloud2, PointField

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.

    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step

def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt


def main(args=None) -> None:
    rclpy.init(args=args)
    starling_data_node = StarlingDataNode()

    try:
        rclpy.spin(starling_data_node)
    except KeyboardInterrupt:
        # Save the data and shutdown upon keyboard interrupt
        starling_data_node.save_point_cloud_data("point_cloud_data.npy")
        starling_data_node.destroy_node()
        rclpy.shutdown()

    # rclpy.spin(starling_data_node)
    # starling_data_node.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
