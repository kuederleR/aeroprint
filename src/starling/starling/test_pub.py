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


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
import ros_numpy
import matplotlib.pyplot as plt

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

        # Subscribe to the desired point cloud topic (e.g., /tof_pc or /voa_pc_out)
        self.point_cloud_subscriber = self.create_subscription(
            PointCloud2, '/tof_pc', self.point_cloud_callback, qos_profile)

        # self.point_cloud_subscriber2 = self.create_subscription2(
        #     PointCloud2, '/voa_pc_out', self.point_cloud_callback, qos_profile)

    def point_cloud_callback(self, point_cloud_msg):
        # Raw Data
        print("point could msg:", point_cloud_msg)
        # Data 
        print("Point cloud msg data:", point_cloud_msg.data)

        # Convert PointCloud2 message to NumPy array
        point_cloud_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(point_cloud_msg)

        # Process the received point cloud data here
        xcor = point_cloud_array[:, 0]
        ycor = point_cloud_array[:, 1]
        zcor = point_cloud_array[:, 2]
        print("Received point cloud data:")
        print("X coordinates:", xcor)
        print("Y coordinates:", ycor)
        print("Z coordinates:", zcor)

        # Plot the point cloud data
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(xcor, ycor, zcor)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()

def main(args=None) -> None:
    rclpy.init(args=args)
    starling_data_node = StarlingDataNode()
    rclpy.spin(starling_data_node)
    starling_data_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
