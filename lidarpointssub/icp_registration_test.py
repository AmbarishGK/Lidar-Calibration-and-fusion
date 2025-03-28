#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct
import open3d as o3d
import copy
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import message_filters
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math
import time

class LidarCalibrationMerger(Node):
    def __init__(self):
        super().__init__('lidar_calibration_merger')
        
        # Declare parameters
        self.declare_parameter('input_topic1', '/velodyne_points')
        self.declare_parameter('input_topic2', '/velodyne_points_2')
        self.declare_parameter('output_topic', '/merged_cloud')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('calibration_mode', True)

        # Configure default parameters
        self.declare_parameter('max_correspondence_distance', 0.05)  # 5cm for ICP
        self.declare_parameter('max_iterations', 50)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('tf_parent', 'lidar1')
        self.declare_parameter('tf_child', 'lidar2')
        
        # Get parameters
        self.input_topic1 = self.get_parameter('input_topic1').value
        self.input_topic2 = self.get_parameter('input_topic2').value
        self.output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.calibration_mode = self.get_parameter('calibration_mode').value
        self.max_correspondence_distance = self.get_parameter('max_correspondence_distance').value
        self.max_iterations = self.get_parameter('max_iterations').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.tf_parent = self.get_parameter('tf_parent').value
        self.tf_child = self.get_parameter('tf_child').value
        
        # Set up QoS profile for subscribers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publisher for merged point cloud
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)
        
        # Create TF broadcaster if needed
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Set up message filters for synchronization
        self.sub1 = message_filters.Subscriber(self, PointCloud2, self.input_topic1, qos_profile=qos)
        self.sub2 = message_filters.Subscriber(self, PointCloud2, self.input_topic2, qos_profile=qos)
        
        # Use approximate time synchronization
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub1, self.sub2], 
            queue_size=10,
            slop=0.1  # 100ms
        )
        self.sync.registerCallback(self.pointcloud_callback)
        
        # Initialize calibration variables
        self.calibration_done = False
        self.transformation_matrix = np.identity(4)  # Initial transformation (identity)
        self.calibration_count = 0
        self.max_calibration_count = 10  # Number of point clouds to use for calibration
        self.calibration_transformations = []
        
        self.get_logger().info('LiDAR Calibration and Merger initialized')
        self.get_logger().info(f'Listening on topics: {self.input_topic1} and {self.input_topic2}')
        self.get_logger().info(f'Publishing merged cloud to: {self.output_topic}')
        self.get_logger().info(f'Calibration mode: {"ON" if self.calibration_mode else "OFF"}')

    def pointcloud2_to_o3d(self, cloud_msg):
        """Convert a PointCloud2 message to an Open3D point cloud."""
        # Get field offsets
        field_names = [field.name for field in cloud_msg.fields]
        offsets = {field.name: field.offset for field in cloud_msg.fields}
        
        # Check if required fields exist
        required_fields = ['x', 'y', 'z']
        if not all(field in field_names for field in required_fields):
            self.get_logger().error('Point cloud missing required XYZ fields')
            return None
        
        # Extract points
        points = []
        
        # Process each point
        for i in range(0, len(cloud_msg.data), cloud_msg.point_step):
            x = struct.unpack_from('f', cloud_msg.data, offset=i + offsets['x'])[0]
            y = struct.unpack_from('f', cloud_msg.data, offset=i + offsets['y'])[0]
            z = struct.unpack_from('f', cloud_msg.data, offset=i + offsets['z'])[0]
            
            # Skip invalid points (NaN or infinite values)
            if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(z)):
                continue
            
            points.append([x, y, z])
            print(points)
        
        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        
        # Estimate normals (needed for better registration)
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        
        return pcd, cloud_msg.header.frame_id

    def o3d_to_pointcloud2(self, pcd, frame_id, stamp):
        """Convert an Open3D point cloud to a PointCloud2 message."""
        points = np.asarray(pcd.points)
        
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        
        # Define fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Create point cloud data
        msg.height = 1
        msg.width = points.shape[0]
        msg.point_step = 12  # 3 * sizeof(float32)
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.is_bigendian = False
        
        # Convert points to bytes
        msg.data = points.astype(np.float32).tobytes()
        
        return msg

    def calibrate_point_clouds(self, source_pcd, target_pcd):
        """Calibrate two point clouds using ICP."""
        # Downsample point clouds for faster registration
        source_down = source_pcd.voxel_down_sample(voxel_size=2)
        target_down = target_pcd.voxel_down_sample(voxel_size=2)
        
        # Apply point-to-plane ICP
        # found from Stack Overflow
        # https://stackoverflow.com/questions/70609895/how-to-use-open3d-icp-registration-with-pointclouds
        self.get_logger().info(f"Running ICP with max_correspondence_distance={self.max_correspondence_distance}, max_iterations={self.max_iterations}")
        result = o3d.pipelines.registration.registration_icp(
            source_down, target_down, 
            self.max_correspondence_distance, 
            np.identity(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=self.max_iterations)
        )
        
        # Log calibration results
        self.get_logger().info(f"ICP fitness: {result.fitness}")
        self.get_logger().info(f"ICP RMSE: {result.inlier_rmse}")
        
        return result.transformation

    def publish_transform(self, transformation_matrix, timestamp):

        # This one was AI Generated so I ll have to make the relevant changes here to make it work
        """Publish the transformation as a TF transform."""
        if not self.publish_tf:
            return
        
        # Extract rotation and translation from transformation matrix
        rotation_matrix = transformation_matrix[:3, :3]
        translation = transformation_matrix[:3, 3]
        
        # Convert rotation matrix to quaternion
        # This is a simplified version - you might want to use a library like scipy or tf_transformations
        trace = rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) * s
            qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) * s
            qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) * s
        elif rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
            s = 2.0 * np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2])
            qw = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
            qx = 0.25 * s
            qy = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
            qz = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
        elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
            s = 2.0 * np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2])
            qw = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
            qx = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
            qy = 0.25 * s
            qz = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1])
            qw = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
            qx = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
            qy = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
            qz = 0.25 * s
        
        # Create and publish transform
        transform = TransformStamped()
        transform.header.stamp = timestamp
        transform.header.frame_id = self.tf_parent
        transform.child_frame_id = self.tf_child
        
        # Set translation
        transform.transform.translation.x = float(translation[0])
        transform.transform.translation.y = float(translation[1])
        transform.transform.translation.z = float(translation[2])
        
        # Set rotation
        transform.transform.rotation.x = float(qx)
        transform.transform.rotation.y = float(qy)
        transform.transform.rotation.z = float(qz)
        transform.transform.rotation.w = float(qw)
        
        # Publish the transform
        self.tf_broadcaster.sendTransform(transform)
        
        # Log the transformation
        self.get_logger().info(f"Published transform from {self.tf_parent} to {self.tf_child}")
        self.get_logger().info(f"Translation: [{translation[0]:.4f}, {translation[1]:.4f}, {translation[2]:.4f}]")
        self.get_logger().info(f"Rotation (quaternion): [{qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f}]")

    def pointcloud_callback(self, cloud_msg1, cloud_msg2):
        """Process synchronized point cloud messages."""
        self.get_logger().debug('Received synchronized point clouds')
        
        # Convert ROS messages to Open3D point clouds
        pcd1, frame_id1 = self.pointcloud2_to_o3d(cloud_msg1)
        pcd2, frame_id2 = self.pointcloud2_to_o3d(cloud_msg2)
        
        if pcd1 is None or pcd2 is None:
            self.get_logger().warn('Could not convert one or both point clouds')
            return
        
        # If in calibration mode and calibration not done yet
        if self.calibration_mode and not self.calibration_done:
            self.get_logger().info(f"Calibration step {self.calibration_count + 1}/{self.max_calibration_count}")
            
            # Perform calibration
            transformation = self.calibrate_point_clouds(pcd2, pcd1)  # Transform pcd2 to align with pcd1
            self.calibration_transformations.append(transformation)
            self.calibration_count += 1
            
            # Check if we have enough calibration samples
            if self.calibration_count >= self.max_calibration_count:
                # Average the transformations
                self.transformation_matrix = np.mean(self.calibration_transformations, axis=0)
                self.calibration_done = True
                self.get_logger().info("Calibration completed!")
                self.get_logger().info(f"Final transformation matrix:\n{self.transformation_matrix}")
                
                # Publish the transformation
                self.publish_transform(self.transformation_matrix, self.get_clock().now().to_msg())
        
        # Apply the transformation to the second point cloud
        if self.calibration_done or not self.calibration_mode:
            # If not in calibration mode and no calibration done, use identity transform
            if not self.calibration_mode and not self.calibration_done:
                self.transformation_matrix = np.identity(4)
            
            # Transform pcd2 to align with pcd1
            pcd2_transformed = copy.deepcopy(pcd2)
            pcd2_transformed.transform(self.transformation_matrix)
            
            # Merge the point clouds
            merged_pcd = o3d.geometry.PointCloud()
            merged_pcd.points = o3d.utility.Vector3dVector(
                np.vstack((np.asarray(pcd1.points), np.asarray(pcd2_transformed.points)))
            )
            
            # Convert back to ROS message
            merged_msg = self.o3d_to_pointcloud2(
                merged_pcd, 
                self.target_frame, 
                self.get_clock().now().to_msg()
            )
            
            # Publish merged point cloud
            self.publisher.publish(merged_msg)
            
            # Publish the transform periodically
            if self.publish_tf and self.calibration_done:
                self.publish_transform(self.transformation_matrix, self.get_clock().now().to_msg())
            
            self.get_logger().debug(f'Published merged cloud with {merged_msg.width} points')

def main(args=None):
    rclpy.init(args=args)
    node = LidarCalibrationMerger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
