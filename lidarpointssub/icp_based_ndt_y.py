#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct
import open3d as o3d
import copy
import message_filters
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class NDTScanMatcher(Node):
    def __init__(self):
        super().__init__('ndt_scan_matcher')

        # Declare parameters
        self.declare_parameter('input_topic1', '/velodyne_points')
        self.declare_parameter('input_topic2', '/velodyne_points_2')
        self.declare_parameter('output_tf_parent', 'velodyne')
        self.declare_parameter('output_tf_child', 'velodyne_2')
        self.declare_parameter('voxel_size', 0.5)
        self.declare_parameter('max_iterations', 50)

        # Get parameters
        self.input_topic1 = self.get_parameter('input_topic1').value
        self.input_topic2 = self.get_parameter('input_topic2').value
        self.output_tf_parent = self.get_parameter('output_tf_parent').value
        self.output_tf_child = self.get_parameter('output_tf_child').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.max_iterations = self.get_parameter('max_iterations').value

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Publishers
        self.transformed_publisher_ = self.create_publisher(PointCloud2, "/velodyne_points_2_transformed", 10)
        self.merged_publisher_ = self.create_publisher(PointCloud2, "/merged_point_cloud", 10)

        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Message filters for synchronization
        self.sub1 = message_filters.Subscriber(self, PointCloud2, self.input_topic1, qos_profile=qos)
        self.sub2 = message_filters.Subscriber(self, PointCloud2, self.input_topic2, qos_profile=qos)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub1, self.sub2],
            queue_size=10,
            slop=10.0
        )
        self.sync.registerCallback(self.pointcloud_callback)

        self.get_logger().info('NDT Scan Matcher initialized')

    def save_o3d_to_ply(self, pcd, filename="pointcloud.ply"):
        """Save an Open3D point cloud to a PLY file."""
        try:
            o3d.io.write_point_cloud(filename, pcd)
            self.get_logger().info(f"Saved point cloud to {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to save PLY: {e}")

    def save_o3d_to_pcd(self, pcd, filename="pointcloud.pcd"):
        """Save an Open3D point cloud to a PCD file."""

        # Here I create a PCD file based on Lidar and how I saw the format of PCD in the NDT demo





        try:
            o3d.io.write_point_cloud(filename, pcd)
            self.get_logger().info(f"Saved point cloud to {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to save PCD: {e}")


    def pointcloud2_to_o3d(self, cloud_msg):
        """Convert a PointCloud2 message to an Open3D point cloud."""
        field_names = [field.name for field in cloud_msg.fields]
        offsets = {field.name: field.offset for field in cloud_msg.fields}

        if not all(field in field_names for field in ['x', 'y', 'z']):
            self.get_logger().error('Point cloud missing required XYZ fields')
            return None

        points = []
        for i in range(0, len(cloud_msg.data), cloud_msg.point_step):
            x = struct.unpack_from('f', cloud_msg.data, offset=i + offsets['x'])[0]
            y = struct.unpack_from('f', cloud_msg.data, offset=i + offsets['y'])[0]
            z = struct.unpack_from('f', cloud_msg.data, offset=i + offsets['z'])[0]
            if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(z)):
                continue
            points.append([x, y, z])

        if not points:
            self.get_logger().error('No valid points found in point cloud')
            return None

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        return pcd

    def o3d_to_pointcloud2(self, pcd, frame_id, stamp):
        """Convert an Open3D point cloud to a PointCloud2 message."""
        points = np.asarray(pcd.points)
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = points.shape[0]
        msg.is_dense = True
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.data = points.astype(np.float32).tobytes()
        return msg

    def ndt_scan_matching(self, source_pcd, target_pcd):
        """Performs NDT scan matching between two point clouds."""
        source_down = source_pcd.voxel_down_sample(self.voxel_size)
        target_down = target_pcd.voxel_down_sample(self.voxel_size)

        source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size * 2, max_nn=30))
        target_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.voxel_size * 2, max_nn=30))

        result = o3d.pipelines.registration.registration_icp(
            source_down, target_down,
            self.voxel_size * 2,
            np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=self.max_iterations,
                relative_fitness=1e-6,
                relative_rmse=1e-6
            )
        )
        return result.transformation

    def publish_transform(self, transformation, timestamp):
        """Publish the transformation as a TF transform."""
        rotation_matrix = transformation[:3, :3]
        translation = transformation[:3, 3]

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

        transform = TransformStamped()
        transform.header.stamp = timestamp
        transform.header.frame_id = self.output_tf_parent
        transform.child_frame_id = self.output_tf_child

        transform.transform.translation.x = float(translation[0])
        transform.transform.translation.y = float(translation[1])
        transform.transform.translation.z = float(translation[2])

        transform.transform.rotation.x = float(qx)
        transform.transform.rotation.y = float(qy)
        transform.transform.rotation.z = float(qz)
        transform.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(transform)

        self.get_logger().info(f"Published transform from {self.output_tf_parent} to {self.output_tf_child}")
        self.get_logger().info(f"Translation: [{translation[0]:.4f}, {translation[1]:.4f}, {translation[2]:.4f}]")
        self.get_logger().info(f"Rotation (quaternion): [{qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f}]")

    def pointcloud_callback(self, cloud_msg1, cloud_msg2):
        """Process synchronized point cloud messages."""
        # print(cloud_msg1)
        self.get_logger().info('Received synchronized point clouds')

        source_pcd = self.pointcloud2_to_o3d(cloud_msg1)
        target_pcd = self.pointcloud2_to_o3d(cloud_msg2)
        # self.save_o3d_to_ply(source_pcd, "source_cloud.ply")
        # self.save_o3d_to_ply(target_pcd, "target_cloud.ply")
        self.save_o3d_to_pcd(source_pcd, "source_cloud.pcd")
        self.save_o3d_to_pcd(target_pcd, "target_cloud.pcd")


        if source_pcd is None or target_pcd is None:
            self.get_logger().warn('Could not convert one or both point clouds')
            return

        transformation = self.ndt_scan_matching(source_pcd, target_pcd)

        target_pcd_transformed = copy.deepcopy(target_pcd).transform(transformation)

        transformed_msg = self.o3d_to_pointcloud2(target_pcd_transformed, cloud_msg1.header.frame_id, cloud_msg1.header.stamp)
        self.transformed_publisher_.publish(transformed_msg)

        merged_pcd = source_pcd + target_pcd_transformed
        merged_msg = self.o3d_to_pointcloud2(merged_pcd, cloud_msg1.header.frame_id, cloud_msg1.header.stamp)
        self.merged_publisher_.publish(merged_msg)

        self.publish_transform(transformation, cloud_msg1.header.stamp)

def main(args=None):
    rclpy.init(args=args)
    ndt_matcher = NDTScanMatcher()
    rclpy.spin(ndt_matcher)
    ndt_matcher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()