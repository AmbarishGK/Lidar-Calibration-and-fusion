#!/usr/bin/env python3

import rclpy
from rclpy.node import Node, ParameterDescriptor
from sensor_msgs.msg import PointCloud2
import numpy as np
import open3d as o3d
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# import sensor_msgs.point_cloud2 as pc2
import sensor_msgs_py.point_cloud2 as pc2
import os

class PCDSaverNode(Node):
    def __init__(self):
        super().__init__('pcd_saver_node')

        self.declare_parameter(
            'input_topic',
            '/velodyne_points_2',
            ParameterDescriptor(description='ROS 2 topic to subscribe for PointCloud2 messages')
        )
        self.declare_parameter(
            'target_frame_to_save',
            'velodyne_1',
            ParameterDescriptor(description='Only save point clouds with this frame_id (leave empty to save all)')
        )
        self.declare_parameter(
            'output_filename',
            'velo2.pcd',
            ParameterDescriptor(description='Base filename for the saved point cloud')
        )
        self.declare_parameter(
            'output_directory',
            '.',
            ParameterDescriptor(description='Directory to save the PCD files (default: current directory)')
        )
        self.declare_parameter(
            'save_once',
            False,
            ParameterDescriptor(description='Save only the first matching point cloud and then exit')
        )
        self.declare_parameter(
            'x_field',
            'x',
            ParameterDescriptor(description='Name of the X coordinate field in PointCloud2')
        )
        self.declare_parameter(
            'y_field',
            'y',
            ParameterDescriptor(description='Name of the Y coordinate field in PointCloud2')
        )
        self.declare_parameter(
            'z_field',
            'z',
            ParameterDescriptor(description='Name of the Z coordinate field in PointCloud2')
        )

        self.input_topic = self.get_parameter('input_topic').value
        self.target_frame_to_save = self.get_parameter('target_frame_to_save').value
        self.output_filename = self.get_parameter('output_filename').value
        self.output_directory = self.get_parameter('output_directory').value
        self.save_once = self.get_parameter('save_once').value
        self.x_field = self.get_parameter('x_field').value
        self.y_field = self.get_parameter('y_field').value
        self.z_field = self.get_parameter('z_field').value

        os.makedirs(self.output_directory, exist_ok=True)
        self.get_logger().info(f'Saving PCD files to: {self.output_directory}')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            qos_profile=qos
        )

        self.get_logger().info(f'PCD Saver Node initialized, listening to {self.input_topic}')
        if self.target_frame_to_save:
            self.get_logger().info(f'Only saving point clouds with frame_id: "{self.target_frame_to_save}"')
        else:
            self.get_logger().info('Saving all incoming point clouds regardless of frame_id')
        self.save_counter = 0
        self.has_saved = False

    def pointcloud_callback(self, cloud_msg):
        """Callback function to process incoming point cloud messages and filter by frame_id."""
        received_frame = cloud_msg.header.frame_id
        if self.target_frame_to_save and received_frame != self.target_frame_to_save:
            self.get_logger().info(f'Received point cloud in frame "{received_frame}", skipping (target: "{self.target_frame_to_save}")')
            return

        self.get_logger().info(f'Processing point cloud in frame: {received_frame}')
        o3d_cloud = self.pointcloud2_to_o3d(cloud_msg)

        if o3d_cloud is not None:
            filename = self.output_filename
            if "." not in filename:
                filename = os.path.join(self.output_directory, f"{filename}_{self.save_counter}.pcd")
            else:
                base, ext = filename.split('.')
                filename = os.path.join(self.output_directory, f"{base}_{self.save_counter}.{ext}")

            try:
                self.save_o3d_to_pcd(o3d_cloud, filename)
                self.save_counter += 1
                self.has_saved = True
                self.get_logger().info(f"Saved point cloud to: {filename}")
                if self.save_once:
                    self.get_logger().info('Saved the first matching point cloud. Shutting down.')
                    rclpy.shutdown()
            except Exception as e:
                self.get_logger().error(f"Error saving point cloud: {e}")
        else:
            self.get_logger().warn('Failed to convert PointCloud2 to Open3D format, not saving.')

    def pointcloud2_to_o3d(self, cloud_msg):
        """Convert a PointCloud2 message to an Open3D point cloud (explicit data extraction)."""
        x_field_name = self.x_field
        y_field_name = self.y_field
        z_field_name = self.z_field

        x_offset = -1
        y_offset = -1
        z_offset = -1
        data_type = None
        item_size = 0

        for field in cloud_msg.fields:
            if field.name == x_field_name:
                x_offset = field.offset
                data_type = field.datatype
                item_size = field.count * {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}[data_type]
            elif field.name == y_field_name:
                y_offset = field.offset
            elif field.name == z_field_name:
                z_offset = field.offset

        if x_offset == -1 or y_offset == -1 or z_offset == -1:
            self.get_logger().error(f'Missing required XYZ fields: x={x_field_name}, y={y_field_name}, z={z_field_name}')
            return None

        points = []
        for p in pc2.read_points(cloud_msg, field_names=[x_field_name, y_field_name, z_field_name], skip_nans=True):
            points.append([p[0], p[1], p[2]])

        if not points:
            self.get_logger().warn('No valid XYZ points received')
            return None

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points, dtype=np.float32))
        return pcd

    def save_o3d_to_pcd(self, pcd, filename="pointcloud.pcd"):
        """Save an Open3D point cloud to a PCD file in ASCII format."""
        try:
            o3d.io.write_point_cloud(filename, pcd, write_ascii=True)
        except Exception as e:
            self.get_logger().error(f"Failed to save PCD to {filename}: {e}")
            raise

def main(args=None):
    rclpy.init(args=args)
    pcd_saver_node = PCDSaverNode()
    rclpy.spin(pcd_saver_node)
    pcd_saver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()