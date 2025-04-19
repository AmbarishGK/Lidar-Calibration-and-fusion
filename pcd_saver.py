# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node, ParameterDescriptor
# from sensor_msgs.msg import PointCloud2
# import numpy as np
# import open3d as o3d
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# import sensor_msgs_py.point_cloud2 as pc2
# import os

# class PCDSaverNode(Node):
#     def __init__(self):
#         super().__init__('pcd_saver_node')

#         self.declare_parameter(
#             'input_topic',
#             '/velodyne_points_2',
#             ParameterDescriptor(description='ROS 2 topic to subscribe for PointCloud2 messages')
#         )
#         self.declare_parameter(
#             'target_frame_to_save',
#             'velodyne_1',
#             ParameterDescriptor(description='Only save point clouds with this frame_id (leave empty to save all)')
#         )
#         self.declare_parameter(
#             'output_filename',
#             'multi_calib_lidar_2.pcd',
#             ParameterDescriptor(description='Base filename for the saved point cloud')
#         )
#         self.declare_parameter(
#             'output_directory',
#             './multi_calib_dataset_12_lidar/',
#             ParameterDescriptor(description='Directory to save the PCD files (default: current directory)')
#         )
#         self.declare_parameter(
#             'save_once',
#             False,
#             ParameterDescriptor(description='Save only the first matching point cloud and then exit')
#         )
#         self.declare_parameter(
#             'x_field',
#             'x',
#             ParameterDescriptor(description='Name of the X coordinate field in PointCloud2')
#         )
#         self.declare_parameter(
#             'y_field',
#             'y',
#             ParameterDescriptor(description='Name of the Y coordinate field in PointCloud2')
#         )
#         self.declare_parameter(
#             'z_field',
#             'z',
#             ParameterDescriptor(description='Name of the Z coordinate field in PointCloud2')
#         )

#         self.input_topic = self.get_parameter('input_topic').value
#         self.target_frame_to_save = self.get_parameter('target_frame_to_save').value
#         self.output_filename = self.get_parameter('output_filename').value
#         self.output_directory = self.get_parameter('output_directory').value
#         self.save_once = self.get_parameter('save_once').value
#         self.x_field = self.get_parameter('x_field').value
#         self.y_field = self.get_parameter('y_field').value
#         self.z_field = self.get_parameter('z_field').value

#         os.makedirs(self.output_directory, exist_ok=True)
#         self.get_logger().info(f'Saving PCD files to: {self.output_directory}')

#         qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=10
#         )

#         self.subscription = self.create_subscription(
#             PointCloud2,
#             self.input_topic,
#             self.pointcloud_callback,
#             qos_profile=qos
#         )

#         self.get_logger().info(f'PCD Saver Node initialized, listening to {self.input_topic}')
#         if self.target_frame_to_save:
#             self.get_logger().info(f'Only saving point clouds with frame_id: "{self.target_frame_to_save}"')
#         else:
#             self.get_logger().info('Saving all incoming point clouds regardless of frame_id')
#         self.save_counter = 0
#         self.has_saved = False

#     def pointcloud_callback(self, cloud_msg):
#         """Callback function to process incoming point cloud messages and filter by frame_id."""
#         received_frame = cloud_msg.header.frame_id
#         if self.target_frame_to_save and received_frame != self.target_frame_to_save:
#             self.get_logger().info(f'Received point cloud in frame "{received_frame}", skipping (target: "{self.target_frame_to_save}")')
#             return

#         self.get_logger().info(f'Processing point cloud in frame: {received_frame}')
#         o3d_cloud = self.pointcloud2_to_o3d(cloud_msg)

#         if o3d_cloud is not None:
#             filename = self.output_filename
#             if "." not in filename:
#                 filename = os.path.join(self.output_directory, f"{filename}_{self.save_counter}.pcd")
#             else:
#                 base, ext = filename.split('.')
#                 filename = os.path.join(self.output_directory, f"{base}_{self.save_counter}.{ext}")

#             try:
#                 self.save_o3d_to_pcd(o3d_cloud, filename)
#                 self.save_counter += 1
#                 self.has_saved = True
#                 self.get_logger().info(f"Saved point cloud to: {filename}")
#                 self.get_logger().info(f"Saved {self.save_counter}/10 point clouds.")

#                 # Shutdown logic: stop after 10 clouds or if save_once is set
#                 if self.save_once or self.save_counter >= 10:
#                     reason = 'first matching cloud' if self.save_once else f'10 clouds (current count: {self.save_counter})'
#                     self.get_logger().info(f'Reached {reason}. Shutting down.')
#                     rclpy.shutdown()
#             except Exception as e:
#                 self.get_logger().error(f"Error saving point cloud: {e}")
#         else:
#             self.get_logger().warn('Failed to convert PointCloud2 to Open3D format, not saving.')

#     def pointcloud2_to_o3d(self, cloud_msg):
#         """Convert a PointCloud2 message to an Open3D point cloud (explicit data extraction)."""
#         x_field_name = self.x_field
#         y_field_name = self.y_field
#         z_field_name = self.z_field

#         x_offset = -1
#         y_offset = -1
#         z_offset = -1
#         data_type = None
#         item_size = 0

#         for field in cloud_msg.fields:
#             if field.name == x_field_name:
#                 x_offset = field.offset
#                 data_type = field.datatype
#                 item_size = field.count * {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}[data_type]
#             elif field.name == y_field_name:
#                 y_offset = field.offset
#             elif field.name == z_field_name:
#                 z_offset = field.offset

#         if x_offset == -1 or y_offset == -1 or z_offset == -1:
#             self.get_logger().error(f'Missing required XYZ fields: x={x_field_name}, y={y_field_name}, z={z_field_name}')
#             return None

#         points = []
#         for p in pc2.read_points(cloud_msg, field_names=[x_field_name, y_field_name, z_field_name], skip_nans=True):
#             points.append([p[0], p[1], p[2]])

#         if not points:
#             self.get_logger().warn('No valid XYZ points received')
#             return None

#         pcd = o3d.geometry.PointCloud()
#         pcd.points = o3d.utility.Vector3dVector(np.array(points, dtype=np.float32))
#         return pcd

#     def save_o3d_to_pcd(self, pcd, filename="pointcloud.pcd"):
#         """Save an Open3D point cloud to a PCD file in ASCII format."""
#         try:
#             o3d.io.write_point_cloud(filename, pcd, write_ascii=True)
#         except Exception as e:
#             self.get_logger().error(f"Failed to save PCD to {filename}: {e}")
#             raise

# def main(args=None):
#     rclpy.init(args=args)
#     pcd_saver_node = PCDSaverNode()
#     rclpy.spin(pcd_saver_node)
#     pcd_saver_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node, ParameterDescriptor
from sensor_msgs.msg import PointCloud2
import numpy as np
import open3d as o3d
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import sensor_msgs_py.point_cloud2 as pc2
import os
from collections import defaultdict

class MultiPCDSaverNode(Node):
    def __init__(self):
        super().__init__('multi_pcd_saver_node')

        # Declare parameters
        self.declare_parameter(
            'input_topics',
            ['/velodyne_points', '/velodyne_points_2_fixed'],
            ParameterDescriptor(description='List of ROS 2 topics to subscribe')
        )
        self.declare_parameter(
            'target_frames',
            ['velodyne', 'velodyne_2'],
            ParameterDescriptor(description='List of frame_ids to save (same order as topics)')
        )
        self.declare_parameter(
            'output_filenames',
            ['multi_calib_lidar_1.pcd', 'multi_calib_lidar_2.pcd'],
            ParameterDescriptor(description='List of base filenames (same order as topics)')
        )
        self.declare_parameter(
            'output_directory',
            './multi_calib_dataset_9_lidar/',
            ParameterDescriptor(description='Output directory for PCD files')
        )
        self.declare_parameter(
            'max_saves',
            10,
            ParameterDescriptor(description='Maximum number of clouds to save per source')
        )
        self.declare_parameter(
            'field_mappings',
            ['x:x,y:y,z:z', 'x:x,y:y,z:z'],  # Use string format for mappings
            ParameterDescriptor(description='Field mappings as "x:field,y:field,z:field" strings')
        )

        # Get and validate parameters
        self.input_topics = self.get_parameter('input_topics').value
        self.target_frames = self.get_parameter('target_frames').value
        self.output_filenames = self.get_parameter('output_filenames').value
        self.output_directory = self.get_parameter('output_directory').value
        self.max_saves = self.get_parameter('max_saves').value
        self.raw_field_mappings = self.get_parameter('field_mappings').value

        # Parse field mappings
        self.field_mappings = []
        for mapping_str in self.raw_field_mappings:
            mapping = {}
            for pair in mapping_str.split(','):
                key, value = pair.split(':')
                mapping[key.strip()] = value.strip()
            self.field_mappings.append(mapping)

        # Validate parameter lengths
        param_lists = [
            self.input_topics,
            self.target_frames,
            self.output_filenames,
            self.field_mappings
        ]
        if len({len(lst) for lst in param_lists}) != 1:
            self.get_logger().error("Parameter list lengths mismatch! Exiting.")
            rclpy.shutdown()
            return

        os.makedirs(self.output_directory, exist_ok=True)
        self.get_logger().info(f'Saving PCD files to: {self.output_directory}')

        # Initialize counters per source
        self.save_counters = defaultdict(int)
        self.active_sources = len(self.input_topics)

        # Create subscriptions with individual configurations
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        for i, topic in enumerate(self.input_topics):
            self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, idx=i: self.pointcloud_callback(msg, idx),
                qos_profile=qos
            )
            self.get_logger().info(f"Subscribed to {topic} for frame {self.target_frames[i]}")

    def pointcloud_callback(self, cloud_msg, sensor_idx):
        """Process point clouds with source-specific parameters"""
        target_frame = self.target_frames[sensor_idx]
        output_base = self.output_filenames[sensor_idx]
        fields = self.field_mappings[sensor_idx]

        # Frame validation
        received_frame = cloud_msg.header.frame_id
        if received_frame != target_frame:
            self.get_logger().debug(f'Skipping {received_frame} (want {target_frame})')
            return

        # Conversion to Open3D
        o3d_cloud = self.pointcloud2_to_o3d(cloud_msg, fields)
        if o3d_cloud is None:
            self.get_logger().warn(f'Conversion failed for {target_frame}, check field mappings')
            return

        # Filename generation
        current_count = self.save_counters[output_base]
        if current_count >= self.max_saves:
            return  # Already reached limit for this source

        base, ext = os.path.splitext(output_base)
        filename = os.path.join(
            self.output_directory,
            f"{base}_{current_count}{ext if ext else '.pcd'}"
        )

        # Save and update counters
        try:
            o3d.io.write_point_cloud(filename, o3d_cloud, write_ascii=True)
            self.save_counters[output_base] += 1
            self.get_logger().info(f"Saved {filename} ({self.save_counters[output_base]}/{self.max_saves})")
            
            # Check global exit condition
            if all(count >= self.max_saves for count in self.save_counters.values()):
                self.get_logger().info(f"All sources reached {self.max_saves} saves. Shutting down.")
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f"Save failed: {e}")

    def pointcloud2_to_o3d(self, cloud_msg, field_map):
        """Convert a PointCloud2 message to Open3D point cloud using field mapping."""
        x_field = field_map['x']
        y_field = field_map['y']
        z_field = field_map['z']

        # Log available fields for debugging
        available_fields = [f.name for f in cloud_msg.fields]
        self.get_logger().debug(f"Available fields: {available_fields}")

        # Check if required fields exist
        if not all(f in available_fields for f in [x_field, y_field, z_field]):
            self.get_logger().error(
                f"Missing required fields. "
                f"Looking for: {x_field}, {y_field}, {z_field}. "
                f"Available: {available_fields}"
            )
            return None

        # Extract points
        try:
            points = pc2.read_points(
                cloud_msg,
                field_names=[x_field, y_field, z_field],
                skip_nans=True
            )
            xyz = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float32)
        except Exception as e:
            self.get_logger().error(f"Point extraction failed: {e}")
            return None

        if xyz.size == 0:
            self.get_logger().warn("Empty point cloud received")
            return None

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        return pcd

def main(args=None):
    rclpy.init(args=args)
    node = MultiPCDSaverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
