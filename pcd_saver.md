
# Multi-LiDAR PCD Saver Node

A ROS 2 node for saving point cloud data from multiple LiDAR sensors to PCD files with configurable directory structure and naming conventions.

## Features
- 🎯 **Multi-Topic Support**: Subscribe to multiple LiDAR topics simultaneously
- 🖥️ **Frame Validation**: Ensure point clouds match specified frame IDs before saving
- 📂 **Structured Output**: Automatic directory creation and sequential file naming
- ⚙️ **Customizable Field Mapping**: Handle different point cloud field formats
- 🔢 **Save Limiting**: Configure maximum number of saves per source
- 🚀 **ROS 2 Integration**: Native support for ROS 2 Foxy/Humble

### Run your rosbag
- The rosbag should have the two topics from the lidar as pointcloud2 format and can be in the same frame or a diff frame

Change the file as you need here 
``` python
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
            './multi_calib_dataset_10_lidar/',
            ParameterDescriptor(description='Output directory for PCD files')
        )
        self.declare_parameter(
            'max_saves',
            10,
            ParameterDescriptor(description='Maximum number of clouds to save per source')
        )

```

Run the code
``` bash
python3 pcd_saver.py
```

Output
```
026536.057160311] [multi_pcd_saver_node]: Saving PCD files to: ./multi_calib_dataset_10_lidar/
[INFO] [1745026536.059114189] [multi_pcd_saver_node]: Subscribed to /velodyne_points for frame velodyne
[INFO] [1745026536.059372888] [multi_pcd_saver_node]: Subscribed to /velodyne_points_2_fixed for frame velodyne_2
[INFO] [1745026537.669302909] [multi_pcd_saver_node]: Saved ./multi_calib_dataset_10_lidar/multi_calib_lidar_1_0.pcd (1/10)
[INFO] [1745026538.350163466] [multi_pcd_saver_node]: Saved ./multi_calib_dataset_10_lidar/multi_calib_lidar_1_1.pcd (2/10)
[INFO] [1745026540.076962070] [multi_pcd_saver_node]: Saved ./multi_calib_dataset_10_lidar/multi_calib_lidar_1_2.pcd (3/10)
[INFO] [1745026540.277462500] [multi_pcd_saver_node]: Saved ./multi_calib_dataset_10_lidar/multi_calib_lidar_1_3.pcd (4/10)
[INFO] [1745026541.197359311] [multi_pcd_saver_node]: Saved ./multi_calib_dataset_10_lidar/multi_calib_lidar_1_4.pcd (5/10)
[INFO] [1745026545.367077212] [multi_pcd_saver_node]: Saved ./multi_calib_dataset_10_lidar/multi_calib_lidar_1_5.pcd (6/10)
[INFO] [1745026546.919800641] [multi_pcd_saver_node]: Saved ./multi_calib_dataset_10_lidar/multi_calib_lidar_1_6.pcd (7/10)
[INFO] [1745026546.967906818] [multi_pcd_saver_node]: Saved ./multi_calib_dataset_10_lidar/multi_calib_lidar_1_7.pcd (8/10)
[INFO] [1745026546.987775893] [multi_pcd_saver_node]: Saved ./multi_calib_dataset_10_lidar/multi_calib_lidar_1_8.pcd (9/10)
[INFO] [1745026547.007770231] [multi_pcd_saver_node]: Saved ./multi_calib_dataset_10_lidar/multi_calib_lidar_1_9.pcd (10/10)
[INFO] [1745026547.007929258] [multi_pcd_saver_node]: All sources reached 10 saves. Shutting down.
```