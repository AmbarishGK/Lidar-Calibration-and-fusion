import open3d as o3d
import os

def visualize_pcd_pairs(base_name1, base_name2, directory, num_pairs=10):
    """
    Visualize PCD pairs in separate windows with filenames in the window title.
    """
    for frame_idx in range(num_pairs):
        # Construct file paths
        pcd1_path = os.path.join(directory, f"{base_name1}_{frame_idx}.pcd")
        pcd2_path = os.path.join(directory, f"{base_name2}_{frame_idx}.pcd")

        # Load point clouds
        pcd1 = o3d.io.read_point_cloud(pcd1_path)
        pcd2 = o3d.io.read_point_cloud(pcd2_path)

        if pcd1.is_empty() or pcd2.is_empty():
            print(f"Skipping frame {frame_idx}: one or both files missing or empty.")
            continue

        # Color code
        pcd1.paint_uniform_color([1, 0, 0])  # Red
        pcd2.paint_uniform_color([0, 1, 0])  # Green

        # Set window title to filenames
        window_title = f"{os.path.basename(pcd1_path)}  |  {os.path.basename(pcd2_path)}"

        # Visualize
        o3d.visualization.draw_geometries(
            [pcd1, pcd2],
            window_name=window_title,
            width=800,
            height=600
        )

# Example usage
visualize_pcd_pairs(
    base_name1="multi_calib_lidar_1",
    base_name2="multi_calib_lidar_2",
    directory="./multi_calib_dataset_6_lidar",
    num_pairs=10
)
