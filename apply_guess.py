import open3d as o3d
import numpy as np
import math
import signal
import sys
import os
import csv

# Global visualizer reference for signal handler
vis = None

def signal_handler(sig, frame):
    print("\n🛑 Caught Ctrl+C! Closing window...")
    if vis is not None:
        vis.close()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def euler_to_rotation_matrix_degrees(roll, pitch, yaw):
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)

    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])

    Ry = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])

    Rz = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])

    return Rz @ Ry @ Rx

def apply_manual_transform(pcd, tx=0, ty=0, tz=0, roll=0, pitch=0, yaw=0):
    R = euler_to_rotation_matrix_degrees(roll, pitch, yaw)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    pcd.transform(T)
    return pcd

def ask_and_save_ground_truth(source_path, target_path, tx, ty, tz, roll, pitch, yaw, csv_path="ground_truth.csv"):
    source_name = os.path.basename(source_path)
    target_name = os.path.basename(target_path)

    user_input = input("✅ Is the initial guess correct? (y/n): ").strip().lower()
    if user_input != 'y':
        print("❌ Not saving to ground truth.")
        return

    # Extract dataset name (assuming it's the parent folder)
    dataset_name = os.path.basename(os.path.dirname(source_path))

    # Check if CSV exists and write header if not
    file_exists = os.path.isfile(csv_path)
    with open(csv_path, mode='a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        if not file_exists:
            writer.writerow([
                "data", "source", "target",
                "x (m)", "y (m)", "z (m)",
                "roll (deg)", "pitch (deg)", "yaw (deg)"
            ])
        writer.writerow([
            "",  # leave "data" column blank
            source_name, target_name,
            tx, ty, tz,
            roll, pitch, yaw
        ])
    print(f"✅ Ground truth saved to {csv_path}")

def visualize_with_manual_guess(source_path, target_path,
                                 tx=0, ty=0, tz=0,
                                 roll=0, pitch=0, yaw=0):
    global vis

    source = o3d.io.read_point_cloud(source_path)
    target = o3d.io.read_point_cloud(target_path)

    if source.is_empty() or target.is_empty():
        print("❌ One or both point clouds are empty or invalid.")
        return

    source = apply_manual_transform(source, tx, ty, tz, roll, pitch, yaw)
    source.paint_uniform_color([1, 0, 0])  # Red
    target.paint_uniform_color([0, 1, 0])  # Green

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Manual Initial Guess Alignment", width=1000, height=800)
    vis.add_geometry(source)
    vis.add_geometry(target)

    try:
        print("👀 Press Ctrl+C in terminal to exit.")
        vis.run()
    finally:
        vis.destroy_window()

    # Ask and save to CSV
    ask_and_save_ground_truth(source_path, target_path, tx, ty, tz, roll, pitch, yaw)

# Example usage
visualize_with_manual_guess(
    source_path="./data/multi_calib_dataset_14_lidar/multi_calib_lidar_1_0.pcd",  # Red
    target_path="./data/multi_calib_dataset_14_lidar/multi_calib_lidar_2_0.pcd",  # Green
    tx=2.8, ty=3.3, tz=0,
    roll=0, pitch=0, yaw=-22
)


# visualize_with_manual_guess(
#     source_path="./data/multi_calib_dataset_14_lidar/multi_calib_lidar_1_0.pcd",  # Red
#     target_path="./data/multi_calib_dataset_14_lidar/multi_calib_lidar_2_0.pcd",  # Green
#     tx=0, ty=0, tz=0,
#     roll=0, pitch=0, yaw=0
# )