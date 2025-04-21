import open3d as o3d
import numpy as np
import math
import signal
import sys
import os
import csv

# Global visualizer reference
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
            "",  # "data" left blank
            source_name, target_name,
            tx, ty, tz,
            roll, pitch, yaw
        ])
    print(f"✅ Ground truth saved to {csv_path}")

def ask_and_save_screenshot(vis, source_path, target_path, save_dir=None,
                             screenshot_basename=None, screenshot_postfix=None):
    user_input = input("🖼️ Do you want to save a screenshot? (y/n): ").strip().lower()
    if user_input != 'y':
        print("📷 Skipping screenshot save.")
        return

    # Handle save directory
    if save_dir and not os.path.isdir(save_dir):
        try:
            os.makedirs(save_dir)
            print(f"📂 Created directory: {save_dir}")
        except Exception as e:
            print(f"⚠️ Could not create directory '{save_dir}', saving in current directory.")
            save_dir = "."
    elif not save_dir:
        save_dir = "."

    # Build file name
    source_name = os.path.splitext(os.path.basename(source_path))[0]
    target_name = os.path.splitext(os.path.basename(target_path))[0]
    prefix = f"{screenshot_basename}_" if screenshot_basename else ""
    postfix = f"_{screenshot_postfix}" if screenshot_postfix else ""
    filename = f"{prefix}screenshot_{source_name}_vs_{target_name}{postfix}.png"
    path = os.path.join(save_dir, filename)

    vis.capture_screen_image(path)
    print(f"✅ Screenshot saved to: {path}")

def visualize_with_manual_guess(source_path, target_path,
                                 tx=0, ty=0, tz=0,
                                 roll=0, pitch=0, yaw=0,
                                 save_dir=None,
                                 screenshot_basename=None,
                                 screenshot_postfix=None):  # NEW PARAM
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
        ask_and_save_screenshot(vis, source_path, target_path,
                                save_dir=save_dir,
                                screenshot_basename=screenshot_basename,
                                screenshot_postfix=screenshot_postfix)
        vis.destroy_window()

    ask_and_save_ground_truth(source_path, target_path, tx, ty, tz, roll, pitch, yaw)

# 🔍 Example usage

source_path="./data/multi_calib_dataset_12_lidar/multi_calib_lidar_1_0.pcd"
target_path="./data/multi_calib_dataset_12_lidar/multi_calib_lidar_2_0.pcd"
save_dir="./data/multi_calib_dataset_12_lidar/"
screenshot_basename="dataset12"
visualize_with_manual_guess(
    source_path=source_path,
    target_path=target_path,
    tx=2.0, ty=4.4, tz=0,
    roll=0, pitch=0, yaw=47,
    save_dir=save_dir,
    screenshot_basename=screenshot_basename,
    screenshot_postfix="aligned"
)

visualize_with_manual_guess(
    source_path=source_path,
    target_path=target_path,
    tx=0, ty=0, tz=0,
    roll=0, pitch=0, yaw=0,
    save_dir=save_dir,
    screenshot_basename=screenshot_basename,
    screenshot_postfix="unaligned"
)
