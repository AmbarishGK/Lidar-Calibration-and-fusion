#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h> // For pcl::transformPointCloud

using namespace std::chrono_literals;

int
main ()
{
  // Define the filenames
  std::string target_filename = "first_scan.pcd";
  std::string input_filename = "second_scan.pcd";
  std::string output_filename = "second_scan_transformed.pcd";

  // Loading first scan of the environment.
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (target_filename, *target_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file %s \n", target_filename.c_str());
    return (-1);
  }
  std::cout << "Loaded " << target_cloud->size () << " data points from " << target_filename << std::endl;

  // Loading second scan of the environment from a new perspective.
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (input_filename, *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file %s \n", input_filename.c_str());
    return (-1);
  }
  std::cout << "Loaded " << input_cloud->size () << " data points from " << input_filename << std::endl;

  // Filtering input scan to roughly 10% of original size to increase speed of registration.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
  approximate_voxel_filter.setInputCloud (input_cloud);
  approximate_voxel_filter.filter (*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size ()
            << " data points from " << input_filename << std::endl;

  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon (0.01);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize (0.1);
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution (1.0);

  // Setting max number of registration iterations.
  ndt.setMaximumIterations (100);

  // Setting point cloud to be aligned.
  ndt.setInputSource (filtered_cloud);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget (target_cloud);

  // Set initial alignment estimate based on the provided transformation info.
  // Transformation info:
  float init_x = 1.9f;
  float init_y = 3.75f;
  float init_z = 0.0f;
  float init_roll = 0.0f;
  float init_pitch = 0.0f;
  float init_yaw = 0.8f;

  Eigen::AngleAxisf init_rotation_x(init_roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(init_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(init_yaw, Eigen::Vector3f::UnitZ());

  Eigen::Translation3f init_translation(init_x, init_y, init_z);

  Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align (*output_cloud, init_guess);

  std::cout << "Normal Distributions Transform has " << (ndt.hasConverged ()?"converged":"not converged")
            << ", score: " << ndt.getFitnessScore () << std::endl;

  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());

  // Saving transformed input cloud.
  pcl::io::savePCDFileASCII (output_filename, *output_cloud);
  std::cout << "Transformed point cloud saved to: " << output_filename << std::endl;

  // Initializing point cloud visualizer
  pcl::visualization::PCLVisualizer::Ptr
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);

  // Coloring and visualizing target cloud (red).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  target_color (target_cloud, 255, 0, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "target cloud");

  // Coloring and visualizing transformed input cloud (green).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  output_color (output_cloud, 0, 255, 0);
  viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud");

  // Starting visualizer
  viewer_final->addCoordinateSystem (1.0, "global");
  viewer_final->initCameraParameters ();

  // Wait until visualizer window is closed.
  while (!viewer_final->wasStopped ())
  {
    viewer_final->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }

  return (0);
}
