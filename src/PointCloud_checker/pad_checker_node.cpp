// planar planarsegmentation.cpp
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/voxel_grid.h>

#include <robotic_arm_inspector/planes_msg.h>
#include <cmath>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pub;

struct plane_struct{
	pcl_msgs::ModelCoefficients coefficients;
	pcl::PointIndices inliers;
};

void publish_pc(pcl::PointCloud<pcl::PointXYZ> cloud){
  // convert to pointCloud2
  pcl::PCLPointCloud2 point_cloud2;
  pcl::toPCLPointCloud2(cloud, point_cloud2);
  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(point_cloud2, output);
  // Publish the data
  pub.publish (output);
}

plane_struct get_plane_struct(pcl::PointCloud<pcl::PointXYZ> cloud){
	pcl::ModelCoefficients coefficients;
	pcl::PointIndices inliers;
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.1);

  publish_pc(cloud);
	seg.setInputCloud (cloud.makeShared ());
	seg.segment (inliers, coefficients); 

  plane_struct ret;
  int i = 1;
  if (inliers.indices.size () == 0) {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return ret;
  } else {
    while (inliers.indices.size () > 0){
      ros::Duration(1).sleep();
      // Extract inliers
      pcl::PointIndices::Ptr inliersptr (new pcl::PointIndices (inliers));
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(cloud.makeShared());
      extract.setIndices(inliersptr);
      extract.setNegative(true);
      pcl::PointCloud<pcl::PointXYZ> cloudF;
      extract.filter(cloudF);
      cloud.swap(cloudF);
      publish_pc(cloud);

      // Publish the model coefficients
      pcl_msgs::ModelCoefficients ros_coefficients;
      pcl_conversions::fromPCL(coefficients, ros_coefficients);
      ROS_INFO("%d: [%f,%f,%f,%f]", i++, ros_coefficients.values[0],ros_coefficients.values[1],ros_coefficients.values[2],ros_coefficients.values[3]);

      ret = {ros_coefficients, inliers};
      seg.setInputCloud (cloud.makeShared ());
      seg.segment (inliers, coefficients);
    }
  }
  return ret;
}

float compute_point_plane_distance(pcl_msgs::ModelCoefficients coefficients, float point[3]){
  float A = coefficients.values[0];
  float B = coefficients.values[1];
  float C = coefficients.values[2];
  float D = coefficients.values[3];
  float x = point[0];
  float y = point[1];
  float z = point[2];

  // ROS_INFO("plane [%f,%f,%f,%f]", A, B, C, D);
  // ROS_INFO("point [%f,%f,%f]", x, y, z);

  float num = fabs(A*x + B*y + C*z - D);
  float den = sqrt(pow(A,2) + pow(B,2) + pow(C,2));
  float dist = num/den;

  // ROS_INFO("fabs [%f,%f,%f,%f]", fabs(A), fabs(B), fabs(C), fabs(D));
  // ROS_INFO("num: %f", num);
  // ROS_INFO("den: %f", den);

  // ROS_INFO("Distance: %f", dist);
  return dist;
}

float compute_average_distance(pcl::PointCloud<pcl::PointXYZ> pc1, pcl::PointCloud<pcl::PointXYZ> pc2){
	plane_struct plane1 = get_plane_struct(pc1);
	plane_struct plane2 = get_plane_struct(pc2);
  // ROS_INFO_STREAM("plane1: " << plane1.inliers);
  // ROS_INFO_STREAM("plane2: " << plane2.inliers);
  // adesso ho gli indici dei punti appartenenti al piano
  // devo spostare sia il punto che il piano in base alla posizione del kinect
  float point[3] = {1.0, 1.0, 1.0};
  float dist_sum = 0.0;
  size_t plane1_size = plane1.inliers.indices.size();
  for (size_t i = 0; i < plane1_size; ++i){
    point[0] = pc1.points[plane1.inliers.indices[i]].x;
    point[1] = pc1.points[plane1.inliers.indices[i]].y;
    point[2] = pc1.points[plane1.inliers.indices[i]].z;
    // ROS_INFO("point [%f,%f,%f]", point[0], point[1], point[2]);
    // faccio la distanza punto piano tra un punto di un piano e l'altro piano
    float dist = compute_point_plane_distance(plane2.coefficients, point);
    // per fare le cose meno ignorri dovrei fare la distanza media tra i punti appartenenti al piano e l'altro piano
    // ROS_INFO("Point: [%f, %f, %f] Distance: %f", point[0], point[1], point[2], dist); 
    dist_sum += dist;
  }
  float avg_distance = dist_sum/plane1_size;
  ROS_INFO("Average Distance: %f", avg_distance); 
  return avg_distance;
}

pcl::PointCloud<pcl::PointXYZ> filter_cloud(pcl::PointCloud<pcl::PointXYZ> cloud_origin){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  *cloud = cloud_origin;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud (cloud);
  voxel_grid.setLeafSize (0.1, 0.1, 0.1);
  voxel_grid.filter(*cloud);

  return *cloud;
}

// il tipo del messaggio dipende dal topic che è stato creato su quel tipo di mesaggio
void cloud_analyzer (const robotic_arm_inspector::planes_msgConstPtr& input) {
	// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
	pcl::PointCloud<pcl::PointXYZ> pc1;
	pcl::PointCloud<pcl::PointXYZ> pc2;
	pcl::fromROSMsg (input->pc1, pc1);
	pcl::fromROSMsg (input->pc2, pc2);  

  // ROS_INFO("PC1 points: %lu", pc1.points.size());
  pc1 = filter_cloud(pc1);
  // ROS_INFO("PC1 points: %lu", pc1.points.size());

  // ROS_INFO("PC2 points: %lu", pc2.points.size());
  pc2 = filter_cloud(pc2);
  // ROS_INFO("PC2 points: %lu", pc2.points.size());

  float dist = compute_average_distance(pc1,pc2);

}

int main (int argc, char** argv) {
	// Initialize ROS
	ros::init (argc, argv, "pad_checker_node");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("/robotic_arm_inspector/pad_check", 1, cloud_analyzer);

	// Create a ROS publisher for the filtered cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

	// Spin
	ros::spin ();
}