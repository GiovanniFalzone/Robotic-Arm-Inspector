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

void get_plane_struct(pcl::PointCloud<pcl::PointXYZ> cloud, plane_struct* ret){
	pcl::ModelCoefficients coefficients;
	pcl::PointIndices inliers;
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.005);

  publish_pc(cloud);
	seg.setInputCloud (cloud.makeShared ());
	seg.segment (inliers, coefficients); 

  int i = 0;
  if (inliers.indices.size () == 0) {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return;
  } else {
    while (inliers.indices.size () > 0 && i < 2){
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
      ret[i] = {ros_coefficients, inliers};
      
      ROS_INFO("plane %d: [%f,%f,%f,%f]", i, ros_coefficients.values[0],ros_coefficients.values[1],ros_coefficients.values[2],ros_coefficients.values[3]);
      
      seg.setInputCloud (cloud.makeShared ());
      seg.segment (inliers, coefficients);
      i++;
    }
  }
  return;
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

  float num = fabs(A*x + B*y + C*z + D);
  float den = sqrt(pow(A,2) + pow(B,2) + pow(C,2));
  float dist = num/den;

  // ROS_INFO("fabs [%f,%f,%f,%f]", fabs(A), fabs(B), fabs(C), fabs(D));
  // ROS_INFO("num: %f", num);
  // ROS_INFO("den: %f", den);

  return dist;
}

float compute_average_distance(pcl::PointCloud<pcl::PointXYZ> pc){
	plane_struct planes[2];
  get_plane_struct(pc, planes);
	// plane_struct plane2 = get_plane_struct(pc2);
  // ROS_INFO_STREAM("plane1: " << planes[0].inliers.indices.size());
  // ROS_INFO_STREAM("plane2: " << plane2.inliers);
  // adesso ho gli indici dei punti appartenenti al piano
  // devo spostare sia il punto che il piano in base alla posizione del kinect
  float point[3] = {1.0, 1.0, 1.0};
  float dist_sum = 0.0;
  size_t plane1_size = planes[0].inliers.indices.size();
  for (size_t i = 0; i < plane1_size; ++i){
    point[0] = pc.points[planes[0].inliers.indices[i]].x;
    point[1] = pc.points[planes[0].inliers.indices[i]].y;
    point[2] = pc.points[planes[0].inliers.indices[i]].z;
    // ROS_INFO("point [%f,%f,%f]", point[0], point[1], point[2]);
    // faccio la distanza punto piano tra un punto di un piano e l'altro piano
    float dist = compute_point_plane_distance(planes[1].coefficients, point);
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
  
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-1, 1); //1.3 works well
  pass.filter(*cloud);

  pass.setInputCloud(cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-1, 1); //1.3 works well
  pass.filter(*cloud);

  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-1, 1); //1.3 works well
  pass.filter(*cloud);

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.05, 0.05, 0.1);
  sor.filter (*cloud);

  // publish_pc(*cloud);

  return *cloud;
}

// il tipo del messaggio dipende dal topic che è stato creato su quel tipo di mesaggio
void cloud_analyzer (const robotic_arm_inspector::planes_msgConstPtr& input) {
	// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
	pcl::PointCloud<pcl::PointXYZ> pc;
	// pcl::PointCloud<pcl::PointXYZ> pc2;
	pcl::fromROSMsg (input->pc, pc);
	// pcl::fromROSMsg (input->pc2, pc2);  

  // ROS_INFO("PC1 points: %lu", pc1.points.size());
  pc = filter_cloud(pc);
  // ROS_INFO("PC1 points: %lu", pc1.points.size());

  // ROS_INFO("PC2 points: %lu", pc2.points.size());
  // pc2 = filter_cloud(pc2);
  // ROS_INFO("PC2 points: %lu", pc2.points.size());

  float dist = compute_average_distance(pc);

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