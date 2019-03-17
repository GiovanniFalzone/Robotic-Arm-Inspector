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

#include <robotic_arm_inspector/planes_msg.h>

ros::Publisher pub;

void get_plane_coefficients(pcl::PointCloud<pcl::PointXYZ> cloud){
	pcl::ModelCoefficients coefficients;
	pcl::PointIndices inliers;
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (cloud.makeShared ());
	seg.segment (inliers, coefficients); 

	// Publish the model coefficients
	pcl_msgs::ModelCoefficients ros_coefficients;
	pcl_conversions::fromPCL(coefficients, ros_coefficients);
	pub.publish (ros_coefficients);
  ROS_INFO("[%f,%f,%f,%f]", ros_coefficients.values[0],ros_coefficients.values[1],ros_coefficients.values[2],ros_coefficients.values[3]);
}

// il tipo del messaggio dipende dal topic che è stato creato su quel tipo di mesaggio
void cloud_analyzer (const robotic_arm_inspector::planes_msgConstPtr& input) {
	// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
	pcl::PointCloud<pcl::PointXYZ> pc1;
	pcl::PointCloud<pcl::PointXYZ> pc2;
	pcl::fromROSMsg (input->pc1, pc1);
	pcl::fromROSMsg (input->pc2, pc2);
	get_plane_coefficients(pc1);
	get_plane_coefficients(pc2);
}

int main (int argc, char** argv) {
	// Initialize ROS
	ros::init (argc, argv, "pad_checker_node");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("/robotic_arm_inspector/pad_check", 1, cloud_analyzer);

	// Create a ROS publisher for the output model coefficients
	pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);

	// Spin
	ros::spin ();
}