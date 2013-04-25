/*
 * Simple Feature detection for benchmarking purpose
 *
 *  Created on: Apr 24, 2013
 *      Author: Kai Franke
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);

int main(int, char**);

// if false filtered points will be removed from the point cloud, if true overwritten by NaN
bool keep_organized;

// Initialize Subscriber
ros::Subscriber sub;

// Initialize Publisher for plane coefficients and debug pointcloud
ros::Publisher pub_coeffs;
ros::Publisher pub_no_plane;
ros::Publisher pub_height;

// Distance threshold for plane
double threshold_plane;

// Size of the downsampled voxel
double voxel_size;

// Minimum number of points on a plane to be recognized as a plane
int min_inliers;

// Create ROS message for filtered point cloud
sensor_msgs::PointCloud2 input_filtered;

// Create ROS message for debug point cloud
// sensor_msgs::PointCloud2 output;

// Declare the segmentation object for planes
pcl::SACSegmentation<pcl::PointXYZ> seg_plane;

// Declare the filtering object for planes
pcl::ExtractIndices<pcl::PointXYZ> extract_planes;

// Callback function when subscribed to point cloud
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);

// checks if found plane coefficients are the floor
bool isFloor( const pcl::ModelCoefficients::Ptr coefficients );

// parameter variable for the plane
double a_max, a_min, b_max, b_min, c_max, c_min, d_max, d_min;
