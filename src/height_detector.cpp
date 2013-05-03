/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

//\Author Kai Franke, Robert Bosch LLC

#include <height_detector.h>

int main (int argc, char** argv)
{
	ros::init (argc, argv, "height_detector");
	ros::NodeHandle nh("~");
	// Create a ROS subscriber for the input point cloud
	sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

	// Create publihser to publish debug pointcloud
	pub_no_plane = nh.advertise<sensor_msgs::PointCloud2> ("pointcloud_no_plane", 1);
	
	// Create publisher to publish found coefficients
	pub_coeffs = nh.advertise<pcl::ModelCoefficients> ("coefficients_plane", 1);
	
	// Create publisher to publish found height
	pub_height = nh.advertise<std_msgs::Float32> ("height", 1);
	
	// get all parameters from parameter server
	nh.param("threshold", threshold_plane, 0.1);
	nh.param("keep_organized", keep_organized, false);
	nh.param("voxel_size", voxel_size, 0.05);
	nh.param("min_inliers", min_inliers, 500);
	nh.param("a_min", a_min,  0.95);
	nh.param("a_max", a_max,  1.05);
	nh.param("b_min", b_min, -0.05);
	nh.param("b_max", b_max,  0.05);
	nh.param("c_min", c_min, -0.05);
	nh.param("c_max", c_max,  0.05);
	nh.param("d_min", d_min, -0.90);
	nh.param("d_max", d_max, -0.40);
	
	// Set up SAC parameters for plane segmentation
	seg_plane.setOptimizeCoefficients (true);
	seg_plane.setModelType (pcl::SACMODEL_PLANE);
	seg_plane.setMethodType (pcl::SAC_RANSAC);
	seg_plane.setMaxIterations (1000);

	// Extract the found plane to remove the table
	extract_planes.setNegative (true);

	// Spin
	ros::spin ();
}

bool isFloor( const pcl::ModelCoefficients::Ptr coefficients  )
{
  double a = coefficients->values[0];
  double b = coefficients->values[1];
  double c = coefficients->values[2];
  double d = coefficients->values[3];
  
  if( a < a_min || a > a_max )
  {
    return false;
  }
  if( b < b_min || b > b_max )
  {
    return false;
  }
  if( c < c_min || c > c_max )
  {
    return false;
  }
  if( d < d_min || d > d_max )
  {
    return false;
  }
  
  return true;
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	// Construct point cloud to work with
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // Point indices for found plane
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
  
	// Construct point cloud after plane removal
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_plane (new pcl::PointCloud<pcl::PointXYZ>);

	// construct coefficients for plane
	pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);

	// Create a pointcloud to store the downsampled point cloud
	sensor_msgs::PointCloud2::Ptr input_voxeled (new sensor_msgs::PointCloud2);


	//
	// downscale the points
	//
	// Create the filtering object and downsample the dataset using the parameter leaf size
	pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
	sor.setInputCloud (input);
	sor.setLeafSize (voxel_size,voxel_size,voxel_size);
	sor.filter (*input_voxeled);


	// convert the message
	pcl::fromROSMsg (*input_voxeled, *cloud);

  // set maximal distance from point to planar surface to be identified as plane
  seg_plane.setDistanceThreshold (threshold_plane);
	  
  do
  {
    seg_plane.setInputCloud (cloud);
	  // 
	  // find biggest planar surface
	  // 
	  inliers_plane.reset( new pcl::PointIndices );
	  seg_plane.segment (*inliers_plane, *coefficients_plane);
  
    //publish found coefficients for plane for debug
    pub_coeffs.publish(coefficients_plane);
      
    // check if plane with enough points was found
    if( inliers_plane->indices.size() < min_inliers )
    {
      ROS_WARN("Found plane with %lu inliers which is less than min_inliers=%i", inliers_plane->indices.size(), min_inliers);
      return;
    }
    
    // check if coefficients represnet floor
    if( isFloor( coefficients_plane ) )
    {
      std_msgs::Float32 height;
      height.data = coefficients_plane->values[3];
      pub_height.publish( height );
      return;
    }
    
	  // 
	  // remove plane from point cloud
	  // 
	  extract_planes.setInputCloud(cloud);
	  extract_planes.setIndices (inliers_plane);
	  extract_planes.setKeepOrganized(keep_organized);
	  extract_planes.filter (*cloud_no_plane);
	  
	  /* DEBUG output
	  // convert back to ROS message
	  //
	  pcl::toROSMsg(*cloud_no_plane, output);
	  // fill in header
	  output.header.stamp = ros::Time::now();
	  output.header.frame_id = "camera";

	  // publish points of remaining cloud
	  pub_no_plane.publish(output);*/
	  
	  // set filtered cloud to be new input cloud
	  cloud = cloud_no_plane;
  }  while( inliers_plane->indices.size() > min_inliers );  // abort if plane too small
}

