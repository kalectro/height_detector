/*
 * Simple Feature detection for benchmarking purpose
 *
 *  Created on: Apr 24, 2013
 *      Author: Kai Franke
 */
 
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
