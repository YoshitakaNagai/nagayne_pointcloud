/*
 *	calculate unflatness of LaserScan
 *
 * 	author : Yoshitaka Nagai
 */

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "std_msgs/String.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLHeader.h>
#include <string>
#include <std_msgs/Float32.h>
#include <vector>
#include <Eigen/Core>

//#include "nagayne_pointcloud/point_id.h"

sensor_msgs::LaserScan laser_sub, laser_pub;

sensor_msgs::LaserScan unflatness_calc(sensor_msgs::LaserScan laser_data);

bool calc_flag = false;


void laser_callback(const sensor_msgs::LaserScanConstPtr &msg){
	laser_sub = *msg;
	laser_pub = unflatness_calc(laser_sub);	
	calc_flag = true;
}


float calc_polar2rectangular_x(float theta, float range){
	return range*cos(theta);
}


float calc_polar2rectangular_y(float theta, float range){
	return range*sin(theta);
}


sensor_msgs::LaserScan unflatness_calc(sensor_msgs::LaserScan laser_data){
	//p(x,y)
	size_t laser_data_size = laser_data.ranges.size();
	std::vector<Eigen::Vector2f> raw_xy;
	for(size_t i=0;i<laser_data_size;i++){
		float x = calc_polar2rectangular_x(i*laser_data.angle_increment, laser_data.ranges[i]);
		float y = calc_polar2rectangular_y(i*laser_data.angle_increment, laser_data.ranges[i]);
		Eigen::Vector2f tmp(x,y);
		raw_xy.push_back(tmp);
	}

	//vec(x,y)
	size_t raw_xy_size = raw_xy.size();
	std::vector<Eigen::Vector2f> diff;
	for(size_t i=0;i<raw_xy_size;i++){
		Eigen::Vector2f tmp = raw_xy[i+1] - raw_xy[i];
		diff.push_back(tmp);
	}

	//vec(x,y)/|vec(x,y)|
	size_t diff_size = diff.size();
	std::vector<Eigen::Vector2f> diff_normalized;
	for(size_t i=0;i<diff_size;i++){
		Eigen::Vector2f tmp = diff[i].normalized();
		diff_normalized.push_back(tmp);
	}
	
	//dot product
	size_t diff_normalized_size = diff_normalized.size();
	std::vector<float> dot;
	for(size_t i=0;i<diff_normalized_size;i++){
		float tmp = diff_normalized[i].dot(diff_normalized[i+1]);
		dot.push_back(tmp);
	}

	//unflatness
	size_t dot_size = dot.size();
	laser_data.intensities[0] = 0;
	laser_data.intensities[laser_data.ranges.size()-1] = 0;
	for(size_t i=0;i<dot_size;i++){
		laser_data.intensities[i+1] = 1 - dot[i];
	}
	
	return laser_data;
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "unflatness_calculator");
	ros::NodeHandle nh;
	ros::Subscriber sq_lidar_sub = nh.subscribe("/sq_lidar/scan",10,laser_callback);
	ros::Publisher unflatness_pub = nh.advertise<sensor_msgs::LaserScan>("/nagayne_LaserScan/unflatness",10);
	//ros::Publisher point_id_pub = nh.advertise<nagayne_pointcloud::point_id>("/nagayne_pointcloud/point_id",100);
	
	ros::Rate r(100);
	while(ros::ok())
	{
		if(calc_flag){
			unflatness_pub.publish(laser_pub);
			calc_flag = false;
		}
		r.sleep();
		ros::spinOnce();
	}
}
