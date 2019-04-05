#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <math.h>
#include <string>
#include <std_msgs/Float32.h>
#include <vector>
#include <Eigen/Core>

#include "nagayne_pointcloud/point_id.h"

sensor_msgs::LaserScan laser, laser0, laser1, laser2;

void unflatness_calc();
double calc_rotation_speed(double delta_time, double dYAW);

std_msgs::Float32 lidar_tfrog_vel;

double delta_yaw = 0;
double delta_time = 0;

bool laser0_flag = false;
bool laser1_flag = false;
bool laser2_flag = false;
bool calc_flag = false;
bool first_flag = false;

void laser_callback(const sensor_msgs::LaserScanConstPtr &msg){
	laser = *msg;
	if(laser.header.frame_id == "laser1_link"){
    	laser0 = laser;
		//std::cout << "laser0.angle_increment = " << laser0.angle_increment << std::endl;
	 	laser0_flag =true;
		//std::cout << "laser0_flag = true" << std::endl;
	}else if(laser.header.frame_id == "laser2_link"){
        laser1 = laser;
		laser1_flag =true;
	}else if(laser.header.frame_id == "laser3_link"){
        laser2 = laser;
		laser2_flag =true;
	}
	
	if(laser0_flag && laser1_flag && laser2_flag && first_flag){
		//std::cout << "a" << std::endl;
		unflatness_calc();
		calc_flag = true;
	}
}


void tfrog_callback(const std_msgs::Float32ConstPtr &msg){
	lidar_tfrog_vel = *msg;
}

float calc_polar2rectangular_x(float theta, float range){
	return range*cos(theta);
}

float calc_polar2rectangular_y(float theta, float range){
	return range*sin(theta);
}

void unflatness_calc(){
	float k = 1.0;
	//laser0
	//p(x,y)
	std::vector<Eigen::Vector2f> raw_xy_0;
	for(size_t i=0;i<laser0.ranges.size();i++){
		float x = calc_polar2rectangular_x(i*laser0.angle_increment, laser0.ranges[i]);
		float y = calc_polar2rectangular_y(i*laser0.angle_increment, laser0.ranges[i]);
		Eigen::Vector2f tmp(x,y);
		raw_xy_0.push_back(tmp);
	}

	//vec(x,y)
	std::vector<Eigen::Vector2f> diff0;
	for(size_t i=0;i<raw_xy_0.size();i++){
		Eigen::Vector2f tmp = raw_xy_0[i+1] - raw_xy_0[i];
		diff0.push_back(tmp);
	}

	//vec(x,y)/|vec(x,y)|
	std::vector<Eigen::Vector2f> diff0_normalized;
	for(size_t i=0;i<diff0.size();i++){
		Eigen::Vector2f tmp = diff0[i].normalized();
		diff0_normalized.push_back(tmp);
	}
	
	//dot product
	std::vector<float> dot_0;
	for(size_t i=0;i<diff0_normalized.size();i++){
		float tmp = diff0_normalized[i].dot(diff0_normalized[i+1]);
		dot_0.push_back(tmp);
	}

	//unflatness
	laser0.intensities[0] = 0;
	laser0.intensities[laser0.ranges.size()-1] = 0;
	for(size_t i=0;i<dot_0.size();i++){
		laser0.intensities[i+1] = k*(1 - dot_0[i]);
	}


	//laser1
	//p(x,y)
	std::vector<Eigen::Vector2f> raw_xy_1;
	for(size_t i=0;i<laser1.ranges.size();i++){
		float x = calc_polar2rectangular_x(i*laser1.angle_increment, laser1.ranges[i]);
		float y = calc_polar2rectangular_y(i*laser1.angle_increment, laser1.ranges[i]);
		Eigen::Vector2f tmp(x,y);
		raw_xy_1.push_back(tmp);
	}

	//vec(x,y)
	std::vector<Eigen::Vector2f> diff1;
	for(size_t i=0;i<raw_xy_1.size();i++){
		Eigen::Vector2f tmp = raw_xy_1[i+1] - raw_xy_1[i];
		diff1.push_back(tmp);
	}

	//vec(x,y)/|vec(x,y)|
	std::vector<Eigen::Vector2f> diff1_normalized;
	for(size_t i=0;i<diff1.size();i++){
		Eigen::Vector2f tmp = diff1[i].normalized();
		diff1_normalized.push_back(tmp);
	}
	
	//dot product
	std::vector<float> dot_1;
	for(size_t i=0;i<diff1_normalized.size();i++){
		float tmp = diff1_normalized[i].dot(diff1_normalized[i+1]);
		dot_1.push_back(tmp);
	}

	//unflatness
	laser1.intensities[0] = 0;
	laser1.intensities[laser1.ranges.size()-1] = 0;
	for(size_t i=0;i<dot_1.size();i++){
		laser1.intensities[i+1] = k*(1 - dot_1[i]);
	}
	
	
	//laser2
	//p(x,y)
	std::vector<Eigen::Vector2f> raw_xy_2;
	for(size_t i=0;i<laser2.ranges.size();i++){
		float x = calc_polar2rectangular_x(i*laser2.angle_increment, laser2.ranges[i]);
		float y = calc_polar2rectangular_y(i*laser2.angle_increment, laser2.ranges[i]);
		Eigen::Vector2f tmp(x,y);
		raw_xy_2.push_back(tmp);
	}

	//vec(x,y)
	std::vector<Eigen::Vector2f> diff2;
	for(size_t i=0;i<raw_xy_2.size();i++){
		Eigen::Vector2f tmp = raw_xy_2[i+1] - raw_xy_2[i];
		diff2.push_back(tmp);
	}

	//vec(x,y)/|vec(x,y)|
	std::vector<Eigen::Vector2f> diff2_normalized;
	for(size_t i=0;i<diff2.size();i++){
		Eigen::Vector2f tmp = diff2[i].normalized();
		diff2_normalized.push_back(tmp);
	}
	
	//dot product
	std::vector<float> dot_2;
	for(size_t i=0;i<diff2_normalized.size();i++){
		float tmp = diff2_normalized[i].dot(diff2_normalized[i+1]);
		dot_2.push_back(tmp);
	}

	//unflatness
	laser2.intensities[0] = 0;
	laser2.intensities[laser2.ranges.size()-1] = 0;
	for(size_t i=0;i<dot_2.size();i++){
		laser2.intensities[i+1] = k*(1 - dot_2[i]);
	}

}


double calc_rotation_speed(double delta_time, double dYAW){
	return dYAW/delta_time;
}



int main(int argc, char *argv[])
{
	ros::init(argc, argv, "viz_unflatness");
	ros::NodeHandle nh;
	ros::Subscriber laser_sub = nh.subscribe("/sq_lidar/scan",100,laser_callback);
	ros::Subscriber tfrog_sub = nh.subscribe("/lidar_tfrog/vel",100,tfrog_callback);
	ros::Publisher laser0_pub = nh.advertise<sensor_msgs::LaserScan>("/output2intensities/laser0_unflat",100);
	ros::Publisher laser1_pub = nh.advertise<sensor_msgs::LaserScan>("/output2intensities/laser1_unflat",100);
	ros::Publisher laser2_pub = nh.advertise<sensor_msgs::LaserScan>("/output2intensities/laser2_unflat",100);
	
	tf::TransformListener listener;
	tf::TransformListener listener1;
	tf::TransformListener listener2;
	tf::TransformListener listener3;

	bool lap_flag;
	bool ff = true;
	ros::Time now_time_ = ros::Time::now();
	ros::Time tmp_time_ = now_time_;
	double now_time = now_time_.toSec();
	double tmp_time = tmp_time_.toSec();
	double tmp_time2 = tmp_time_.toSec();
	double tmp_yaw; // for calc 1lap time

	double yawtmp = 0; //for calc delta_yaw
	double rot_speed;
	double pid_vel;

	ros::Rate r(100);
	while(ros::ok())
	{
		bool transformed = true;
		tf::StampedTransform transform;
    	geometry_msgs::TransformStamped _transform;
		
    	try{
       		listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
       		tf::transformStampedTFToMsg(transform, _transform);
			transformed = true;
			//std::cout << "transformed" << std::endl;
    	}   
    	catch (tf::TransformException ex1){
       		ROS_ERROR("%s",ex1.what());
       		ros::Duration(1.0).sleep();
    	} 
		
		bool transformed1 = true;
		tf::StampedTransform transform1;
     	geometry_msgs::TransformStamped _transform1;
		
     	try{
       		listener1.lookupTransform("/base_link", "/laser1_link", ros::Time(0), transform1);
       		tf::transformStampedTFToMsg(transform1, _transform1);
			transformed1 = true;
			//std::cout << "transformed_1" << std::endl;
     	}   
     	catch (tf::TransformException ex1){
       		ROS_ERROR("%s",ex1.what());
       		ros::Duration(0.1).sleep();
    	}   
		
		bool transformed2 = true;
		tf::StampedTransform transform2;
     	geometry_msgs::TransformStamped _transform2;
		
     	try{
       		listener2.lookupTransform("/base_link", "/laser2_link", ros::Time(0), transform2);
       		tf::transformStampedTFToMsg(transform2, _transform2);
			transformed2 = true;
			//std::cout << "transformed_2" << std::endl;
     	}   
     	catch (tf::TransformException ex2){
       		ROS_ERROR("%s",ex2.what());
       		ros::Duration(0.1).sleep();
    	}   
		
		bool transformed3 = true;
		tf::StampedTransform transform3;
     	geometry_msgs::TransformStamped _transform3;
		
     	try{
       		listener3.lookupTransform("/base_link", "/laser3_link", ros::Time(0), transform3);
       		tf::transformStampedTFToMsg(transform3, _transform3);
			transformed3 = true;
			//std::cout << "transformed_3" << std::endl;
     	}   
     	catch (tf::TransformException ex3){
       		ROS_ERROR("%s",ex3.what());
       		ros::Duration(0.1).sleep();
    	} 
		
		if(transformed && transformed1 && transformed2 && transformed3){
			double yaw[3];
			yaw[0] = tf::getYaw(_transform1.transform.rotation);
			yaw[1] = tf::getYaw(_transform2.transform.rotation);
			yaw[2] = tf::getYaw(_transform3.transform.rotation);
			double yawmax = yaw[0];
			for(int i=0;i<3;i++){
				if(yaw[i] < 0){
					yaw[i] = M_PI + (M_PI +yaw[i]);
				}
			}
			double tmpyaw[3];
			double dyaw[3];
			if(first_flag){
				for(int i=0;i<3;i++){
					dyaw[i] = fabs(yaw[i] - tmpyaw[i]);
				}
				double tmpdyaw = dyaw[1];
				for(int i=1;i<3;i++){
					if(tmpdyaw > dyaw[i]){
						tmpdyaw = dyaw[i];
					}
					tmpyaw[i] = yaw[i];
				}
				delta_yaw = tmpdyaw;
			}else{
				for(int i=0;i<3;i++){
					tmpyaw[i] = yaw[i];
				}
			}

			for(int i=0;i<3;i++){
				tmpyaw[i] = yaw[i];
			}

			if(calc_flag && first_flag){
				laser0_pub.publish(laser0);
				laser1_pub.publish(laser1);
				laser2_pub.publish(laser2);
				//std::cout << "published" << std::endl;
				calc_flag = false;
			}
			//std::cout << "first_flag = true" << std::endl;
			now_time_ = ros::Time::now();
			now_time = now_time_.toSec();
			delta_time = now_time - tmp_time;
			tmp_time = now_time;
			/*
			tmp_time = now_time;
			tmp_yaw = YAW;
			if(ff){
				if(YAW > alpha){
					lap_flag = false;
				}else{
					lap_flag = true;
					ff = false;
				}
			}
			if(lap_flag){
				std::cout << "1lap time : " << now_time - tmp_time2 << std::endl;
				tmp_time2 = now_time;
				lap_flag = false;
			}
			if(YAW > 2*M_PI -0.1){
				ff = true;
			}
			*/
			first_flag = true;
		}
		r.sleep();
		ros::spinOnce();
	}
}
