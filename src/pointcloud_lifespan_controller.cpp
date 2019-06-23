/*
 *	lifespan controller of nagayne_pointcloud
 *
 * 	author : Yoshitaka Nagai
 */

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <boost/thread.hpp>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif


typedef pcl::PointXYZHSV PointIUT;
typedef pcl::PointCloud<PointIUT> CloudIUT;
typedef pcl::PointCloud<PointIUT>::Ptr CloudIUTPtr;


class Nagayne
{
	public:
		Nagayne();

		void odom_callback(const nav_msgs::OdometryConstPtr&);
		void fresh_pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
		
		void pc_lifespan_controller(void);
		void pt_lifespan_keeper(void);
		void scorekeeper(void);
		CloudIUTPtr pc_downsampling(void);

	private:
		bool fresh_pc_callback_flag = false;
		bool odom_callback_flag = false;
		bool lidar_first_flag = false;
		bool odom_first_flag = false;
		bool tf_listen_flag = false;

		float Hz = 100;
		
		double hokuyo_max_range;
		double uf_score_rate, distance_score, min_lifespan;
				
		ros::Subscriber sub_pc;
		ros::Subscriber sub_odom;	
		ros::Publisher DCP_pub;
	
		std::string target_frame = "/odom";
		std::string fixed_frame = "/centerlaser_";

		sensor_msgs::PointCloud2 pub_pc;
		sensor_msgs::PointCloud2 veteran_pc;
		sensor_msgs::PointCloud2 fresh_pc;
		sensor_msgs::PointCloud2 transformed_pc;
		
		nav_msgs::Odometry odom;
		
		tf::TransformListener listener;
		tf::StampedTransform transform;

		CloudIUTPtr veteran_pc_ {new CloudIUT};
		CloudIUTPtr fresh_pc_ {new CloudIUT};
		CloudIUTPtr transformed_pc_ {new CloudIUT};
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_lifespan_keeper");
	
	Nagayne nagayne_pointcloud;
	nagayne_pointcloud.pc_lifespan_controller();
	
	return 0;
}


Nagayne::Nagayne(void)
{
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	nh.getParam("uf_score_rate", uf_score_rate);
	nh.getParam("distance_score", distance_score);
	nh.getParam("min_lifespan", min_lifespan);
	nh.getParam("hokuyo_max_range", hokuyo_max_range);

	sub_pc = n.subscribe("/nagayne_PointCloud2/fusioned", 30, &Nagayne::fresh_pc_callback, this);
    sub_odom = n.subscribe("/odom", 30, &Nagayne::odom_callback, this);

	DCP_pub = n.advertise<sensor_msgs::PointCloud2>("/nagayne_PointCloud2/lifespan_controlled", 30);
}


void Nagayne::pc_lifespan_controller(void)
{
	ros::Rate r(Hz);
	while(ros::ok()){
		try{
       		listener.lookupTransform("/odom","/centerlaser_", ros::Time(0), transform);
			tf_listen_flag = true;
     	}   
     	catch (tf::TransformException ex){
       		ROS_ERROR("%s",ex.what());
       		ros::Duration(1.0).sleep();
    	} 
		
		if(fresh_pc_callback_flag && odom_callback_flag && tf_listen_flag){
			scorekeeper();
			pt_lifespan_keeper();
			
			pcl::toROSMsg(*veteran_pc_, veteran_pc);
			pcl_ros::transformPointCloud(target_frame, veteran_pc, transformed_pc, listener);
			pcl::fromROSMsg(transformed_pc, *veteran_pc_);
			
			pcl::toROSMsg(*fresh_pc_, fresh_pc);
			pcl_ros::transformPointCloud(target_frame, fresh_pc, transformed_pc, listener);
			pcl::fromROSMsg(transformed_pc, *fresh_pc_);
			
			*veteran_pc_ += *fresh_pc_;
					
			*veteran_pc_ = *pc_downsampling();
			
			pcl::toROSMsg(*veteran_pc_, pub_pc);
			pub_pc.header.stamp = ros::Time::now();
			DCP_pub.publish(pub_pc);
			fresh_pc_callback_flag = false;
			odom_callback_flag = false;
			tf_listen_flag = false;
		}
		r.sleep();
		ros::spinOnce();
	}
}


void Nagayne::fresh_pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *fresh_pc_);
    
	if(!lidar_first_flag){
		*veteran_pc_ = *fresh_pc_;
		lidar_first_flag = true;
	}
	
	fresh_pc_callback_flag = true;
}


void Nagayne::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
	odom = *msg;
	odom_callback_flag = true;
}


void Nagayne::pt_lifespan_keeper(void)
{
	float dt = 0.01;
	for(auto& pt : veteran_pc_->points){
		pt.v -= dt;
	}
}


void Nagayne::scorekeeper(void)
{
	for(auto& pt : fresh_pc_->points){
		float unflatness_score = 0.5 * pt.s;
		float score = (float)uf_score_rate * unflatness_score + (float)min_lifespan;
		std::cout << "score : " << score << std::endl;
		pt.v = score;
		//pt.v = (float)distance_score * sqrt(pow(pt.x,2) + pow(pt.y,2) + pow(pt.z,2)) / (float)hokuyo_max_range + min_lifespan;
	}
}


CloudIUTPtr Nagayne::pc_downsampling(void)
{
	pcl::PassThrough<PointIUT> pass;
	CloudIUTPtr filtered_pc_ {new CloudIUT};
	pass.setInputCloud(veteran_pc_);
	pass.setFilterFieldName ("v");
	pass.setFilterLimits(0.0, uf_score_rate + min_lifespan);
	pass.filter(*filtered_pc_);

	return filtered_pc_;
}

