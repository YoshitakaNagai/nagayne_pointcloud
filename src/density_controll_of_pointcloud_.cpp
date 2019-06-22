/*
 *	density controller of nagayne_pointcloud
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



class Nagayne
{
	public:
		Nagayne();

		void lcl_callback(const nav_msgs::OdometryConstPtr&);
		void fresh_pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
		
		void controll_density(void);
		void pt_lifespan_keeper(void);
		void scorekeeper(void);
		void pc_downsampling(void);

	private:
		int save_num;
		int scan_num;

		bool fresh_pc_callback_flag = false;
		bool lcl_callback_flag = false;
		bool lidar_first_flag = false;
		bool odom_first_flag = false;
		bool tf_listen_flag = false;
		bool transform_flag = false;

		float begin_time;
		float tmp_time;
		float Hz = 100;

		double a, b, c, d;
				
		ros::Subscriber sub_pc;
		ros::Subscriber sub_lcl;	
		ros::Publisher DCP_pub;
		ros::Time target_time;
	
		std::string target_frame = "/odom";
		std::string fixed_frame = "/centerlaser_";

		sensor_msgs::PointCloud2 pub_pc;
		sensor_msgs::PointCloud2 veteran_pc;
		sensor_msgs::PointCloud2 fresh_pc;
		sensor_msgs::PointCloud2 transformed_pc;
		
		nav_msgs::Odometry init_odom;
		nav_msgs::Odometry odom;
		
		tf::TransformListener listener;
		tf::StampedTransform transform;
     	//geometry_msgs::TransformStamped _transform;

		typedef pcl::PointXYZHSV PointIUT;
		typedef pcl::PointCloud<PointIUT> CloudIUT;
		typedef pcl::PointCloud<PointIUT>::Ptr CloudIUTPtr;
		CloudIUTPtr veteran_pc_ {new CloudIUT};
		CloudIUTPtr fresh_pc_ {new CloudIUT};
		CloudIUTPtr transformed_pc_ {new CloudIUT};
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "density_controll_of_pointcloud_v4");
	
	Nagayne nagayne_pointcloud;
	nagayne_pointcloud.controll_density();
	
	return 0;
}


Nagayne::Nagayne(void)
{
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	nh.getParam("save_num", save_num);
	nh.getParam("scan_num", scan_num);
	nh.getParam("a", a);
	nh.getParam("b", b);
	nh.getParam("c", c);
	nh.getParam("d", d);

	sub_pc = n.subscribe("/nagayne_PointCloud2/fusioned", 30, &Nagayne::fresh_pc_callback, this);
    sub_lcl = n.subscribe("/odom", 30, &Nagayne::lcl_callback, this);

	DCP_pub = n.advertise<sensor_msgs::PointCloud2>("/nagayne_PointCloud2/density_controlled", 30);
}


void Nagayne::controll_density(void)
{
	ros::Rate r(Hz);
	while(ros::ok()){
		try{
       		listener.lookupTransform("/odom","/centerlaser_", ros::Time(0), transform);
       		//tf::transformStampedTFToMsg(transform, _transform);
			tf_listen_flag = true;
     	}   
     	catch (tf::TransformException ex){
       		ROS_ERROR("%s",ex.what());
       		ros::Duration(1.0).sleep();
    	} 
		
		if(fresh_pc_callback_flag && lcl_callback_flag && tf_listen_flag){
			scorekeeper();
			pt_lifespan_keeper();
			pc_downsampling();
			//transform_flag = pcl_ros::transformPointCloud(target_frame, target_time, *veteran_pc_, fixed_frame, *transformed_pc_, listener);
			pcl::toROSMsg(*veteran_pc_, veteran_pc);
			pcl_ros::transformPointCloud(target_frame, veteran_pc, transformed_pc, listener);
			pcl::fromROSMsg(transformed_pc, *veteran_pc_);
			//*veteran_pc_ = *transformed_pc_;
			pcl::toROSMsg(*fresh_pc_, fresh_pc);
			pcl_ros::transformPointCloud(fixed_frame, fresh_pc, transformed_pc, listener);
			pcl::fromROSMsg(transformed_pc, *fresh_pc_);
			//veteran_pc_->header.stamp = fresh_pc_->header.stamp;
			*veteran_pc_ += *fresh_pc_;
			veteran_pc_->header.frame_id = fixed_frame;
				
			pcl::toROSMsg(*veteran_pc_, pub_pc);
			pub_pc.header.stamp = ros::Time::now();
			DCP_pub.publish(pub_pc);
			fresh_pc_callback_flag = false;
			lcl_callback_flag = false;
			tf_listen_flag = false;
			transform_flag = false;
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


void Nagayne::lcl_callback(const nav_msgs::OdometryConstPtr &msg)
{
	odom = *msg;
	target_time = odom.header.stamp;
	lcl_callback_flag = true;
}


void Nagayne::pt_lifespan_keeper(void)
{
	float dt = 1/Hz;
	//for(auto& pt : transformed_pc_->points){
	for(auto& pt : veteran_pc_->points){
		pt.v -= dt;
	}
}


void Nagayne::scorekeeper(void)
{
	for(auto& pt : fresh_pc_->points){
		float unflatness_score = pt.s/2;
		float score = (float)a*unflatness_score + (float)c;
		pt.v = score;
		//pt.v = (float)b*sqrt(pow(pt.x,2) + pow(pt.y,2) + pow(pt.z,2))/30.0 + c;
	}
}


void Nagayne::pc_downsampling(void)
{
	pcl::PassThrough<PointIUT> pass;
	//pass.setInputCloud(transformed_pc_);
	pass.setInputCloud(veteran_pc_);
	pass.setFilterFieldName ("v");
	pass.setFilterLimits(0.0, a+c);
	pass.filter(*veteran_pc_);
}

