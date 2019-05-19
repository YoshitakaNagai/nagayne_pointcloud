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
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

#include <tf/tf.h>
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
CloudIUTPtr veteran_pc_ (new CloudIUT);
CloudIUTPtr fresh_pc_ (new CloudIUT);
CloudIUTPtr transformed_pc_ (new CloudIUT);


class Nagayne
{
	public:
		Nagayne();

		void lcl_callback(const nav_msgs::OdometryConstPtr&);
		void fresh_pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
		
		void controll_density(void);

		CloudIUTPtr pt_lifespan_keeper(CloudIUTPtr);
		CloudIUTPtr scorekeeper(CloudIUTPtr);
		CloudIUTPtr pc_downsampling(CloudIUTPtr);

	private:
		int save_num;
		int scan_num;

		bool fresh_pc_callback_flag = false;
		bool lcl_callback_flag = false;
		bool first_flag = false;

		float begin_time;
		float tmp_time;
		float Hz = 100;
		float odom_x, odom_y, odom_z;
		float odom_qx, odom_qy, odom_qz, odom_qw;
		float tmp_odom_x = 0.0;
		float tmp_odom_y = 0.0;
		float tmp_odom_z = 0.0;
		float tmp_odom_qx = 0.0;
		float tmp_odom_qy = 0.0;
		float tmp_odom_qz = 0.0;
		float tmp_odom_qw = 0.0;
		float odom_dx, odom_dy, odom_dz;
		float odom_dqx, odom_dqy, odom_dqz, odom_dqw;

		double dR, dP, dY;
		double a, b, c, d;
		
				
		ros::Subscriber sub_pc;
		ros::Subscriber sub_lcl;
		
		ros::Publisher DCP_pub;
		
		sensor_msgs::PointCloud2 pc_;
		nav_msgs::Odometry init_odom;
		nav_msgs::Odometry odom;

		Eigen::Affine3f transform;
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

	transform = Eigen::Affine3f::Identity();

	sub_pc = n.subscribe("/nagayne_PointCloud2/fusioned", 10, &Nagayne::fresh_pc_callback, this);
    sub_lcl = n.subscribe("/odom", 10, &Nagayne::lcl_callback, this);

	DCP_pub = n.advertise<sensor_msgs::PointCloud2>("/nagayne_PointCloud2/density_controlled", 10);
}


void Nagayne::controll_density(void){
	ros::Rate r(Hz);
	while(ros::ok()){
		if(fresh_pc_callback_flag && lcl_callback_flag){
			std::cout << "flags : true" << std::endl;
			
			pcl::transformPointCloud(*veteran_pc_, *veteran_pc_, transform);
			fresh_pc_ = scorekeeper(fresh_pc_);
			//pcl::transformPointCloud(*veteran_pc_, *transformed_pc_, transform);
			//veteran_pc_ = transformed_pc_;
			std::cout << "before_veteran_pc_size : " << veteran_pc_->points.size() << std::endl;
			*veteran_pc_ += *fresh_pc_;

			std::cout << "veteran_pc_size : " << veteran_pc_->points.size() << std::endl;
			veteran_pc_ = pt_lifespan_keeper(veteran_pc_);
			veteran_pc_ = pc_downsampling(veteran_pc_);
			
			std::cout << "after_pc_size : " << veteran_pc_->points.size() << std::endl;
			
			pcl::toROSMsg(*veteran_pc_, pc_);
			pc_.header.stamp = ros::Time::now();
			pc_.header.frame_id = veteran_pc_->header.frame_id;
			DCP_pub.publish(pc_);
			
			fresh_pc_callback_flag = false;
			lcl_callback_flag = false;
		}
		r.sleep();
		ros::spinOnce();
	}
}


void Nagayne::fresh_pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *fresh_pc_);
    
	if(!first_flag){
		*veteran_pc_ = *fresh_pc_;
		first_flag = true;
	}
	
	fresh_pc_callback_flag = true;
	//std::cout << "fresh_pc_callback_flag : " << fresh_pc_callback_flag << std::endl;
}


void Nagayne::lcl_callback(const nav_msgs::OdometryConstPtr &msg)
{
	odom = *msg;

	odom_x = odom.pose.pose.position.x;
	odom_y = odom.pose.pose.position.y;
	odom_z = odom.pose.pose.position.z;
	odom_dx = odom_x - tmp_odom_x;
	odom_dy = odom_y - tmp_odom_y;
	odom_dz = odom_z - tmp_odom_z;
	transform.translation() << odom_dx, odom_dy, odom_dz;

	odom_qx = odom.pose.pose.orientation.x;
	odom_qy = odom.pose.pose.orientation.y;
	odom_qz = odom.pose.pose.orientation.z;
	odom_qw = odom.pose.pose.orientation.w;
	odom_dqx = odom_qx - tmp_odom_qx;
	odom_dqy = odom_qy - tmp_odom_qy;
	odom_dqz = odom_qz - tmp_odom_qz;
	odom_dqw = odom_qw - tmp_odom_qw;
	tf::Quaternion tf_quaternion(odom_dqx, odom_dqy, odom_dqz, odom_dqw);
	tf::Matrix3x3 tf_matrix(tf_quaternion);
	tf_matrix.getRPY(dR, dP, dY);
	transform.rotate(Eigen::AngleAxisf((float)dR, Eigen::Vector3f::UnitX()));
	transform.rotate(Eigen::AngleAxisf((float)dP, Eigen::Vector3f::UnitY()));
	transform.rotate(Eigen::AngleAxisf((float)dY, Eigen::Vector3f::UnitZ()));

	tmp_odom_x = odom_x;
	tmp_odom_y = odom_y;
	tmp_odom_z = odom_z;

	tmp_odom_qx = odom_qx;
	tmp_odom_qy = odom_qy;
	tmp_odom_qz = odom_qz;
	tmp_odom_qw = odom_qw;

	lcl_callback_flag = true;
	//std::cout << "lcl_callback_flag : " << lcl_callback_flag << std::endl;
}


CloudIUTPtr Nagayne::pt_lifespan_keeper(CloudIUTPtr vpc)
{
	float dt = 1/Hz;
	for(auto& pt : vpc->points){
		pt.v -= dt;
	}
	return vpc;
}


CloudIUTPtr Nagayne::scorekeeper(CloudIUTPtr before_scored_pc_)
{
	for(auto& pt : before_scored_pc_->points){
		float unflatness_score = pt.s/2;
		float score = (float)a*unflatness_score + (float)c;
		pt.v = score;
		//pt.v = (float)b*sqrt(pow(pt.x,2) + pow(pt.y,2) + pow(pt.z,2))/30.0 + c;
	}

	return before_scored_pc_;
}


CloudIUTPtr Nagayne::pc_downsampling(CloudIUTPtr after_scored_pc)
{
	CloudIUTPtr pc_filterd (new CloudIUT);
	pcl::PassThrough<PointIUT> pass;
	pass.setInputCloud(after_scored_pc);
	pass.setFilterFieldName ("v");
	pass.setFilterLimits (0, (float)a + (float)c);
	//pass.setFilterLimits (0, b + c);
	pass.filter(*pc_filterd);

	return pc_filterd;
}
