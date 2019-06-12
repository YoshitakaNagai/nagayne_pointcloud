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

		float begin_time;
		float tmp_time;
		float Hz = 100;
		float odom_x, odom_y, odom_z;
		float odom_qx, odom_qy, odom_qz, odom_qw;
		float tmp_odom_x;
		float tmp_odom_y;
		float tmp_odom_z;
		float tmp_odom_qx;
		float tmp_odom_qy;
		float tmp_odom_qz;
		float tmp_odom_qw;

		float odom_dx, odom_dy, odom_dz;

		double dR, dP, dY;
		double R, P, Y;
		double tmp_R, tmp_P, tmp_Y;
		double a, b, c, d;
		
				
		ros::Subscriber sub_pc;
		ros::Subscriber sub_lcl;
		
		ros::Publisher DCP_pub;
		
		sensor_msgs::PointCloud2 pc_;
		nav_msgs::Odometry init_odom;
		nav_msgs::Odometry odom;

		Eigen::Affine3f transform;
		Eigen::Matrix4f H_transform;
		Eigen::Matrix4f H_transform_XYZ;
		Eigen::Matrix4f H_transform_R;
		Eigen::Matrix4f H_transform_P;
		Eigen::Matrix4f H_transform_Y;
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
	H_transform = Eigen::Matrix4f::Identity();
	H_transform_XYZ = Eigen::Matrix4f::Identity();
	H_transform_R = Eigen::Matrix4f::Identity();
	H_transform_P = Eigen::Matrix4f::Identity();
	H_transform_Y = Eigen::Matrix4f::Identity();

	sub_pc = n.subscribe("/nagayne_PointCloud2/fusioned", 30, &Nagayne::fresh_pc_callback, this);
    sub_lcl = n.subscribe("/odom", 30, &Nagayne::lcl_callback, this);

	DCP_pub = n.advertise<sensor_msgs::PointCloud2>("/nagayne_PointCloud2/density_controlled", 30);
	//DCP_pub = n.advertise<CloudIUT>("/nagayne_PointCloud2/density_controlled", 30);
}


void Nagayne::controll_density(void)
{
	ros::Rate r(Hz);
	while(ros::ok()){
		if(fresh_pc_callback_flag && lcl_callback_flag){
			//std::cout << "flags : true" << std::endl;
			
			
			scorekeeper();
			std::cout << "before_veteran_pc_size : " << veteran_pc_->points.size() << std::endl;
			*veteran_pc_ += *fresh_pc_;

			//pcl::transformPointCloud(*veteran_pc_, *veteran_pc_, transform);
			pcl::transformPointCloud(*veteran_pc_, *veteran_pc_, H_transform);
			
			std::cout << "veteran_pc_size : " << veteran_pc_->points.size() << std::endl;
			pt_lifespan_keeper();
			
			//std::cout << "scored_veteran_pc_size : " << veteran_pc_->points.size() << std::endl;

			pc_downsampling();

			//std::cout << "downsampled_veteran_pc_size : " << veteran_pc_->points.size() << std::endl;
			
			pcl::toROSMsg(*veteran_pc_, pc_);
			pc_.header.stamp = ros::Time::now();
			pc_.header.frame_id = veteran_pc_->header.frame_id;
			DCP_pub.publish(pc_);
			//DCP_pub.publish(*veteran_pc_);
			
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
    
	if(!lidar_first_flag){
		*veteran_pc_ = *fresh_pc_;
		lidar_first_flag = true;
	}
	
	fresh_pc_callback_flag = true;
	//std::cout << "fresh_pc_callback_flag : " << fresh_pc_callback_flag << std::endl;
}


void Nagayne::lcl_callback(const nav_msgs::OdometryConstPtr &msg)
{
	odom = *msg;
	
	if(!odom_first_flag){
		tmp_odom_x = odom.pose.pose.position.x;
		tmp_odom_y = odom.pose.pose.position.y;
		tmp_odom_z = odom.pose.pose.position.z;
		
		odom_qx = odom.pose.pose.orientation.x;
		odom_qy = odom.pose.pose.orientation.y;
		odom_qz = odom.pose.pose.orientation.z;
		odom_qw = odom.pose.pose.orientation.w;
		tf::Quaternion tf_quaternion(odom_qx, odom_qy, odom_qz, odom_qw);
		tf::Matrix3x3 tf_matrix(tf_quaternion);
		tf_matrix.getRPY(tmp_R, tmp_P, tmp_Y);
		
		odom_first_flag = true;
	}

	//translation
	odom_x = odom.pose.pose.position.x;
	odom_y = odom.pose.pose.position.y;
	odom_z = odom.pose.pose.position.z;
	odom_dx = -(odom_x - tmp_odom_x);
	odom_dy = -(odom_y - tmp_odom_y);
	odom_dz = -(odom_z - tmp_odom_z);
	//Affine
	transform.translation() << odom_dx, odom_dy, odom_dz;
	//Homogeneous
	H_transform_XYZ(0,3) = odom_dx;
	H_transform_XYZ(1,3) = odom_dy;
	H_transform_XYZ(2,3) = odom_dz;
	
	
	//rotation
	odom_qx = odom.pose.pose.orientation.x;
	odom_qy = odom.pose.pose.orientation.y;
	odom_qz = odom.pose.pose.orientation.z;
	odom_qw = odom.pose.pose.orientation.w;
	tf::Quaternion tf_quaternion(odom_qx, odom_qy, odom_qz, odom_qw);
	tf::Matrix3x3 tf_matrix(tf_quaternion);
	tf_matrix.getRPY(R, P, Y);
	
	dR = -(R - tmp_R);
	dP = -(P - tmp_P);
	dY = -(Y - tmp_Y);
	
	//Affine
	transform.rotate(Eigen::AngleAxisf((float)dR, Eigen::Vector3f::UnitX()));
	transform.rotate(Eigen::AngleAxisf((float)dP, Eigen::Vector3f::UnitY()));
	transform.rotate(Eigen::AngleAxisf((float)dY, Eigen::Vector3f::UnitZ()));
	//Homogeneous
	H_transform_R(1,1) = cos(dR);
	H_transform_R(1,2) = -sin(dR);
	H_transform_R(2,1) = sin(dR);
	H_transform_R(2,2) = cos(dR);
	H_transform_P(0,0) = cos(dP);
	H_transform_P(0,2) = sin(dP);
	H_transform_P(2,0) = -sin(dP);
	H_transform_P(2,2) = cos(dP);
	H_transform_Y(0,0) = cos(dY);
	H_transform_Y(0,1) = -sin(dY);
	H_transform_Y(1,0) = sin(dY);
	H_transform_Y(1,1) = cos(dY);
	H_transform = H_transform_XYZ * H_transform_R * H_transform_P * H_transform_Y;

	std::cout << H_transform << std::endl;

	tmp_odom_x = odom_x;
	tmp_odom_y = odom_y;
	tmp_odom_z = odom_z;
	tmp_R = R;
	tmp_P = P;
	tmp_Y = Y;
	lcl_callback_flag = true;
	//std::cout << "lcl_callback_flag : " << lcl_callback_flag << std::endl;
}


void Nagayne::pt_lifespan_keeper(void)
{
	float dt = 1/Hz;
	std::cout << "dt=" << dt << std::endl;
	
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
	std::cout << "verteran_pc size = " << veteran_pc_->points.size() << std::endl;
	pcl::PassThrough<PointIUT> pass;
	pass.setInputCloud(veteran_pc_);
	pass.setFilterFieldName ("v");
	pass.setFilterLimits(0.0, a+c);
	pass.filter(*veteran_pc_);

	std::cout << "filtered veteran_pc size = " << veteran_pc_->points.size() << std::endl;
}
