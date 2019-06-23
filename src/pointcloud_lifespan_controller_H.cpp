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
		void translation(void);
		double pi_2_pi(double);

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
		float odom_xg, odom_yg, odom_zg;
		float tmp_odom_xg, tmp_odom_yg, tmp_odom_zg;
		float odom_dxg, odom_dyg, odom_dzg;
		float odom_qxg, odom_qyg, odom_qzg, odom_qwg;

		double Rg, Pg, Yg;
		double dRg, dPg, dYg;
		double tmp_Rg, tmp_Pg, tmp_Yg;
		double a, b, c, d;
		
				
		ros::Subscriber sub_pc;
		ros::Subscriber sub_lcl;
		
		ros::Publisher DCP_pub;
		
		sensor_msgs::PointCloud2 pc_;
		nav_msgs::Odometry init_odom;
		nav_msgs::Odometry odom;
		
		typedef pcl::PointXYZHSV PointIUT;
		typedef pcl::PointCloud<PointIUT> CloudIUT;
		typedef pcl::PointCloud<PointIUT>::Ptr CloudIUTPtr;
		CloudIUTPtr veteran_pc_ {new CloudIUT};
		CloudIUTPtr fresh_pc_ {new CloudIUT};
		CloudIUTPtr transformed_pc_ {new CloudIUT};

		Eigen::Matrix4f dH_transform_local = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f H_transform_XYZ = Eigen::Matrix4f::Identity();
		Eigen::Vector3f V_transform_dXYZ = Eigen::Vector3f::Zero();
		Eigen::Matrix4f H_XYZ = Eigen::Matrix4f::Identity();
		Eigen::Vector3f V_dXYZ = Eigen::Vector3f::Zero();
		
		Eigen::Matrix3f H_transform_R = Eigen::Matrix3f::Identity();
		Eigen::Matrix3f H_transform_P = Eigen::Matrix3f::Identity();
		Eigen::Matrix3f H_transform_Y = Eigen::Matrix3f::Identity();
		Eigen::Matrix3f H_transform_dR = Eigen::Matrix3f::Identity();
		Eigen::Matrix3f H_transform_dP = Eigen::Matrix3f::Identity();
		Eigen::Matrix3f H_transform_dY = Eigen::Matrix3f::Identity();
		Eigen::Matrix3f H_rotation_d = Eigen::Matrix3f::Identity();
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
		if(fresh_pc_callback_flag && lcl_callback_flag){
			scorekeeper();
			pt_lifespan_keeper();
			std::cout << dH_transform_local << std::endl;
			pcl::transformPointCloud(*veteran_pc_, *transformed_pc_, dH_transform_local);
			//translation();
			*veteran_pc_ = *transformed_pc_;
			pc_downsampling();
			//veteran_pc_->header.stamp = ros::Time::now().toSec();
			veteran_pc_->header.frame_id = fresh_pc_->header.frame_id;
			*veteran_pc_ += *fresh_pc_;
			
			pcl::toROSMsg(*veteran_pc_, pc_);
			pc_.header.stamp = ros::Time::now();
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
    
	if(!lidar_first_flag){
		*veteran_pc_ = *fresh_pc_;
		lidar_first_flag = true;
	}
	
	fresh_pc_callback_flag = true;
}


void Nagayne::lcl_callback(const nav_msgs::OdometryConstPtr &msg)
{
	odom = *msg;
	
	if(!odom_first_flag){
		tmp_odom_xg = odom.pose.pose.position.x;
		tmp_odom_yg = odom.pose.pose.position.y;
		tmp_odom_zg = odom.pose.pose.position.z;
		
		odom_qxg = odom.pose.pose.orientation.x;
		odom_qyg = odom.pose.pose.orientation.y;
		odom_qzg = odom.pose.pose.orientation.z;
		odom_qwg = odom.pose.pose.orientation.w;
		tf::Quaternion tf_quaternion(odom_qxg, odom_qyg, odom_qzg, odom_qwg);
		tf::Matrix3x3 tf_matrix(tf_quaternion);
		tf_matrix.getRPY(tmp_Rg, tmp_Pg, tmp_Yg);
		odom_first_flag = true;
	}

	//translation
	odom_xg = odom.pose.pose.position.x;
	odom_yg = odom.pose.pose.position.y;
	odom_zg = odom.pose.pose.position.z;
	
	odom_dxg = odom_xg - tmp_odom_xg;
	odom_dyg = odom_yg - tmp_odom_yg;
	odom_dzg = odom_zg - tmp_odom_zg;
	//Homogeneous_translation
	V_dXYZ(0) = -odom_dxg;
	V_dXYZ(1) = -odom_dyg;
	V_dXYZ(2) = -odom_dzg;
	
	//rotation
	odom_qxg = odom.pose.pose.orientation.x;
	odom_qyg = odom.pose.pose.orientation.y;
	odom_qzg = odom.pose.pose.orientation.z;
	odom_qwg = odom.pose.pose.orientation.w;
	tf::Quaternion tf_quaternion(odom_qxg, odom_qyg, odom_qzg, odom_qwg);
	tf::Matrix3x3 tf_matrix(tf_quaternion);
	tf_matrix.getRPY(Rg, Pg, Yg);
	
	// Rg = pi_2_pi(Rg);
	// Pg = pi_2_pi(Pg);
	// Yg = pi_2_pi(Yg);

	H_transform_R(1,1) = cos(-Rg);
	H_transform_R(1,2) = -sin(-Rg);
	H_transform_R(2,1) = sin(-Rg);
	H_transform_R(2,2) = cos(-Rg);
	
	H_transform_P(0,0) = cos(-Pg);
	H_transform_P(0,2) = sin(-Pg);
	H_transform_P(2,0) = -sin(-Pg);
	H_transform_P(2,2) = cos(-Pg);
	
	H_transform_Y(0,0) = cos(-Yg);
	H_transform_Y(0,1) = -sin(-Yg);
	H_transform_Y(1,0) = sin(-Yg);
	H_transform_Y(1,1) = cos(-Yg);

	dRg = Rg - tmp_Rg;
	dPg = Pg - tmp_Pg;
	dYg = Yg - tmp_Yg;
	
	/* dRg *= -1; */
	/* dPg *= -1; */
	/* dYg *= -1; */

	/* dRg = pi_2_pi(dRg); */
	/* dPg = pi_2_pi(dPg); */
	/* dYg = pi_2_pi(dYg); */
	
	H_transform_dR(1,1) = cos(-dRg);
	H_transform_dR(1,2) = -sin(-dRg);
	H_transform_dR(2,1) = sin(-dRg);
	H_transform_dR(2,2) = cos(-dRg);
	
	H_transform_dP(0,0) = cos(-dPg);
	H_transform_dP(0,2) = sin(-dPg);
	H_transform_dP(2,0) = -sin(-dPg);
	H_transform_dP(2,2) = cos(-dPg);
	
	H_transform_dY(0,0) = cos(-dYg);
	H_transform_dY(0,1) = -sin(-dYg);
	H_transform_dY(1,0) = sin(-dYg);
	H_transform_dY(1,1) = cos(-dYg);

	

	V_transform_dXYZ = H_transform_Y * H_transform_P * H_transform_R * V_dXYZ;
	std::cout << V_transform_dXYZ << std::endl;
	for(size_t i=0;i<3;i++){
		dH_transform_local(i,3) = V_transform_dXYZ(i);
	}
	
	H_rotation_d = H_transform_dY * H_transform_dP * H_transform_dR;
	for(size_t i=0;i<3;i++){
		for(size_t j=0;j<3;j++){
			dH_transform_local(i,j) = H_rotation_d(i,j);
		}
	}
	
	tmp_odom_xg = odom_xg;
	tmp_odom_yg = odom_yg;
	tmp_odom_zg = odom_zg;
	tmp_Rg = Rg;
	tmp_Pg = Pg;
	tmp_Yg = Yg;

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


void Nagayne::translation(void)
{
	for(auto& pt : transformed_pc_->points){
		pt.x -= odom_dxg;
		pt.y -= odom_dyg;
		pt.z -= odom_dzg;
	}
}


double Nagayne::pi_2_pi(double angle)
{
	if(angle > M_PI){
		return -M_PI + (angle - M_PI);
	}else{
		return angle;
	}
}
