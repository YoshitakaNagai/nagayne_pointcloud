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
CloudIUTPtr pc_iut_ (new CloudIUT);
CloudIUTPtr veteran_pc_ (new CloudIUT);
CloudIUTPtr output_save_pc_ (new CloudIUT);

nav_msgs::Odometry odom_;
nav_msgs::Odometry init_odom_;
nav_msgs::Odometry sq_time;

ros::Publisher pub;
ros::Publisher DCP_pub;

Eigen::Matrix4f transform_matrix;
Eigen::Matrix4f inverse_transform_matrix;

double a, b, c, d;

size_t extra_size;

const int resolution = 1000;
int histogram[resolution];
int save_num;
int scan_num;

bool init_lcl_flag = false;
bool save_flag = false;
bool fresh_pc_callback_flag = false;
bool veteran_pc_callback_flag = false;
bool first_flag = false;

float z_threshold = 30.0;
float begin_time;
float tmp_time;

Eigen::Matrix4f create_matrix(nav_msgs::Odometry odom_now, float reflect){

    double roll_now, pitch_now, yaw_now;

    tf::Quaternion q_now(odom_now.pose.pose.orientation.x, odom_now.pose.pose.orientation.y, odom_now.pose.pose.orientation.z, odom_now.pose.pose.orientation.w);
    tf::Matrix3x3(q_now).getRPY(roll_now, pitch_now, yaw_now);

    Eigen::Translation3f init_translation(reflect*odom_now.pose.pose.position.x, reflect*odom_now.pose.pose.position.y, reflect*odom_now.pose.pose.position.z);
    Eigen::AngleAxisf init_rotation_x(reflect*roll_now, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(reflect*pitch_now, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(reflect*yaw_now, Eigen::Vector3f::UnitZ());

    Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

    return init_guess;
}

void lcl_callback(nav_msgs::Odometry msg){
    if(init_lcl_flag){
        init_odom_ = msg;
        init_lcl_flag = false;
    }
    odom_ = msg;
    transform_matrix = create_matrix(odom_, 1.0);
}


CloudIUTPtr pt_lifespan_keeper(CloudIUTPtr vpc){
	float beginning_time = ros::Time::now().toSec();
	float dt = 0.01;
	for(auto& pt : vpc->points){
		pt.v -= dt;
	}
	tmp_time = beginning_time;
	std::cout << "tmp_time : " << std::endl;
	return vpc;
}


CloudIUTPtr scorekeeper(CloudIUTPtr before_scored_pc_)
{
	for(auto& pt : before_scored_pc_->points){
		float unflatness_score = pt.s/2;
		float score = (float)a*unflatness_score + c;
		pt.v = score;
		//pt.v = (float)b*sqrt(pow(pt.x,2) + pow(pt.y,2) + pow(pt.z,2))/30.0 + c;
	}

	return before_scored_pc_;
}


CloudIUTPtr pc_downsampling(CloudIUTPtr after_scored_pc){
	CloudIUTPtr pc_filterd (new CloudIUT);
	pcl::PassThrough<PointIUT> pass;
	pass.setInputCloud(after_scored_pc);
	pass.setFilterFieldName ("v");
	pass.setFilterLimits (0, a + c);
	//pass.setFilterLimits (0, b + c);
	pass.filter(*pc_filterd);
	return pc_filterd;
}

void controll_density(CloudIUTPtr fresh_pc_){
	fresh_pc_ = scorekeeper(fresh_pc_);
	*veteran_pc_ += *fresh_pc_;
	veteran_pc_ = pt_lifespan_keeper(veteran_pc_);
	veteran_pc_ = pc_downsampling(veteran_pc_);
	
	sensor_msgs::PointCloud2 pc_;
	pcl::toROSMsg(*veteran_pc_, pc_);
	pc_.header.stamp = ros::Time::now();
	pc_.header.frame_id = veteran_pc_->header.frame_id;
	DCP_pub.publish(pc_);
}

void fresh_pc_callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
	CloudIUTPtr single_pc_(new CloudIUT);
	CloudIUTPtr output_pc_after (new CloudIUT);
	CloudIUTPtr output_pc (new CloudIUT);

	pcl::fromROSMsg(*msg, *single_pc_);
    
	if(!first_flag){
		*veteran_pc_ = *single_pc_;		
		first_flag = true;
	}
	
	pcl::transformPointCloud(*single_pc_, *output_pc, transform_matrix);
    for(size_t i=0;i<single_pc_->points.size();i++){
        double distance = sqrt(pow(single_pc_->points[i].x, 2)+
                pow(single_pc_->points[i].y, 2)+
                pow(single_pc_->points[i].z, 2));
        if(distance < 50){
            output_pc_after->points.push_back(output_pc->points[i]);
        }
    }
	

    Eigen::Matrix4f inverse_transform_matrix = transform_matrix.inverse();
    pcl::transformPointCloud(*output_pc_after, *output_save_pc_, inverse_transform_matrix);

	controll_density(output_save_pc_);

	extra_size = output_save_pc_->points.size();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "density_controll_of_pointcloud_v3");
    ros::NodeHandle n;
	ros::NodeHandle nh("~");
	nh.getParam("save_num", save_num);
	nh.getParam("scan_num", scan_num);
	nh.getParam("a", a);
	nh.getParam("b", b);
	nh.getParam("c", c);
	nh.getParam("d", d);

    ros::Subscriber sub_pc = n.subscribe("/nagayne_PointCloud2/fusioned", 10, fresh_pc_callback);
    ros::Subscriber sub_lcl = n.subscribe("/odom", 10, lcl_callback);
	DCP_pub = n.advertise<sensor_msgs::PointCloud2>("/nagayne_PointCloud2/density_controlled", 10);

    nav_msgs::Odometry init_odom;
    init_odom.header.frame_id = "/map";
    init_odom.child_frame_id = "/base_link";
    init_odom.pose.pose.position.x = 0.0;
    init_odom.pose.pose.position.y = 0.0;
    init_odom.pose.pose.position.z = 0.0;
    init_odom.pose.pose.orientation.x = 0.0;
    init_odom.pose.pose.orientation.y = 0.0;
    init_odom.pose.pose.orientation.z = 0.0;
    init_odom.pose.pose.orientation.w = 0.0;

    odom_ = init_odom;

	std::cout<<"start"<<std::endl;
	
	ros::spin();
}
