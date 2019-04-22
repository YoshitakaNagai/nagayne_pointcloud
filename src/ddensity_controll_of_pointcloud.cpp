/*
 *	density controller of nagayne_pointcloud
 *
 * 	author : Yoshitaka Nagai
 */

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

#ifdef _OPENMP
#include <omp.h>
#endif



typedef pcl::PointXYZRGBNormal PointIUTS;
typedef pcl::PointCloud<PointIUTS> CloudIUTS;
typedef pcl::PointCloud<PointIUTS>::Ptr CloudIUTSPtr;

CloudIUTSPtr single_pc_(new CloudIUTS);
CloudIUTSPtr save_pc_ (new CloudIUTS);
CloudIUTSPtr output_save_pc (new CloudIUTS);
CloudIUTSPtr output_pc_after (new CloudIUTS);
CloudIUTSPtr output_pc (new CloudIUTS);
CloudIUTSPtr old_pc_ (new CloudIUTS);
CloudIUTSPtr veteran_pc_ (new CloudIUTS);
CloudIUTSPtr downsampled_pc_ (new CloudIUTS);

nav_msgs::Odometry odom_;
nav_msgs::Odometry init_odom_;

nav_msgs::Odometry sq_time;

ros::Publisher pub;
ros::Publisher DCP_pub;


Eigen::Matrix4f transform_matrix;
Eigen::Matrix4f inverse_transform_matrix;


std::vector<float> score;

double a, b, c, d;

size_t extra_size = 0;

int count_ = 0;
int save_num;
int scan_num;

bool init_lcl_flag = false;
bool save_flag = false;
bool pc_callback_flag = false;
bool veteran_pc_callback_flag = false;

float z_threshold = 30.0;

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

CloudIUTS scorekeeper(CloudIUTS before_scored_pc)
{
	size_t pc_size = before_scored_pc.points.size();
	for(size_t i=0;i<pc_size;i++){
		//r = intensity, g = unflatness, b = time, curvature = score;
		float unflatness_score = pow((float)a*before_scored_pc.points[i].g, (float)b);
		float operating_time = before_scored_pc.header.stamp - before_scored_pc.points[i].b;
		float operating_time_score = pow((float)c*operating_time, (float)d);
		before_scored_pc.points[i].curvature = unflatness_score - operating_time_score;
	}
	return before_scored_pc;
}

CloudIUTS pc_downsampling(CloudIUTS after_scored_pc){
	size_t pc_size = after_scored_pc.points.size();
	for(size_t i=0;i<pc_size;i++){
		score.push_back(after_scored_pc.points[i].curvature);
	}
	sort(score.begin(), score.end());
	float border_score = score.at(extra_size);

	for(size_t i=0;i<pc_size;i++){
		//r = intensity, g = unflatness, b = time, curvature = score;
		if(after_scored_pc.points[i].curvature < border_score){
			after_scored_pc.points.erase(after_scored_pc.points.begin() + i);
		}
	}
	return after_scored_pc;
}

void controll_density(){
	*veteran_pc_ += *output_save_pc;
	*veteran_pc_ = scorekeeper(*veteran_pc_);
	*downsampled_pc_ = pc_downsampling(*veteran_pc_);
	sensor_msgs::PointCloud2 pc_;
	pcl::toROSMsg(*downsampled_pc_, pc_);
	pc_.header.stamp = ros::Time::now();
	//pc_.header.frame_id = msg->header.frame_id;
	DCP_pub.publish(pc_);
}

void veteran_pc_callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
	pcl::fromROSMsg(*msg, *veteran_pc_);
	veteran_pc_callback_flag = true;
}


void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    pcl::fromROSMsg(*msg, *single_pc_);
    pcl::transformPointCloud(*single_pc_, *output_pc, transform_matrix);

    for(size_t i=0;i<single_pc_->points.size();i++){
        double distance = sqrt(pow(single_pc_->points[i].x, 2)+
                pow(single_pc_->points[i].y, 2)+
                pow(single_pc_->points[i].z, 2));
        if(distance < 50){	//調整可
            //if(single_pc_->points[i].z <= z_threshold){
            PointIUTS temp;
            output_pc_after->points.push_back(output_pc->points[i]);
        }
    }

    Eigen::Matrix4f inverse_transform_matrix = transform_matrix.inverse();
    pcl::transformPointCloud(*output_pc_after, *output_save_pc, inverse_transform_matrix);

	extra_size = output_save_pc->points.size();

	pc_callback_flag = true;

	if(!save_flag){
		if(count_ < save_num){
			*save_pc_ += *output_save_pc;//sadakuni
			//*save_pc_ += *output_pc_after;//test_nagai
			old_pc_ = output_save_pc;
		}else{
			int old_pc_size = (int)old_pc_->points.size();
			save_pc_->points.erase(save_pc_->points.begin(), save_pc_->points.begin()+old_pc_size);
			*save_pc_ += *output_save_pc;
			old_pc_ = output_save_pc;
			
			save_flag = true;
		}
	
		sensor_msgs::PointCloud2 pc_;
		pcl::toROSMsg(*save_pc_, pc_);
		pc_.header.stamp = ros::Time::now();
		pc_.header.frame_id = msg->header.frame_id;
		// pc_.header.frame_id = "/odom3d";	//変更
		pub.publish(pc_);
		count_++;
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sq_lcl");
    ros::NodeHandle n;
	ros::NodeHandle nh("~");
	nh.getParam("save_num", save_num);
	nh.getParam("scan_num", scan_num);
	nh.getParam("a", scan_num);
	nh.getParam("b", scan_num);
	nh.getParam("c", scan_num);
	nh.getParam("d", scan_num);

    ros::Subscriber sub_pc = n.subscribe("/nagayne_PointCloud2/fusioned", 30, pc_callback);
    ros::Subscriber sub_lcl = n.subscribe("/odom", 30, lcl_callback);
	ros::Subscriber sub_veteran_pc = n.subscribe("/nagayne_PointCloud2/density_controlled", 30, veteran_pc_callback);

	pub = n.advertise<sensor_msgs::PointCloud2>("/nagayne_PointCloud2/density_controlled", 30);
	DCP_pub = n.advertise<sensor_msgs::PointCloud2>("/nagayne_PointCloud2/density_controlled", 30);

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
	
	ros::Rate r(100);
	while(ros::ok()){	
		if(save_flag && pc_callback_flag && veteran_pc_callback_flag){
			controll_density();
			pc_callback_flag = false;
			veteran_pc_callback_flag = false;
		}
		r.sleep();
		ros::spinOnce();
	}
}
