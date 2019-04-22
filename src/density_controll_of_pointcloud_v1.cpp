/*
 *	density controller of nagayne_pointcloud
 *
 * 	author : Yoshitaka Nagai
 */



#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLHeader.h>
#include <std_msgs/Header.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
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

#include <vector>
#include <array>
#include <algorithm>

#include "nagayne_pointcloud/point_id.h"


typedef pcl::PointXYZI PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr; //ポインタ宣言のpointcloud
std::vector<CloudAPtr> CloudPtrList;
std::vector<Eigen::Vector2f> tmpTimeList;
std::vector<float> score;


CloudAPtr save_pc_ (new CloudA); // ポインタのpointcloud宣言であるため，動的メモリ領域の解放をする必要なし
CloudAPtr old_pc_ (new CloudA);
CloudAPtr tmp_pc (new CloudA);
CloudAPtr tmp_pc_before_downsampling (new CloudA);
CloudAPtr tmp_1pc_after_downsampling (new CloudA);

nav_msgs::Odometry odom_;
nav_msgs::Odometry init_odom_;

nav_msgs::Odometry sq_time;

ros::Publisher pub;
//ros::Publisher array_pub;
ros::Publisher pub_sq_time;

Eigen::Matrix4f transform_matrix;
Eigen::Matrix4f inverse_transform_matrix;


size_t CloudPtrList_size;	

int count_ = 0;
int save_num;
int scan_num;

bool init_lcl_flag = false;
bool first_flag = false;

float tmp_time, before_time, after_time, operating_time;
float z_threshold = 30.0;
float begin_time;
double a, b, c, d;

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


float timekeeper(Eigen::Vector2f time_vec_i, Eigen::Vector2f time_vec_last){
	return time_vec_last(0,0) - time_vec_i(1,0);
}

void calc_score(size_t list_size){
	for(size_t i=0;i<list_size;i++){
		*tmp_pc = *CloudPtrList.at(i);
		operating_time = timekeeper(tmpTimeList.at(i), tmpTimeList.at(list_size-1));
		float time_score = pow((float)a*operating_time, (float)b);
		size_t tmp_pc_size = (size_t)tmp_pc->points.size();
		
		for(size_t j=0;j<tmp_pc_size;j++){
			float unflatness_score = pow((float)c*tmp_pc->points[j].intensity, (float)d);
			tmp_pc->points[j].intensity = unflatness_score - time_score;
		}
		CloudPtrList.at(i) = tmp_pc;
	}
}


void pointcloud_downsampler(size_t list_size, size_t extra_size){
	for(size_t i=0;i<list_size;i++){
		*tmp_pc_before_downsampling += *CloudPtrList.at(i);
	}
	size_t tmp_ds_pc_size = tmp_pc_before_downsampling->points.size();
	//array<float,tmp_ds_pc_size> score;
	for(size_t i=0;i<tmp_ds_pc_size;i++){
		score.push_back(tmp_pc_before_downsampling->points[i].intensity);
	}
	sort(score.begin(), score.end());
	float border_score = score.at(extra_size);
	
	for(size_t i=0;i<list_size;i++){
		*tmp_1pc_after_downsampling = *CloudPtrList.at(i);
		size_t pc_after_size = tmp_1pc_after_downsampling->points.size();
		for(size_t j=0;j<pc_after_size;j++){
			if(border_score > tmp_1pc_after_downsampling->points[i].intensity && j > 0){
				/* tmp_1pc_after_downsampling->points.erase(tmp_1pc_after_downsampling->points.begin()+j-1,tmp_1pc_after_downsampling->points.begin()+j);	 */
				tmp_1pc_after_downsampling->points.erase(tmp_1pc_after_downsampling->points.begin()+j-1, tmp_1pc_after_downsampling->points.begin()+j);	
			}
		}
		*CloudPtrList.at(i) = *tmp_1pc_after_downsampling;
	}
}


void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    CloudAPtr single_pc_(new CloudA);
    pcl::fromROSMsg(*msg, *single_pc_);

    CloudAPtr output_pc (new CloudA);
    pcl::transformPointCloud(*single_pc_, *output_pc, transform_matrix);

    CloudAPtr output_pc_after (new CloudA);

    for(size_t i=0;i<single_pc_->points.size();i++){
        double distance = sqrt(pow(single_pc_->points[i].x, 2)+
                pow(single_pc_->points[i].y, 2)+
                pow(single_pc_->points[i].z, 2));
        if(distance < 50){	//調整可 移動量を加味した上で
            //if(single_pc_->points[i].z <= z_threshold){
            	PointA temp;
            	output_pc_after->points.push_back(output_pc->points[i]);
			//}
		}
    }

    CloudAPtr output_save_pc (new CloudA);
    Eigen::Matrix4f inverse_transform_matrix = transform_matrix.inverse();
    pcl::transformPointCloud(*output_pc_after, *output_save_pc, inverse_transform_matrix);  

	if(!first_flag){
		before_time = begin_time;
		after_time = msg->header.stamp.toSec();

	}else{
		before_time = tmp_time;
		after_time = msg->header.stamp.toSec();
	}

	
	Eigen::Vector2f tmpTime_vec(before_time,after_time);


	size_t extra_pc_size = output_save_pc->points.size();

	if(count_ < save_num){
		
		CloudPtrList.push_back(output_save_pc);
		tmpTimeList.push_back(tmpTime_vec);
		//old_pc_ = output_save_pc;
		
		CloudPtrList_size = CloudPtrList.size();		
		
		calc_score(CloudPtrList_size);
		
		if(count_ > scan_num){
			pointcloud_downsampler(CloudPtrList_size, extra_pc_size);
		}
		count_++;
    }else{
        //int old_pc_size = (int)old_pc_->points.size();
        //save_pc_->points.erase(save_pc_->points.begin(), save_pc_->points.begin()+old_pc_size);
        //*save_pc_ += *output_save_pc;
		CloudPtrList.push_back(output_save_pc);
		CloudPtrList.erase(CloudPtrList.begin());
		tmpTimeList.push_back(tmpTime_vec);
		tmpTimeList.erase(tmpTimeList.begin());
		//old_pc_ = output_save_pc;
		CloudPtrList_size = CloudPtrList.size();	
		calc_score(CloudPtrList_size);
		pointcloud_downsampler(CloudPtrList_size, extra_pc_size);
	}

	tmp_time = after_time;
   

	for(size_t i=0;i<CloudPtrList_size;i++){
		*save_pc_ += *CloudPtrList.at(i);
	}

	sensor_msgs::PointCloud2 downsampled_PointCloud2;
    pcl::toROSMsg(*save_pc_, downsampled_PointCloud2);
    downsampled_PointCloud2.header.stamp = ros::Time::now();
    downsampled_PointCloud2.header.frame_id = msg->header.frame_id;
    pub.publish(downsampled_PointCloud2);
	std::cout << "pub" << std::endl;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "density_controll_of_pointcloud");
    ros::NodeHandle n;
	ros::NodeHandle nh("~");
	nh.getParam("save_num", save_num);
	nh.getParam("scan_num", save_num);
	nh.getParam("a", a);
	nh.getParam("b", b);
	nh.getParam("c", c);
	nh.getParam("d", d);

	begin_time = ros::Time::now().toSec();

    ros::Subscriber sub_lcl = n.subscribe("/odom", 10, lcl_callback);
    ros::Subscriber sub_pc = n.subscribe("/nagayne_PointCloud2/from_LaserScan", 10, pc_callback);
    pub = n.advertise<sensor_msgs::PointCloud2>("/nagayne_PointCloud2/density_controlled", 10);
    //array_pub = n.advertise<std_msgs::Float32MultiArray>("/nagayne_pointcloud/array", 100);
    //pcl_pub = n.advertise<CloudA>("/nagayne_pointcloud/pcl", 100);

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
    
	return 0;
}
