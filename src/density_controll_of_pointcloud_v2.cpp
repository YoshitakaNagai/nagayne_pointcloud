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
#include <pcl/filters/approximate_voxel_grid.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif


typedef pcl::PointXYZINormal PointINormal;
typedef pcl::PointCloud<PointINormal> CloudINormal;
typedef pcl::PointCloud<PointINormal>::Ptr CloudINormalPtr;

typedef pcl::PointXYZHSV PointIUT;
typedef pcl::PointCloud<PointIUT> CloudIUT;
typedef pcl::PointCloud<PointIUT>::Ptr CloudIUTPtr;
CloudIUTPtr pc_iut_ (new CloudIUT);
CloudIUTPtr output_save_pc (new CloudIUT);
CloudIUTPtr single_pc_(new CloudIUT);
CloudIUTPtr save_pc_ (new CloudIUT);
CloudIUTPtr output_pc_after (new CloudIUT);
CloudIUTPtr output_pc (new CloudIUT);
CloudIUTPtr old_pc_ (new CloudIUT);

class PointXYZIUTS : public pcl::PointXYZHSV
{
public:
	float score;
};

typedef PointXYZIUTS PointIUTS;
typedef pcl::PointCloud<PointIUTS> CloudIUTS;
typedef pcl::PointCloud<PointIUTS>::Ptr CloudIUTSPtr;



CloudIUTSPtr downsampled_pc_ (new CloudIUTS);

nav_msgs::Odometry odom_;
nav_msgs::Odometry init_odom_;

nav_msgs::Odometry sq_time;

ros::Publisher pub;
ros::Publisher DCP_pub;


Eigen::Matrix4f transform_matrix;
Eigen::Matrix4f inverse_transform_matrix;

std::vector<float> score_list;


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


//std::map<CloudIUTPtr, float> scorekeeper(CloudIUTPtr before_scored_pc)
CloudIUTSPtr scorekeeper(CloudIUTSPtr before_scored_pc)
{
	size_t i = 0;
	//#pragma omp parallel for	
	for(auto& pt : before_scored_pc->points){
	//for(size_t i=0;i<pc_size;i++){
		float unflatness_score = pow((float)a*pt.s, (float)b);
		//float unflatness_score = pow((float)a*before_scored_pc->points[i].s, (float)b);
		float operating_time = ros::Time::now().toSec() - pt.v;
		//float operating_time = ros::Time::now().toSec() - before_scored_pc->points[i].v;
		float operating_time_score = - pow((float)c*operating_time, (float)d);
		float score = unflatness_score + operating_time_score;
		std::cout << "score = " << score << std::endl;
		pt.score = unflatness_score + operating_time_score;
		//score_map.insert(std::make_pair(pt, score));
		score_list.push_back(score);
		if(i > extra_size){
			sort(score_list.begin(), score_list.end());
			score_list.pop_back();
		}
		i++;
	}
	
	return before_scored_pc;
}


//CloudIUTPtr pc_downsampling(CloudIUTPtr after_scored_pc, std::vector<size_t> sorted_score_id){
CloudIUTSPtr pc_downsampling(CloudIUTSPtr after_scored_pc){
	//size_t pc_size = after_scored_pc->points.size();
	float border_score = score_list.back();
	size_t i = 0;
	
	for(auto& pt : after_scored_pc->points){
		if(pt.score <= border_score){
			after_scored_pc->points.erase(after_scored_pc->points.begin() + i -1);
		}
		i++;
		if(i > extra_size){
			break;
		}
	}
	
	return after_scored_pc;
}

CloudIUTSPtr pc_evolution(CloudIUTPtr raw_pc_){
	CloudIUTSPtr evolution_pc_ (new CloudIUTS);
	size_t pc_size = raw_pc_->points.size();
	for(size_t i=0;i<pc_size;i++){
		std::cout << "evolutioning" << std::endl;
		evolution_pc_->points[i].x += raw_pc_->points[i].x;
		evolution_pc_->points[i].y += raw_pc_->points[i].y;
		evolution_pc_->points[i].z += raw_pc_->points[i].z;
		evolution_pc_->points[i].h += raw_pc_->points[i].h;
		evolution_pc_->points[i].s += raw_pc_->points[i].s;
		evolution_pc_->points[i].v += raw_pc_->points[i].v;
		evolution_pc_->points[i].score += 0;
	}
	return evolution_pc_;
}

CloudIUTPtr pc_degeneration(CloudIUTSPtr evo_pc){
	CloudIUTPtr degeneration_pc (new CloudIUT);
	size_t i = 0;
	for(auto& pt: evo_pc->points){
		std::cout << "degenerationning" << std::endl;
		degeneration_pc->points[i].x += pt.x;
		degeneration_pc->points[i].y += pt.y;
		degeneration_pc->points[i].z += pt.z;
		degeneration_pc->points[i].h += pt.h;
		degeneration_pc->points[i].s += pt.s;
		degeneration_pc->points[i].v += pt.score;
		i++;
	}
	return degeneration_pc;
}

void controll_density(){
	CloudIUTPtr veteran_pc_ (new CloudIUT);
	CloudIUTSPtr evolutioned_pc_ (new CloudIUTS);
	CloudIUTPtr degenerationed_pc_ (new CloudIUT);

	*veteran_pc_ += *output_save_pc;
	evolutioned_pc_ = pc_evolution(veteran_pc_);
	evolutioned_pc_ = scorekeeper(evolutioned_pc_);
	evolutioned_pc_ = pc_downsampling(evolutioned_pc_);
	degenerationed_pc_ = pc_degeneration(evolutioned_pc_);
	
	sensor_msgs::PointCloud2 pc_;
	pcl::toROSMsg(*degenerationed_pc_, pc_);
	pc_.header.stamp = ros::Time::now();
	//pc_.header.frame_id = msg->header.frame_id;
	DCP_pub.publish(pc_);
}

void veteran_pc_callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
	//pcl::fromROSMsg(*msg, *pc_iut_);
	pcl::fromROSMsg(*msg, *pc_iut_);
	veteran_pc_callback_flag = true;
	//std::cout << "density controlled" << std::endl;
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
	}

		count_++;
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

    ros::Subscriber sub_pc = n.subscribe("/nagayne_PointCloud2/fusioned", 10, pc_callback);
    ros::Subscriber sub_lcl = n.subscribe("/odom", 10, lcl_callback);
	ros::Subscriber sub_veteran_pc = n.subscribe("/nagayne_PointCloud2/density_controlled", 10, veteran_pc_callback);

	pub = n.advertise<sensor_msgs::PointCloud2>("/nagayne_PointCloud2/density_controlled", 10);
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



	ros::Rate r(100);
	while(ros::ok()){	
		if(save_flag && pc_callback_flag && veteran_pc_callback_flag){
			
			std::cout<<"start"<<std::endl;
			controll_density();
			pc_callback_flag = false;
			veteran_pc_callback_flag = false;
		}
		r.sleep();
		ros::spinOnce();
	}
}
