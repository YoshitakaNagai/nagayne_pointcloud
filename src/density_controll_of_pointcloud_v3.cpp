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
CloudIUTPtr output_save_pc (new CloudIUT);
CloudIUTPtr single_pc_(new CloudIUT);
CloudIUTPtr save_pc_ (new CloudIUT);
CloudIUTPtr output_pc_after (new CloudIUT);
CloudIUTPtr output_pc (new CloudIUT);
CloudIUTPtr old_pc_ (new CloudIUT);
CloudIUTPtr veteran_pc_ (new CloudIUT);

// class PointXYZIUTS : public pcl::PointXYZHSV
// {
// public:
// 	float score;
// };
//
// typedef PointXYZIUTS PointIUTS;
// typedef pcl::PointCloud<PointIUTS> CloudIUTS;
// typedef pcl::PointCloud<PointIUTS>::Ptr CloudIUTSPtr;
//CloudIUTSPtr downsampled_pc_ (new CloudIUTS);

nav_msgs::Odometry odom_;
nav_msgs::Odometry init_odom_;
nav_msgs::Odometry sq_time;

ros::Publisher pub;
ros::Publisher DCP_pub;

Eigen::Matrix4f transform_matrix;
Eigen::Matrix4f inverse_transform_matrix;

double a, b, c, d;
double KP = 1;
double KI = 0;
double KD = 0;

size_t extra_size;
size_t target_size;
size_t histogram_save_num;

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

/*
void calc_score_histogram(int score){
	histogram[score] += 1;
}


int calc_border_score(){
	int histogram_count = 0;
	int border = 0;
	while(histogram_count < extra_size){
		histogram_count += histogram[border];
		border++;
	}
	return border;
}
*/


void pt_lifespan_keeper(CloudIUTPtr vpc){
	float dt = ros::Time::now().toSec() - tmp_time;
	for(auto& pt : vpc->points){
		pt.v -= dt;
	}
}


void scorekeeper(CloudIUTPtr before_scored_pc)
{
	float now_time = ros::Time::now().toSec();
	//float operating_time = now_time - begin_time;
	//#pragma omp parallel for	
	for(auto& pt : before_scored_pc->points){
		float unflatness_score = resolution*pt.s/2;
		//float time_score = resolution*(pt.v - begin_time)/operating_time;
		//float score = (unflatness_score + time_score)/2;
		float score = (float)a*unflatness_score;
		//calc_score_histogram(score);
		pt.v = score;
	}
}


CloudIUTPtr pc_downsampling(CloudIUTPtr after_scored_pc){
	CloudIUTPtr pc_filterd (new CloudIUT);
	pcl::PassThrough<PointIUT> pass;
	pass.setInputCloud(after_scored_pc);
	pass.setFilterFieldName ("v");
	pass.setFilterLimits (0, 1.0);
	pass.filter(*pc_filterd);
	/*
	for(auto& h : histogram){
		h = 0;
	}
	*/
	return pc_filterd;
}

void controll_density(){
	CloudIUTPtr downsampled_pc_ (new CloudIUT);
	
	scorekeeper(output_save_pc);
	*veteran_pc_ += *output_save_pc;
	pt_lifespan_keeper(veteran_pc_);
	downsampled_pc_ = pc_downsampling(veteran_pc_);
	
	sensor_msgs::PointCloud2 pc_;
	pcl::toROSMsg(*downsampled_pc_, pc_);
	pc_.header.stamp = ros::Time::now();
	pc_.header.frame_id = veteran_pc_->header.frame_id;
	DCP_pub.publish(pc_);
}

void veteran_pc_callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
	pcl::fromROSMsg(*msg, *veteran_pc_);
	veteran_pc_callback_flag = true;
	//std::cout << "density controlled" << std::endl;
}



void fresh_pc_callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    pcl::fromROSMsg(*msg, *single_pc_);
    if(!first_flag){
		pub.publish(*msg);
		begin_time = single_pc_->points[0].v;
		tmp_time = ros::Time::now().toSec();
		first_flag = true;
	}
	
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
	fresh_pc_callback_flag = true;
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

	/*
	for(auto& h : histogram){
		h = 0;
	}
	*/

	ros::Rate r(100);
	while(ros::ok()){	
		if(fresh_pc_callback_flag && veteran_pc_callback_flag){
			std::cout<<"start"<<std::endl;
			controll_density();
			fresh_pc_callback_flag = false;
			veteran_pc_callback_flag = false;
		}
		r.sleep();
		ros::spinOnce();
	}
}
