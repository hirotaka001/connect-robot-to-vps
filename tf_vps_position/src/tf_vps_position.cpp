#include <iostream>
#include <regex>

#include <ros/ros.h>
#include "std_msgs/String.h"
// Marker
#include <visualization_msgs/Marker.h>
// tf2
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
// Odom
#include "nav_msgs/Odometry.h"
// json
#include "../include/tf_vps_position/picojson/picojson.h"
// Eigen
#include "../include/tf_vps_position/Eigen/Core"

#include "tf_vps_supporter.hpp"

// Publish方法
// 0 --- None 
// 1 --- thin out
// 2 --- odom
// 3 --- average
int g_mode = 0;
// VPSの結果が届いた回数
unsigned int g_vps_count = 0;
// odomより計算される、指定された距離移動した場合 true となる
bool g_odom_flag = true; 

double g_tolerance_sec = 1.0;

PrevPose g_prev_pose;

std::unique_ptr<Averager> g_vps_avger(new(Averager));
std::unique_ptr<OdomChecker> g_odom = nullptr;
ros::Publisher g_pub;

// VPSの結果をJSONでパースできるように整形する
std::string convertVpsResultToJson(const std_msgs::String::ConstPtr& msg) {
	
	std::string mod_res = std::regex_replace(msg->data, std::regex("\'"), "\"");
	std::string mod_res2 = std::regex_replace(mod_res, std::regex("/data/"), "");
	std::string mod_res3 = std::regex_replace(mod_res2, std::regex("None"), "0");
	return mod_res3;
}

picojson::value parseJson(std::string vps_result_json)
{
	picojson::value v;
	std::string err = picojson::parse(v, vps_result_json);
	if (! err.empty()) {
	  std::cout << "convert failed " << std::endl;
	}
	return v;
}

// set marker info for rviz
void addMarker(double px, double py, double pz, double qx, double qy, double qz, double qw) {
	visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.1;
    marker.pose.position.x = px;
    marker.pose.position.y = py;
    marker.pose.position.z = pz;
    marker.pose.orientation.x = qx;
    marker.pose.orientation.y = qy;
    marker.pose.orientation.z = qz;
    marker.pose.orientation.w = qw;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    g_pub.publish(marker);
}

// Callback for VPS result
void vpsCallback(const std_msgs::String::ConstPtr& msg){

	std::cout << "receive vps result!" << std::endl;
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer); // don't remove?
	static tf2_ros::TransformBroadcaster tfbroadcaster;

	geometry_msgs::TransformStamped transformStamped;
	ros::Time tmp_now = ros::Time::now();
	try{
		transformStamped = tfBuffer.lookupTransform("odom", "base_footprint", tmp_now, ros::Duration(0.2));
		//transformStamped = tfBuffer.lookupTransform("odom", "base_footprint", ros::Time(0), ros::Duration(0.2));
	}
	catch (tf2::TransformException &ex) {
		std::cout << "[ERROR] tf transform ";
		std::cout << ex.what() << std::endl;
		return;
	}

	// convert json style
	std::string vps_result_json = convertVpsResultToJson(msg);

	// parse json
	picojson::value v = parseJson(vps_result_json);
	
	// extract vps pose 
	std::map<std::string, picojson::value> data = v.get<picojson::object>();
	picojson::array& pos = data["position"].get<picojson::array>();
	picojson::array& quat = data["rotation"].get<picojson::array>(); // xyzw	
	double px = pos[0].get<double>();
	double py = pos[1].get<double>();
	double pz = pos[2].get<double>();
	double qx =  quat[0].get<double>();
	double qy =  quat[1].get<double>();
	double qz =  quat[2].get<double>();
	double qw =  quat[3].get<double>();

	// tmp 
	//py = 0.6;
	pz = 0.0;
	std::cout << px << ", " << py << ", " << pz << std::endl;


	if (g_mode == 1 && g_vps_count % 3 != 0) {
		std::cout << "skip current result(thin mode)." << std::endl;
		setPrevPose(px, py, pz, qx, qy, qz, qw, g_prev_pose);
	}

	if (g_mode == 2 && g_odom_flag == false) {
		std::cout << "skip current result(odom mode)." << std::endl;
		setPrevPose(px, py, pz, qx, qy, qz, qw, g_prev_pose);
	}

	if (g_mode == 3) {
		g_vps_avger->add(px, py, pz);
		VpsResult avg = g_vps_avger->avg();
		px = avg.x_;
		py = avg.y_;
		pz = avg.z_;
	}
	g_vps_count++;

	std::string map_id = data["map_id"].get<std::string>();
	std::string time_stamp_secs = data["time_stamp_secs"].get<std::string>();
	std::string time_stamp_nsecs = data["time_stamp_nsecs"].get<std::string>();
	
	//ros::Time requestTime = ros::Time(0);
	ros::Time requestTime = tmp_now;
	ros::Time vpsTime;
	vpsTime.sec = atoi(time_stamp_secs.c_str());
	vpsTime.nsec = atoi(time_stamp_nsecs.c_str());
	ros::Time posTimeNow = tmp_now;
	
	geometry_msgs::PoseStamped pose_odom_to_map;
	try {
		tf2::Quaternion vps_q(qx, qy, qz, qw);
		tf2::Transform vps_tf(vps_q, tf2::Vector3(px, py, pz));
		
		geometry_msgs::PoseStamped tmp_pose;
		tmp_pose.header.frame_id = "base_footprint";
		tmp_pose.header.stamp = requestTime;
		tf2::toMsg(vps_tf.inverse(), tmp_pose.pose);
 
		tfBuffer.transform(tmp_pose, pose_odom_to_map, "odom", ros::Duration(1));
	}
	catch(tf2::TransformException& e) {
		std::cout << "[ERROR] tf transform ";
		std::cout << e.what() << std::endl;
		return;
	}

	// save previous pose
	g_prev_pose.set(px, py, pz, qx, qy, qz, qw);

	tf2::Transform latest_tf_;
	tf2::convert(pose_odom_to_map.pose, latest_tf_);

	ros::Duration transform_tolerance;
	transform_tolerance.fromSec(g_tolerance_sec);
	//ros::Time transform_expiration = (vpsTime + transform_tolerance);
	ros::Time transform_expiration = (posTimeNow + transform_tolerance);
	geometry_msgs::TransformStamped tf_map_to_odom;
	tf_map_to_odom.header.frame_id = "map";
	tf_map_to_odom.header.stamp = transform_expiration;
	tf_map_to_odom.child_frame_id = "odom";
	tf2::convert(latest_tf_.inverse(), tf_map_to_odom.transform);

	std::cout << "publish time:" << posTimeNow << std::endl;
	std::cout << "publish tf:" << tf_map_to_odom << std::endl;

	// Publish tf(/map->/odom)
	tfbroadcaster.sendTransform(tf_map_to_odom);
	// Publish marker
	addMarker(px, py, pz, qx, qy, qz, qw);

	g_odom_flag = false;
}

// Callback for odometry
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

	double posx = msg->pose.pose.position.x;
	double posy = msg->pose.pose.position.y;
	double posz = msg->pose.pose.position.z;

	if (g_odom == nullptr) {
		g_odom = std::make_unique<OdomChecker>(posx, posy, posz);
	}
	else {
		double dist = g_odom->calcDistance(posx, posy, posz);
		if (dist > 0.1) { // m
			g_odom_flag = true;
			g_odom.reset();
		}
	}
}

int main(int argc, char** argv){

	ros::init(argc, argv, "tf_vps_position");
	ros::NodeHandle node;

	// rosparam より Publish モードを取得する
	node.getParam("tf_vps_position_mode", g_mode);
	node.getParam("tf_vps_position_tolerance", g_tolerance_sec);

	std::cout << "rosparam(tf_vps_position_mode):" << g_mode << std::endl;
	std::cout << "tolerance sec(tf_vps_position_tolerance):" << g_tolerance_sec << std::endl;

	g_pub = node.advertise<visualization_msgs::Marker>("vps_pose_marker", 1);
	ros::Subscriber sub = node.subscribe("/vps_pose", 1, &vpsCallback);
	ros::Subscriber sub2 = node.subscribe("/odom", 1, &odomCallback);
	
	ros::spin();
	return 0;
};
