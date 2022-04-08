#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <std_msgs/String.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/TransformStamped.h>

#include <visualization_msgs/Marker.h>

struct Point
{
	double x;
	double y;
	double z;
	double ox;
	double oy;
	double oz;
	double ow;
};

int main(int argc, char **argv)
{
	// ROSの初期化
	ros::init(argc, argv, "wayp_auto");
	ros::NodeHandle node;

	// パラメータの設定
	ros::NodeHandle ph("~");
	std::string read_file;
	double point_distance;
	double deg_thresh;
	double deg_distance;
	ph.param("read_file", read_file, std::string("/home/robo/つくば練習中/way_point/practice_waypoints.txt"));
	ph.param("point_distance", point_distance, 4.0);
	ph.param("deg_thresh", deg_thresh, 15.0);
	ph.param("deg_distance", deg_distance, 1.0);

	// 読み込んだファイルがあるか確認する
	std::ofstream ofs;
	ofs.open(read_file.c_str() , std::ios::out);
	if (ofs.fail())
	{
		std::cerr << "waypointのファイルが見つかりません" << std::endl;
		return -1;
	}

	//変数の宣言
	int way_count = 0;
	std::vector<Point> waypoint;
	double roll_base, pitch_base, yaw_base;

	// tfのListenerとTransformの作成
	tf::TransformListener listener;
	tf::StampedTransform transform;

	// publisherの作成
	ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("/waypoint_marker", 100);

	// whileの回る周期の設定
	ros::Rate rate(10);

	while(ros::ok())
	{
		// tfを利用して /map上の座標を取得する
		while(ros::ok())
		{
			try
			{
				printf("get...\n");
				listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

			}
			catch (tf::TransformException &ex)
			{
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
				continue;
			}

			break;
		}

		// tfの値を一時保存の変数に格納しておく
		Point tmp_point;
		tmp_point.x = transform.getOrigin().x();
		tmp_point.y = transform.getOrigin().y();
		tmp_point.z = transform.getOrigin().z();
		tmp_point.ox = transform.getRotation().x();
		tmp_point.oy = transform.getRotation().y();
		tmp_point.oz = transform.getRotation().z();
		tmp_point.ow = transform.getRotation().w();
		///co.IN_POINT(tmp_point.x,tmp_point.y,tmp_point.z,tmp_point.ox,tmp_point.oy,tmp_point.oz,tmp_point.ow,way_count);

		// 最初の例外処理
		if(way_count == 0)
		{
			tf::Quaternion q_base(tmp_point.ox, tmp_point.oy, tmp_point.oz, tmp_point.ow);
			tf::Matrix3x3 m_base(q_base);
			m_base.getRPY(roll_base, pitch_base, yaw_base);

			waypoint.push_back(tmp_point);
			way_count++;
			continue;
		}

		double diff_way = std::sqrt(std::pow((tmp_point.x - waypoint[way_count-1].x), 2.0) + std::pow((tmp_point.y - waypoint[way_count-1].y), 2.0));

		tf::Quaternion q_now(tmp_point.ox, tmp_point.oy, tmp_point.oz, tmp_point.ow);
		tf::Matrix3x3 m_now(q_now);
		double roll_now, pitch_now, yaw_now;
		m_now.getRPY(roll_now, pitch_now, yaw_now);

		double diff_yaw = yaw_now - yaw_base;
		diff_yaw = std::abs(diff_yaw);
		std::cout << "No. " << way_count << " " << "yaw_base " << yaw_base * 180.0 / M_PI << " yaw_now " << yaw_now * 180.0 / M_PI << " diff_yaw " << diff_yaw * 180.0 / M_PI << " diff_way " << diff_way << std::endl;
        if((diff_yaw * 180.0 / M_PI > deg_thresh && diff_way > deg_distance) || diff_way > point_distance)
		{
			yaw_base = yaw_now;
			waypoint.push_back(tmp_point);
			way_count++;
		}
		// publishするマーカの設定
		visualization_msgs::Marker points;
		points.header.frame_id = "/map";
		points.header.stamp = ros::Time::now();
		points.ns = "POINTS";
		points.color.b = 1.0;
		points.color.a = 1.0;
		points.scale.x = 0.3;
		points.scale.y = 0.3;
		points.scale.z = 0.3;
		points.action = visualization_msgs::Marker::ADD;
		points.type = visualization_msgs::Marker::POINTS;
		points.pose.orientation.w = 1.0;

		// publishするマーカの座標をwaypointから取得する
		for(int i = 1; i < waypoint.size(); i++)
		{
			geometry_msgs::Point p;
			p.x = waypoint[i].x;
			p.y = waypoint[i].y;
			p.z = 0.25;
			points.points.push_back(p);
		}

		// マーカのパブリッシュ
		marker_pub.publish(points);

		ros::spinOnce();
		rate.sleep();
	}

	std::cout << "Finiiiiiiiiiiiiiiiiiiiiiiish!" << std::endl;

	// ファイル書き込み i=0は最初の地点なので無視する
	for(int i = 1; i < waypoint.size(); i++)
	{
		ofs << waypoint[i].x <<","<< waypoint[i].y << "," << waypoint[i].z << ","
		<< waypoint[i].ox << "," << waypoint[i].oy << "," << waypoint[i].oz << ","
		<< waypoint[i].ow << std::endl;
	}

	ofs.close();

	return 0;
}

