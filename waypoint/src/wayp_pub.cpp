#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>

struct Point
{
	double x;
	double y;
	double z;
	double ox;
	double oy;
	double oz;
	double ow;
    int mode;
};
int main(int argc, char **argv)
{
	// ROSの初期化
	ros::init(argc, argv, "waypoint_pub");
	ros::NodeHandle node;

	// パラメータの設定
	ros::NodeHandle ph("~");
	std::string read_file;
	ph.param("read_file", read_file, std::string("/home/useful/way.txt"));


	//変数の宣言
	std::vector<Point> waypoint;

	// publisherの作成
	ros::Publisher marker_pub = node.advertise<visualization_msgs::MarkerArray>("/waypoint_publisher", 100);

	// whileの回る周期の設定
	ros::Rate rate(10);


	// 読み込んだファイルがあるか確認する
	int ret;
	FILE *rfp = fopen(read_file.c_str(), "r");
	if (rfp == NULL)
	{
		ROS_ERROR("Don't exist file!");
		return -1;
	}

	Point tmp;
	while (ret = fscanf(rfp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d\n", &tmp.x, &tmp.y, &tmp.z, &tmp.ox, &tmp.oy, &tmp.oz, &tmp.ow, &tmp.mode) != EOF)
	{
		waypoint.push_back(tmp);
    }
	fclose(rfp);


	while(ros::ok())
	{
		// publishするマーカの設定
		visualization_msgs::MarkerArray array_arrow,array_text;
        visualization_msgs::Marker arrow,text;
		arrow.header.frame_id = text.header.frame_id = "/map";
		arrow.header.stamp = text.header.stamp = ros::Time::now();
		arrow.ns = "ARROWS";
        text.ns = "TEXT";
		arrow.color.b =text.color.b = 1.0;
		text.color.g = 1.0;
		text.color.r = 1.0;
		arrow.color.a =text.color.a = 1.0;
		arrow.scale.x = 1.0;
		arrow.scale.y = 0.2;
		arrow.scale.z =text.scale.z = 0.4;
		arrow.action =text.action = visualization_msgs::Marker::ADD;
		arrow.type = visualization_msgs::Marker::ARROW;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		// publishするマーカの座標をwaypointから取得する
		for(int i = 0; i < waypoint.size(); i++)
		{
			geometry_msgs::Pose p;
			p.position.x = waypoint[i].x;
			p.position.y = waypoint[i].y;
			p.position.z = 0.25;
            p.orientation.x=waypoint[i].ox;
            p.orientation.y=waypoint[i].oy;
            p.orientation.z=waypoint[i].oz;
            p.orientation.w=waypoint[i].ow;
			arrow.pose=text.pose=p;
			text.pose.position.z =0.5;
			arrow.id=text.id=i;
            std::string str;
            str="No."+std::to_string(i)+"-mode"+std::to_string(waypoint[i].mode);
            text.text = str;
			if(waypoint[i].mode==0){
				arrow.color.r=1.0;
			}else{
				arrow.color.r=0.0;
			}
            array_arrow.markers.push_back(arrow);
            array_text.markers.push_back(text);
		}
		// マーカのパブリッシュ
		marker_pub.publish(array_arrow);
        marker_pub.publish(array_text);
		rate.sleep();
	}

	return 0;
}
