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
#include <cctype>
#include <algorithm>
#include <ros/ros.h>
#include<std_msgs/String.h>
bool check_int(std::string str)
{
    if (std::all_of(str.cbegin(), str.cend(), isdigit))
    {
        std::cout << std::stoi(str) << std::endl;
        return true;
    }
    std::cout << "not int" << std::endl;
    return false;
}
bool get_number(int& first,int& second){
	std::string str;
    while(1){
        std::cout<<"変更したいwaypointの番号を入力せよ"<<std::endl;
        std::cout<<"キャンセルする場合は c を入力せよ"<<std::endl;
        std::cin >>str;
        if(check_int(str)){
          break;
        }else if(str =="c"){
          std::cout<<"キャンセル"<<std::endl;
		  return false;
        }
	}
	first = std::stoi(str);
	std::string stri;
    while(1){
        std::cout<<"変更後のwaypointの番号を入力せよ"<<std::endl;
        std::cout<<"キャンセルする場合は c を入力せよ"<<std::endl;
        std::cin >>stri;
        if(check_int(stri)){
          break;
        }else if(stri =="c"){
          std::cout<<"キャンセル"<<std::endl;
		  return false;
        }
	}
	second = std::stoi(stri);
	return true;
}
int main(int argc, char **argv)
{
	// ROSの初期化
	ros::init(argc, argv, "wayp_command");
	ros::NodeHandle nh;

	// パラメータの設定
	ros::NodeHandle ph("~");
	std::string topic_name;
	ph.param("topic_name", topic_name, std::string("/commnad"));

	// publisherの作成
	ros::Publisher command_pub = nh.advertise<std_msgs::String>(topic_name, 100);

	// whileの回る周期の設定
	ros::Rate rate(10);
	while(ros::ok())
	{
	  std::cout<<"ファイルを保存する場合は　save　"<<std::endl;
	  std::cout<<"waypointの番号を変更する場合は　c　"<<std::endl;
	  std::cout<<"を入力して!"<<std::endl;
	  std::string s;
	  bool get_ok=false;
	  while(ros::ok()){
	    std::cin >>s;
        if(s=="save" || s=="c") get_ok=true;
	    if(get_ok) break;
	    if(!get_ok) std::cout<<"コマンドが間違っている"<<std::endl;
	  }
	  std_msgs::String command;
      if(s=="save"){
       command.data="save";
	   command_pub.publish(command);
	   }else if(s=="c"){
         int first,secound;
	     if(get_number(first,secound)){
		   std::ostringstream oss;
		   oss << first<<","<<secound;
           command.data=oss.str();
		   command_pub.publish(command);
	     }
       }
	  rate.sleep();
	}

	return 0;
}