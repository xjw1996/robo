#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <math.h>
#include <tf/LinearMath/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

//参考文献　http://wiki.ros.org/ja/rviz/Tutorials/Interactive%20Markers%3A%20Getting%20Started

// navigation no relationship this code no relationship
// http://wiki.ros.org/ja/turtlebot_navigation/Tutorials/Setup%20the%20Navigation%20Stack%20for%20TurtleBot
struct text_way{
  geometry_msgs::Point pose;
  double yaw;
  int mode; 
};
using namespace visualization_msgs;
class wayp_class{
  public:
    wayp_class( boost::shared_ptr<interactive_markers::InteractiveMarkerServer> serv);
    ~wayp_class();
  private:
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
    std::vector< text_way > wayp;
    void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
    InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg );
    void makeCube(int name, text_way posi);
    void makeCubeALL(int start);
    void read_data(std::string file_name);
    void eraseCube(int start);
    void insertCube(int num,text_way data);
    void save_file();
    ros::NodeHandle nh;
    ros::Subscriber string_sub;
    ros::Subscriber point_sub;
    std::string read_file_name_;
    std::string write_file_name_;
    std::string order_string_name_;
    std::string pose_topic_name_;
    interactive_markers::MenuHandler menu_handler;
    void stringCallback(const std_msgs::StringConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    ros::Publisher marker_pub;
    void publish_line();
    std::string get_mode_name(int mode_number);
};
wayp_class::wayp_class( boost::shared_ptr<interactive_markers::InteractiveMarkerServer> serv):server( serv ){
  menu_handler.insert( "Erase Cube", boost::bind( &wayp_class::processFeedback, this, _1 ));
  menu_handler.insert( "change mode normal", boost::bind( &wayp_class::processFeedback, this, _1 ));
  menu_handler.insert( "change mode serch", boost::bind( &wayp_class::processFeedback, this, _1 ));
  menu_handler.insert( "change mode cancel", boost::bind( &wayp_class::processFeedback, this, _1 ));
  menu_handler.insert( "change mode direct",boost::bind( &wayp_class::processFeedback, this, _1 ));
  menu_handler.insert( "change mode stop",boost::bind( &wayp_class::processFeedback, this, _1 ));
  menu_handler.insert( "change mode signal",boost::bind( &wayp_class::processFeedback, this, _1 ));
  ros::Duration(0.1).sleep();
  ros::NodeHandle private_nh("~");
  private_nh.param("read_file_name", read_file_name_, std::string("waypoint.txt"));
  private_nh.param("write_file_name", write_file_name_, std::string("rewaypoint.txt"));
  private_nh.param("pose_topic_name", pose_topic_name_, std::string("rewaypoint.txt"));
  private_nh.param("order_string_name", order_string_name_, std::string("rewaypoint.txt"));
  read_data(read_file_name_);
  for(int i=0;i<wayp.size();i++){
    makeCube(i,wayp[i]);
  }
  server->applyChanges();
  marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 1);
  string_sub = nh.subscribe(order_string_name_, 1, &wayp_class::stringCallback,this);
  point_sub = nh.subscribe(pose_topic_name_, 1, &wayp_class::poseCallback,this);
  publish_line();
}
void wayp_class::publish_line(){
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id ="map";
  line_strip.header.stamp=ros::Time::now();
  line_strip.ns = "lines";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 0;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.1;
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;
  for(int i=0;i<wayp.size();i++){
    geometry_msgs::Point p;
    p =wayp[i].pose;
    p.z=0;
    line_strip.points.push_back(p);
  }
  marker_pub.publish(line_strip);
}
wayp_class::~wayp_class(){
  server.reset();
}
void wayp_class::read_data(std::string file_name){
	std::ifstream ifs(file_name.c_str());
	if(ifs.fail()){
    std::cerr<<"file open error.\n";
    return;
  }
	std::string str;
	double x,y,z,qx,qy,qz,qw,mode,count;
  bool first=true;
  text_way tmp;
	while(getline(ifs,str)){
    if(first){
      for (char i : str)
          if (i == ',')
              count++;
    }
    first=false;
    if(count==6){
		  sscanf(str.data(),"%lf,%lf,%lf,%lf,%lf,%lf,%lf",&x,&y,&z,&qx,&qy,&qz,&qw);
      tmp.mode=0;
    }else{
      sscanf(str.data(),"%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",&x,&y,&z,&qx,&qy,&qz,&qw,&mode);
      tmp.mode=mode;
    }
    tmp.pose.x=x;
    tmp.pose.y=y;
    tmp.pose.z=z;
    double r,p,y;
    tf::Quaternion quat(qx,qy,qz,qw);
    tf::Matrix3x3(quat).getRPY(r, p, y);
    tmp.yaw=y;
    wayp.push_back(tmp);
	}
	ifs.close();
}
void wayp_class::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    {
      unsigned index = atoi( feedback->marker_name.c_str() );
      wayp[index].pose.x=feedback->pose.position.x;
      wayp[index].pose.y=feedback->pose.position.y;
      wayp[index].pose.z=feedback->pose.position.z;
      tf::Quaternion quat(feedback->pose.orientation.x,feedback->pose.orientation.y,feedback->pose.orientation.z,feedback->pose.orientation.w);
      double r,p,y;
      tf::Matrix3x3(quat).getRPY(r, p, y);
      wayp[index].yaw=y;
      publish_line();
      break;
    }
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
    {
      std::cout<<"Menu"<<feedback->menu_entry_id<<" selected"<<std::endl;
      if(feedback->menu_entry_id==1){
        //erase cube
        unsigned index = atoi( feedback->marker_name.c_str() );
        std::cout<<"No"<<index<<"erase"<<std::endl;
        eraseCube(index);
        wayp.erase(wayp.begin()+index);
        makeCubeALL(index);
        publish_line();
      }
      if(feedback->menu_entry_id>=2){
        unsigned index = atoi( feedback->marker_name.c_str() );
        wayp[index].mode=feedback->menu_entry_id-2;
        std::stringstream s;
        s << index;
        server->erase( s.str());
        server->applyChanges();
        makeCube(index,wayp[index]);
      }
      break;
    }
  }
  server->applyChanges();
}
void wayp_class::makeCube(int name, text_way posi)
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  int_marker.scale=1;
  int_marker.pose.position.x = posi.pose.x;
  int_marker.pose.position.y = posi.pose.y;
  int_marker.pose.position.z = 0.2;
  tf::Quaternion quat=tf::createQuaternionFromRPY(0,0,posi.yaw);
  geometry_msgs::Quaternion geometry_quat;
  quaternionTFToMsg(quat, geometry_quat);
  int_marker.pose.orientation=geometry_quat;
  std::stringstream s;
  s << name;
  int_marker.name = s.str();
  std::stringstream sdes;
  sdes << "No."<<name<<"-"<<"mode."<<get_mode_name(posi.mode);
  int_marker.description = sdes.str();

  InteractiveMarkerControl control;
  control.always_visible = true;
  control.independent_marker_orientation = true;

  //操作しない矢印タイプのマーカーの作成
  InteractiveMarkerControl arrow_control;
  Marker arrow_marker;
  arrow_control.always_visible = true;
  arrow_control.interaction_mode=InteractiveMarkerControl::MOVE_PLANE;
  arrow_marker.type = visualization_msgs::Marker::ARROW;
  arrow_marker.scale.x = 0.5;
  arrow_marker.scale.y = 0.1;
  arrow_marker.scale.z = 0.1;
  arrow_marker.color.r = 0.65;
  arrow_marker.color.g = 0.65;
  arrow_marker.color.b = 0.65;
  arrow_marker.color.a = 1.0;
  arrow_control.markers.push_back( arrow_marker );
  int_marker.controls.push_back( arrow_control );

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  /*
  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  */
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  /*
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
  */
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  /*
  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  */
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
  server->insert( int_marker );
  server->setCallback( int_marker.name, boost::bind( &wayp_class::processFeedback, this, _1 ));
  menu_handler.apply( *server, int_marker.name );
}
void wayp_class::makeCubeALL(int start){
  for(int i=start;i<wayp.size();i++){
    makeCube(i,wayp[i]);
  }
  server->applyChanges();
}
void wayp_class::eraseCube(int start){
  for(int i=start;i<wayp.size();i++){
    std::stringstream s;
    s << i;
    server->erase( s.str());
  }
  server->applyChanges();
  return;
}
void wayp_class::insertCube(int num,text_way data){
  eraseCube(num);
  wayp.insert(wayp.begin() + num,data);
  makeCubeALL(num);
  publish_line();
}
void wayp_class::save_file(){
	std::ofstream ofs;
	ofs.open(write_file_name_.c_str() , std::ios::out);//上書きする
	if (ofs.fail()){
		std::cerr << "失敗" << std::endl;
		return;
	}
	double x,y,z,yaw;
	for(unsigned int num = 0 ; num < wayp.size() ; num++){
    tf::Quaternion quat=tf::createQuaternionFromRPY(0,0,wayp[num].yaw);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
		ofs << wayp[num].pose.x <<","<< wayp[num].pose.y <<","<< wayp[num].pose.z <<","<< geometry_quat.x <<","<< geometry_quat.y <<","<< geometry_quat.z <<","<< geometry_quat.w <<","<<wayp[num].mode<< std::endl;
	}
  ofs.close();
}
void wayp_class::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
  text_way tmp;
  tmp.pose=msg->pose.pose.position;
  double qx,qy,qz,qw,r,p,y;
  qx=msg->pose.pose.orientation.x;
  qy=msg->pose.pose.orientation.y;
  qz=msg->pose.pose.orientation.z;
  qw=msg->pose.pose.orientation.w;
  tf::Quaternion quat(qx,qy,qz,qw);
  tf::Matrix3x3(quat).getRPY(r, p, y);
  tmp.yaw=y;
  tmp.mode=0;
  double min_distance=1000;
  int min_num=0;
  for(int i=0;i<wayp.size();i++){
    double tmp_dis=std::hypot(tmp.pose.x-wayp[i].pose.x,tmp.pose.y-wayp[i].pose.y);
    if(tmp_dis<min_distance){
      min_distance=tmp_dis;
      min_num=i;
    }
  }
  insertCube(min_num+1,tmp);
}
void wayp_class::stringCallback(const std_msgs::StringConstPtr& msg){
  if(msg->data=="save"){
    save_file();
    return;
  }
  std::vector<int> v;
  std::string data = msg->data;
  std::string s;
  std::stringstream ss{data};
  while ( getline(ss, s, ',') ){     // スペース（' '）で区切って，格納
    v.push_back(atoi(s.c_str()));
  }
  text_way backup_data=wayp[v[0]];
  if(v[0]<v[1]){
    //v[0]が小さい値
    eraseCube(v[0]);
    wayp.erase(wayp.begin() + v[0]);
    wayp.insert(wayp.begin() + v[1]-1,backup_data);
    makeCubeALL(v[0]);
  }else{
    eraseCube(v[1]);
    wayp.erase(wayp.begin() + v[0]);
    wayp.insert(wayp.begin() + v[1],backup_data);
    makeCubeALL(v[1]);
  }
  // data = 変更したい数、変更後の数
}
std::string wayp_class::get_mode_name(int mode_number){
  std::string mode_name;
  switch(mode_number){
    case 0:
      mode_name += "normal";
      break;
    case 1:
      mode_name += "serch_mode";
      break;
    case 2:
      mode_name += "cancel";
      break;
    case 3:
      mode_name += "direct";
      break;
    case 4:
      mode_name += "stop";
      break;
    case 5:
      mode_name += "signal";
  }


  return mode_name;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "wayp_edit");
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> sv(new interactive_markers::InteractiveMarkerServer("selection") );
  wayp_class ob(sv);
  ros::MultiThreadedSpinner spinner(2); 
  spinner.spin();
}