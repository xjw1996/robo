#include "ros/ros.h"
#include <sl/Camera.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <chrono>
#include <vector>
#include <algorithm>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <tf/tf.h>
#include <std_msgs/Header.h>
#include <tf/transform_listener.h>
#include <boost/make_shared.hpp>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <zed_yolo_msg/FindObject.h>
#include <zed_yolo_msg/Objects.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;
using namespace sl;
void imageToROSmsg(sensor_msgs::ImagePtr imgMsgPtr, sl::Mat img, std::string frameId, ros::Time t);
cv::Mat slMat2cvMat(Mat& input);
class ROS_ZED{
  public:
    ROS_ZED();
    ~ROS_ZED();
    //メインのループ
    void zed_loop();
  private:
    ///zed関連
    Camera zed;
    InitParameters zed_init_parameters;
    RuntimeParameters zed_runtime_parameters;
    //ros関連
    tf::TransformListener listener_;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    ros::Publisher object_pub_;        
    sl::Mat image_zed, point_cloud;
    //yoloから返ってきたバウンディングボックスのデータ
    darknet_ros_msgs::BoundingBoxes data_box_;
    //imageをdarknet_rosにactionで送るための関数
    void image_send(sensor_msgs::ImagePtr imgMsgPtr,int id);
    void image_send2(sensor_msgs::Image imgMsg,int id);
    //darknet_rosから結果が返ってきたかどうか
    bool get_box_;
    //人が検出する関数
    void serch_person();
    //ゴミを検出する関数
    void serch_gomi();
    //darknet_rosのaction clientに設定されたコールバック関数
    void image_Cb(const actionlib::SimpleClientGoalState& state,
            const darknet_ros_msgs::CheckForObjectsResultConstPtr& result_msg);
    //パラメーター
    //zedから取得する間隔
    double zed_timer_interval_;
    //zed カメラのframe_id
    std::string frame_id;
    //zed カメラの
    std::string fixed_frame_id;
    //zedから取得した画像を送るトピック名
    std::string pub_image_topic_;
    //zedから取得した画像の解像度
    double image_resolution_factor_;
    //action_clientに画像を送った後待つ時間
    double time_out_box;
    std::string person_name_;
    //メインループのカウント
    int id_;
    using DarknetActionClient = actionlib::SimpleActionClient<darknet_ros_msgs::CheckForObjectsAction>;
    DarknetActionClient darknet_action_client_;
    //image_Cbが呼ばれたかどうか
    bool sub_box_;
    //zedカメラ以外のイメージの保管
    sensor_msgs::Image other_camera_msg_;
    //zedカメラ以外のイメージのトピック名
    image_transport::Subscriber sub_other_camera;
    std::string pub_object_topic_name;
    //zedカメラ以外のイメージを使用するかどうか
    bool use_other_camera_;
    //ゴミ判別のリスト
    std::vector<std::string> gomi_list_;
    std::string other_camera_topic_name_;
    void othercameraCb(const sensor_msgs::ImageConstPtr& msg);
    int action_mode_;
    //tf::TransformListener tf_listener;
};
ROS_ZED::ROS_ZED() : it_(nh_), darknet_action_client_(nh_, "/darknet_ros/check_for_objects", true){  
    //zedの設定
    zed_init_parameters.depth_mode = DEPTH_MODE::PERFORMANCE; // Use PERFORMANCE depth mode
    zed_init_parameters.coordinate_units = UNIT::METER; // Use meter units (for depth measurements)
    zed_init_parameters.camera_resolution= sl::RESOLUTION::HD720;
    zed_init_parameters.camera_fps=15;
    zed_init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    zed_init_parameters.sdk_gpu_id = 0;
    ERROR_CODE state = zed.open(zed_init_parameters);
    while(state != ERROR_CODE::SUCCESS) {
        state = zed.open(zed_init_parameters);
        cout << "zed Error " << state << endl;
        if(!ros::ok) return;
        ros::Duration(0.5).sleep();
    }
    if(!ros::ok) return;
    zed_runtime_parameters.sensing_mode = SENSING_MODE::STANDARD;
    //rosの設定
    id_=0;
    ros::NodeHandle private_nh("~");
    private_nh.param("zed_timer_interval", zed_timer_interval_, 0.2);
    private_nh.param("frame_id", frame_id, std::string("zed"));
    private_nh.param("fixed_frame_id", fixed_frame_id, std::string("base_link"));
    private_nh.param("pub_image_topic", pub_image_topic_, std::string("zed_image/image"));
    private_nh.param("image_resolution_factor_", image_resolution_factor_, 1.0);
    private_nh.param("time_out_box", time_out_box, 0.15);
    private_nh.param("gomi_list", gomi_list_, {std::string("Bento"), std::string("Can"),std::string( "Bottle")});
    private_nh.param("person_name", person_name_, std::string("person"));
    private_nh.param("use_other_camera", use_other_camera_, false);
    private_nh.param("other_camera_topic_name", other_camera_topic_name_, std::string("camera/image"));
    private_nh.param("pub_object_topic_name", pub_object_topic_name, std::string("/yolo_object"));
    image_pub_ = it_.advertise(pub_image_topic_, 1);
    if(use_other_camera_){
      sub_other_camera = it_.subscribe(other_camera_topic_name_, 1, &ROS_ZED::othercameraCb,this);
    }
    object_pub_=nh_.advertise<zed_yolo_msg::Objects>(pub_object_topic_name, 1);
    sl::Mat image_tmp(static_cast<int>(1280*image_resolution_factor_),static_cast<int>(720*image_resolution_factor_), MAT_TYPE::U8_C3);
    image_zed=image_tmp;
    darknet_action_client_.waitForServer();
    nh_.setParam("/yolo_mode", -1);
    nh_.getParam("/yolo_mode", action_mode_);
}
ROS_ZED::~ROS_ZED()
{
    zed.close();
}
void ROS_ZED::zed_loop()
{
  Resolution new_image_size(static_cast<int>(1280*image_resolution_factor_),static_cast<int>(720*image_resolution_factor_));
  ros::Time ros_begin;
  ros::Duration ros_duration;
  while(ros::ok){
    nh_.getParam("/yolo_mode", action_mode_);
    id_++;
    ros_begin = ros::Time::now();
    if (zed.grab(zed_runtime_parameters) == ERROR_CODE::SUCCESS) {
      zed.retrieveImage(image_zed, VIEW::LEFT, MEM::CPU, new_image_size);
      zed.retrieveMeasure(point_cloud, MEASURE::XYZRGBA, MEM::CPU, new_image_size);
      //zed.retrieveImage(image_zed, VIEW::LEFT, MEM::CPU);
      sensor_msgs::ImagePtr zed_image_msg = boost::make_shared<sensor_msgs::Image> ();
      imageToROSmsg(zed_image_msg,image_zed,frame_id,ros::Time::now());
      image_pub_.publish(zed_image_msg);
      if (darknet_action_client_.isServerConnected()&&action_mode_>=0){
        switch(action_mode_){
          case 0:
              image_send(zed_image_msg,id_);
              if (get_box_) serch_person();
              break;
          case 1:
              if(use_other_camera_){
                image_send2(other_camera_msg_,id_);
              }else{
                image_send(zed_image_msg,id_);
              }
              if (get_box_) serch_gomi();
              break;
        }
      }
    }
    ros_duration =ros::Time::now() - ros_begin;
    if(ros_duration.toSec() < zed_timer_interval_){
      ros::Duration(zed_timer_interval_ - ros_duration.toSec()).sleep();
    }
  }
}
void ROS_ZED::image_send(sensor_msgs::ImagePtr imgMsgPtr,int id){
  darknet_ros_msgs::CheckForObjectsGoal goal;
  goal.id=id;
  goal.image=*imgMsgPtr;
  sub_box_=false;
  darknet_action_client_.sendGoal(goal,
      boost::bind(&ROS_ZED::image_Cb, this, _1, _2),
      DarknetActionClient::SimpleActiveCallback(),
      DarknetActionClient::SimpleFeedbackCallback());
  bool finished_before_timeout = darknet_action_client_.waitForResult(ros::Duration(time_out_box));
  if (finished_before_timeout)  {
    while(!sub_box_){
      ROS_INFO("wait for getting bouding msg");
      ros::Duration(0.001).sleep();
    }
    get_box_=true;
  }else{
    darknet_action_client_.cancelAllGoals();
    ROS_INFO("bouding mag not comminng before the time out.");
    get_box_=false;
  }
}
void ROS_ZED::image_send2(sensor_msgs::Image imgMsg,int id){
  darknet_ros_msgs::CheckForObjectsGoal goal;
  goal.id=id;
  goal.image=imgMsg;
  sub_box_=false;
  darknet_action_client_.sendGoal(goal,
      boost::bind(&ROS_ZED::image_Cb, this, _1, _2),
      DarknetActionClient::SimpleActiveCallback(),
      DarknetActionClient::SimpleFeedbackCallback());
  bool finished_before_timeout = darknet_action_client_.waitForResult(ros::Duration(time_out_box));
  if (finished_before_timeout)  {
    while(!sub_box_){
      ROS_INFO("wait for getting bouding msg");
      ros::Duration(0.001).sleep();
    }
    get_box_=true;
  }else{
    darknet_action_client_.cancelAllGoals();
    ROS_INFO("bouding mag not comminng before the time out.");
    get_box_=false;
  }
}
void ROS_ZED::serch_person(){
  sl::float4 point3D;
  if(data_box_.bounding_boxes.empty()){
    ROS_INFO("bounding_boxes is empty");
    return;
  }else{
    zed_yolo_msg::Objects result_msg;
    geometry_msgs::PoseStamped pose_zed;
    geometry_msgs::PoseStamped pose_base;
    pose_zed.header.stamp=ros::Time::now();
    pose_zed.header.frame_id=frame_id;
    pose_zed.pose.position.z=0;
    pose_zed.pose.orientation.x=1;
    pose_zed.pose.orientation.y=0;
    pose_zed.pose.orientation.z=0;
    pose_zed.pose.orientation.w=0;
    for(int i=0;i<data_box_.bounding_boxes.size();i++){
      int center_box_x=(data_box_.bounding_boxes[i].xmax+data_box_.bounding_boxes[i].xmin)/2;
      int center_box_y=(data_box_.bounding_boxes[i].ymax+data_box_.bounding_boxes[i].ymin)/2;
      point_cloud.getValue(center_box_x,center_box_y,&point3D);
      double px = point3D.x;
      double py = point3D.y;
      double pz = point3D.z;
      std::cout<<data_box_.bounding_boxes[i].Class<<" of center pixcel is "<<"x:"<<center_box_x<<"  y:"<<center_box_y<<""<<std::endl;
      std::cout<<data_box_.bounding_boxes[i].Class<<" of position is "<<"x:"<<px<<"[m]  y:"<<py<<"[m]  z:"<<pz<<"[m]"<<std::endl;
      zed_yolo_msg::FindObject tmp_obj;
      tmp_obj.name=data_box_.bounding_boxes[i].Class;
      if(!std::isnan(px)){
        pose_zed.pose.position.x=px;
        pose_zed.pose.position.y=py;
        listener_.transformPose(fixed_frame_id,ros::Time(0),pose_zed,frame_id,pose_base);
        tmp_obj.x=pose_base.pose.position.x;
        tmp_obj.y=pose_base.pose.position.y;
        tmp_obj.header=pose_base.header;
        tmp_obj.get_point=true;
      }else{
        tmp_obj.get_point=false;
      }
      result_msg.obj.push_back(tmp_obj);
    }
    object_pub_.publish(result_msg);
  }
}
void ROS_ZED::serch_gomi(){
  if(data_box_.bounding_boxes.empty()){
    ROS_INFO("bounding_boxes is empty");
    return;
  }else{
    zed_yolo_msg::Objects result_msg;
    for(int i=0;i<data_box_.bounding_boxes.size();i++){
      std::cout<<data_box_.bounding_boxes[i].Class<<" is found"<<std::endl;
      std::vector<string>::iterator itr;
      itr = std::find(gomi_list_.begin(), gomi_list_.end(), data_box_.bounding_boxes[i].Class);
      if (itr != gomi_list_.end()){
        ROS_INFO("gomi is found");
        zed_yolo_msg::FindObject tmp_obj;
        tmp_obj.name=data_box_.bounding_boxes[i].Class;
        tmp_obj.get_point=false;
        result_msg.obj.push_back(tmp_obj);
      }
    }
    object_pub_.publish(result_msg);
  }
}
void ROS_ZED::image_Cb(const actionlib::SimpleClientGoalState& state,
            const darknet_ros_msgs::CheckForObjectsResultConstPtr& result_msg)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  data_box_ =result_msg->bounding_boxes;
  sub_box_=true;
}
void ROS_ZED::othercameraCb(const sensor_msgs::ImageConstPtr& msg)
{
  other_camera_msg_ = *msg;
}
cv::Mat slMat2cvMat(Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM::CPU));
}
void imageToROSmsg(sensor_msgs::ImagePtr imgMsgPtr, sl::Mat img, std::string frameId, ros::Time t) {
  if(!imgMsgPtr) {            
    return;
  }

  imgMsgPtr->header.stamp = t;
  imgMsgPtr->header.frame_id = frameId;
  imgMsgPtr->height = img.getHeight();
  imgMsgPtr->width = img.getWidth();
  int num = 1; // for endianness detection
  imgMsgPtr->is_bigendian = !(*(char*)&num == 1);

  imgMsgPtr->step = img.getStepBytes();

  size_t size = imgMsgPtr->step * imgMsgPtr->height;
  imgMsgPtr->data.resize(size);

  sl::MAT_TYPE dataType = img.getDataType();
  switch (dataType) {
  case sl::MAT_TYPE::F32_C1: /**< float 1 channel.*/
    imgMsgPtr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    memcpy((char*)(&imgMsgPtr->data[0]), img.getPtr<sl::float1>(), size);
    break;
  case sl::MAT_TYPE::F32_C2: /**< float 2 channels.*/
    imgMsgPtr->encoding = sensor_msgs::image_encodings::TYPE_32FC2;
    memcpy((char*)(&imgMsgPtr->data[0]), img.getPtr<sl::float2>(), size);
    break;
  case sl::MAT_TYPE::F32_C3: /**< float 3 channels.*/
    imgMsgPtr->encoding = sensor_msgs::image_encodings::TYPE_32FC3;
    memcpy((char*)(&imgMsgPtr->data[0]), img.getPtr<sl::float3>(), size);
    break;
  case sl::MAT_TYPE::F32_C4: /**< float 4 channels.*/
    imgMsgPtr->encoding = sensor_msgs::image_encodings::TYPE_32FC4;
    memcpy((char*)(&imgMsgPtr->data[0]), img.getPtr<sl::float4>(), size);
    break;
  case sl::MAT_TYPE::U8_C1: /**< unsigned char 1 channel.*/
    imgMsgPtr->encoding = sensor_msgs::image_encodings::MONO8;
    memcpy((char*)(&imgMsgPtr->data[0]), img.getPtr<sl::uchar1>(), size);
    break;
  case sl::MAT_TYPE::U8_C2: /**< unsigned char 2 channels.*/
    imgMsgPtr->encoding = sensor_msgs::image_encodings::TYPE_8UC2;
    memcpy((char*)(&imgMsgPtr->data[0]), img.getPtr<sl::uchar2>(), size);
    break;
  case sl::MAT_TYPE::U8_C3: /**< unsigned char 3 channels.*/
    imgMsgPtr->encoding = sensor_msgs::image_encodings::BGR8;
    memcpy((char*)(&imgMsgPtr->data[0]), img.getPtr<sl::uchar3>(), size);
    break;
  case sl::MAT_TYPE::U8_C4: /**< unsigned char 4 channels.*/
    imgMsgPtr->encoding = sensor_msgs::image_encodings::BGRA8;
    memcpy((char*)(&imgMsgPtr->data[0]), img.getPtr<sl::uchar4>(), size);
    break;
  }
}
int main(int argc, char **argv)
{  
  ros::init(argc, argv, "zed_server");
  ROS_ZED ros_zed_obj;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros_zed_obj.zed_loop();
  ros::waitForShutdown();
  return 0;
}
