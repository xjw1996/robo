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
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <boost/make_shared.hpp>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
using namespace std;
using namespace sl;
void imageToROSmsg(sensor_msgs::ImagePtr imgMsgPtr, sl::Mat img, std::string frameId, ros::Time t);
cv::Mat slMat2cvMat(Mat& input);
class ROS_ZED{
  public:
    ROS_ZED();
    ~ROS_ZED();
    void zed_loop();
  private:
    ///zed関連
    Camera zed;
    InitParameters zed_init_parameters;
    RuntimeParameters zed_runtime_parameters;
    //ros関連
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;        
    sl::Mat image_zed, point_cloud;
    darknet_ros_msgs::BoundingBoxes data_box_;
    void image_send(sensor_msgs::ImagePtr imgMsgPtr,int id);
    bool get_box_;
    void image_Cb(const actionlib::SimpleClientGoalState& state,
            const darknet_ros_msgs::CheckForObjectsResultConstPtr& result);
    //パラメーター
    //zedから取得する間隔
    double zed_timer_interval_;
    std::string frame_id;
    std::string child_frame_id;
    std::string pub_image_topic_;
    double image_resolution_factor_;
    double time_out_box;
    int id_;
    using DarknetActionClient = actionlib::SimpleActionClient<darknet_ros_msgs::CheckForObjectsAction>;
    DarknetActionClient darknet_action_client_;
    bool sub_box_;
};
ROS_ZED::ROS_ZED() : it_(nh_), darknet_action_client_(nh_, "/darknet_ros/check_for_objects", true){  
    //zedの設定
    zed_init_parameters.depth_mode = DEPTH_MODE::PERFORMANCE; // Use PERFORMANCE depth mode
    zed_init_parameters.coordinate_units = UNIT::METER; // Use millimeter units (for depth measurements)
    zed_init_parameters.camera_resolution= sl::RESOLUTION::HD720;
    zed_init_parameters.camera_fps=15;
    zed_init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    zed_init_parameters.sdk_gpu_id = 0;
    ERROR_CODE state = zed.open(zed_init_parameters);
    while(state != ERROR_CODE::SUCCESS) {
        state = zed.open(zed_init_parameters);
        cout << "Error " << state << endl;
        ros::Duration(0.5).sleep();
    }
    zed_runtime_parameters.sensing_mode = SENSING_MODE::STANDARD;
    //rosの設定
    id_=0;
    ros::NodeHandle private_nh("~");
    private_nh.param("zed_timer_interval", zed_timer_interval_, 0.03);
    private_nh.param("frame_id", frame_id, std::string("zed"));
    private_nh.param("child_frame_id", child_frame_id, std::string("base_link"));
    private_nh.param("pub_image_topic", pub_image_topic_, std::string("/zed_image"));
    private_nh.param("image_resolution_factor_", image_resolution_factor_, 1.0);
    private_nh.param("time_out_box", time_out_box, 0.1);
    image_pub_ = it_.advertise(pub_image_topic_, 1);
    sl::Mat image_tmp(static_cast<int>(1280*image_resolution_factor_),static_cast<int>(720*image_resolution_factor_), MAT_TYPE::U8_C3);
    image_zed=image_tmp;
    darknet_action_client_.waitForServer();
}
ROS_ZED::~ROS_ZED()
{
    zed.close();
}
void ROS_ZED::zed_loop()
{
  // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
  // Only the headers and pointer to the sl::Mat are copied, not the data itself
  Resolution new_image_size(static_cast<int>(1280*image_resolution_factor_),static_cast<int>(720*image_resolution_factor_));
  ros::Time ros_begin = ros::Time::now();
  ros::Duration ros_duration;
  while(ros::ok){
    id_++;
    ros_begin = ros::Time::now();
    if (zed.grab(zed_runtime_parameters) == ERROR_CODE::SUCCESS) {
      zed.retrieveImage(image_zed, VIEW::LEFT, MEM::CPU, new_image_size);
      //zed.retrieveImage(image_zed, VIEW::LEFT, MEM::CPU);
      sensor_msgs::ImagePtr image_msg = boost::make_shared<sensor_msgs::Image> ();
      imageToROSmsg(image_msg,image_zed,frame_id,ros::Time::now());
      image_pub_.publish(image_msg);
      if (darknet_action_client_.isServerConnected()){
        image_send(image_msg,id_);
        if (get_box_)  {
          zed.retrieveMeasure(point_cloud, MEASURE::XYZRGBA, MEM::CPU, new_image_size);
          sl::float4 point3D;
          if(data_box_.bounding_boxes.empty()){
            ROS_INFO("bounding_boxes is empty");
          }else{
            for(int i=0;i<data_box_.bounding_boxes.size();i++){
              int center_box_x=(data_box_.bounding_boxes[i].xmax+data_box_.bounding_boxes[i].xmin)/2;
              int center_box_y=(data_box_.bounding_boxes[i].ymax+data_box_.bounding_boxes[i].ymin)/2;
              point_cloud.getValue(center_box_x,center_box_y,&point3D);
              float x = point3D.x;
              float y = point3D.y;
              float z = point3D.z;
              std::cout<<data_box_.bounding_boxes[i].Class<<" of center pixcel is"<<"x:"<<center_box_x<<"[m]  y:"<<center_box_y<<"[m]"<<std::endl;
              std::cout<<data_box_.bounding_boxes[i].Class<<" of position is"<<"x:"<<x<<"[m]  y:"<<y<<"[m]  z:"<<z<<"[m]"<<std::endl;
            }
          }
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
void ROS_ZED::image_Cb(const actionlib::SimpleClientGoalState& state,
            const darknet_ros_msgs::CheckForObjectsResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  data_box_ =result->bounding_boxes;
  sub_box_=true;
}
/**
* Conversion function between sl::Mat and cv::Mat
**/
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
  ros::init(argc, argv, "zed_action");
  ROS_ZED ros_zed_obj;
  ros_zed_obj.zed_loop();
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  return 0;
}
