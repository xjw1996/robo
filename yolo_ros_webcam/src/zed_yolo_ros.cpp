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
using namespace std;
using namespace sl;
void imageToROSmsg(sensor_msgs::ImagePtr imgMsgPtr, sl::Mat img, std::string frameId, ros::Time t);
cv::Mat slMat2cvMat(Mat& input);
class ROS_ZED{
  public:
    ROS_ZED();
    ~ROS_ZED();
  private:
    ///zed関連
    Camera zed;
    InitParameters zed_init_parameters;
    RuntimeParameters zed_runtime_parameters;
    //ros関連
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;        
    image_transport::Publisher image_pub_;
    ros::Subscriber BoundingBoxes_sub_;
    ros::Timer get_timer_;
    void zed_timer_callback(const ros::TimerEvent& e);
    void bound_box_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& box_msg);
    //データ関連
    darknet_ros_msgs::BoundingBoxes data_box_;
    bool get_box_;
    sl::Mat image_zed, point_cloud;
    //処理用
    std::mutex mtx_;
    std::condition_variable cond_;
    //パラメーター
    //zedから取得する間隔
    double zed_timer_interval_;
    std::string frame_id;
    std::string child_frame_id;
    std::string pub_image_topic_;
    std::string sub_BoundingBoxes_topic_;
    int image_width_;
    int image_height_;
    int time_out_box;
    bool sub_box_;
};
ROS_ZED::ROS_ZED() : it_(nh_){  
    //zedの設定
    zed_init_parameters.depth_mode = DEPTH_MODE::PERFORMANCE; // Use PERFORMANCE depth mode
    zed_init_parameters.coordinate_units = UNIT::METER; // Use millimeter units (for depth measurements)
    zed_init_parameters.camera_resolution= sl::RESOLUTION::HD720;
    zed_init_parameters.camera_fps=15;
    zed_init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    zed_init_parameters.sdk_gpu_id = 0;
    // Open the camera
    ERROR_CODE state = zed.open(zed_init_parameters);
    if (state != ERROR_CODE::SUCCESS) {
        cout << "Error " << state << ", exit program." << endl;
        return;
    }
    zed_runtime_parameters.sensing_mode = SENSING_MODE::STANDARD;
    //rosの設定
    ros::NodeHandle private_nh("~");
    private_nh.param("zed_timer_interval", zed_timer_interval_, 0.03);
    private_nh.param("frame_id", frame_id, std::string("zed"));
    private_nh.param("child_frame_id", child_frame_id, std::string("base_link"));
    private_nh.param("sub_BoundingBoxes_topic", sub_BoundingBoxes_topic_, std::string("/bouding_topic"));
    private_nh.param("pub_image_topic", pub_image_topic_, std::string("/image_kkkk"));
    private_nh.param("image_width", image_width_, 640);
    private_nh.param("image_height", image_height_, 480);
    private_nh.param("time_out_box_miliseconds", time_out_box, 200);
    get_timer_ =nh_.createTimer(ros::Duration(zed_timer_interval_), &ROS_ZED::zed_timer_callback, this);
    BoundingBoxes_sub_ = nh_.subscribe(sub_BoundingBoxes_topic_,1,&ROS_ZED::bound_box_callback,this);
    image_pub_ = it_.advertise(pub_image_topic_, 1);
    get_box_=false;
    sub_box_=false;
    sl::Mat image_tmp(image_width_, image_height_, MAT_TYPE::U8_C3);
    image_zed=image_tmp;
}
ROS_ZED::~ROS_ZED()
{
    zed.close();
}
void ROS_ZED::zed_timer_callback(const ros::TimerEvent& e)
{
  std::unique_lock<std::mutex> lk(mtx_);
  // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
  // Only the headers and pointer to the sl::Mat are copied, not the data itself
  std::cout<<"zed timer"<<std::endl;
  Resolution new_image_size(image_width_, image_height_);
  if (zed.grab(zed_runtime_parameters) == ERROR_CODE::SUCCESS) {
    zed.retrieveImage(image_zed, VIEW::LEFT, MEM::CPU, new_image_size);
    //zed.retrieveImage(image_zed, VIEW::LEFT, MEM::CPU);
    sensor_msgs::ImagePtr image_msg = boost::make_shared<sensor_msgs::Image> ();
    imageToROSmsg(image_msg,image_zed,frame_id,ros::Time::now());
    image_pub_.publish(image_msg);
    sub_box_=false;
    zed.retrieveMeasure(point_cloud, MEASURE::XYZRGBA, MEM::CPU, new_image_size);
    /*
    std::chrono::steady_clock::time_point tp = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_out_box);
    std::cout<<"send imagetime:"<<ros::Time::now()<<std::endl;
    if (!cond_.wait_until(lk, tp, [this] { return sub_box_; })) {
      std::cout << "boudingbox msg not comming" << std::endl;
      std::cout<<"time out time:"<<ros::Time::now()<<std::endl;
      return;
    }
    */
    while (!sub_box_) {
      // 述語を指定しないバージョン
      // 3秒でタイムアウト
      using namespace std::chrono;
      steady_clock::time_point tp = steady_clock::now() + milliseconds(time_out_box);
      std::cv_status result = cond_.wait_until(lk, tp);
      if (result == std::cv_status::timeout) {
        std::cout << "wait_until_data_to_process1 : timeout" << std::endl;
        return;
      }
    }
    sub_box_=false;
    if(get_box_==false) return;
    sl::float4 point3D;
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
void ROS_ZED::bound_box_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& box_msg){
  std::cout<<"sub box"<<std::endl;
  std::cout<<"sub box time:"<<ros::Time::now()<<std::endl;
  if(box_msg->bounding_boxes.empty()){
    get_box_ = false;    // 準備完了したので待機スレッドを全て起床させる
  }else{
    data_box_ = *box_msg;
    get_box_ = true;
  }
  {
    std::unique_lock<std::mutex> lk(mtx_);
    sub_box_ = true;
  }
    // 準備完了したので待機スレッドを全て起床させる
  cond_.notify_all();
  return;
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
  ros::init(argc, argv, "zed_yolo_ros");
  ROS_ZED ros_zed_obj;
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  return 0;
}
