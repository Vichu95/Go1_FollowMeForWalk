#include <ros/ros.h>
#include <boost/endian/conversion.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <UnitreeCameraSDK.hpp>
#include <unistd.h>


void toImageMsg(sensor_msgs::Image& ros_image, cv::Mat image) {
  ros_image.height = image.rows;
  ros_image.width = image.cols;
  ros_image.encoding = "bgr8";
  ros_image.is_bigendian = (boost::endian::order::native == boost::endian::order::big);
  ros_image.step = image.cols * image.elemSize();
  size_t size = ros_image.step * image.rows;
  ros_image.data.resize(size);
  if (image.isContinuous()) 
    memcpy((char*)(&ros_image.data[0]), image.data, size);
  else {
    uchar* ros_data_ptr = (uchar*)(&ros_image.data[0]);
    uchar* cv_data_ptr = image.data;
    for (int i = 0; i < image.rows; ++i) {
      memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
      ros_data_ptr += ros_image.step;
      cv_data_ptr += image.step;
    }
  }
}

class Camera {
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    int fps_;   
    std::string frame_id_;
    UnitreeCamera cam_;  
    cv::Size frame_size_; 

    public:
        Camera(ros::NodeHandle *nh_, ros::NodeHandle *p_nh_,
                  int device_node, std::string device_name, 
                  int fps, int frame_width, 
                  int frame_height) : it_(*p_nh_),
                                      cam_(device_node),
                                      frame_size_(frame_width, frame_height), 
                                      fps_(fps) {
            image_pub_ = it_.advertise("image", 1);
            cam_.setRawFrameSize(frame_size_); 
            cam_.setRawFrameRate(fps_);
            frame_id_ = device_name;
        }

        ~Camera() {
            cam_.stopCapture(); 
        }

       void run() {
            cam_.startCapture(); 
            auto rate = ros::Rate(fps_);
            while(cam_.isOpened() && ros::ok()) {               
                cv::Mat frame;
                sensor_msgs::Image img_msg;
                img_msg.header.stamp = ros::Time::now();
                img_msg.header.frame_id = frame_id_;
                std::chrono::microseconds t;
                if(!cam_.getRawFrame(frame, t)) { 
                    usleep(1000);
                    continue;
                }
                toImageMsg(img_msg, frame);
                image_pub_.publish(img_msg);
            }
        }
};


int main(int argc, char *argv[]) {   
    ros::init(argc, argv, "go1_camera");
    ros::NodeHandle nh, p_nh("~");
    int camera_id, fps, frame_height, frame_width;
    std::string camera_name;
    ros::param::param<int>("~camera_id", camera_id, 0);    
    ros::param::param<std::string>("~camera_name", camera_name, "not_provided");       
    ros::param::param<int>("~fps", fps, 30);    
    ros::param::param<int>("~frame_width", frame_width, 1856);    
    ros::param::param<int>("~frame_height", frame_height, 800);    
    Camera driver(&nh, &p_nh, camera_id, camera_name, fps, frame_width, frame_height);
    driver.run();
    return 0;
}
