#include <ros/ros.h>
#include "orb_slam2/image_grabber.h"

#include <System.h>

int main(int argc, char** argv) {
  // start node
  ros::init(argc, argv, "orb_slam2");
  ros::start();

  // get relevant tf frames
  ros::NodeHandle nh_private("~");
  // nh_private.param<std::string>("vocabulary", result_frame_, "map_ned");
  // nh_private.param<std::string>("settings", camera_frame_, "camera");


  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM2::System SLAM("", "", ORB_SLAM2::System::RGBD, true);

  // instantiate an object
  // orb_slam2::ImageGrabber igb;

  ros::spin();
  return 0;
}