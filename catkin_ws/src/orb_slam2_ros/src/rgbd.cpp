#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>

#include "orb_slam2_ros/image_grabber.h"

// ORB_SLAM2 includes
#include <System.h>

int main(int argc, char** argv) {
  // start node
  ros::init(argc, argv, "orb_slam2");
  ros::start();

  // node handles
  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh;

  // get relevant file paths
  std::string path_vocab, path_settings;
  nh_private.param<std::string>("vocabulary", path_vocab, "/home/plusk01/dev/kb_blue/catkin_ws/build/orb_slam2_ros/orbslam2-download/src/Vocabulary/ORBvoc.txt");
  nh_private.param<std::string>("settings", path_settings, "/home/plusk01/dev/kb_blue/catkin_ws/src/kb_blue/param/d435.yaml");


  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM2::System SLAM(path_vocab, path_settings, ORB_SLAM2::System::RGBD, true);

  // instantiate an object
  orb_slam2_ros::ImageGrabber igb(&SLAM);

  message_filters::Subscriber<sensor_msgs::Image> sub_rgb(nh, "/camera/color/image_rect_color", 1);
  message_filters::Subscriber<sensor_msgs::Image> sub_depth(nh, "/camera/aligned_depth_to_color/image_raw", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(100), sub_rgb, sub_depth);
  sync.registerCallback(std::bind(&orb_slam2_ros::ImageGrabber::GrabRGBD, &igb, std::placeholders::_1, std::placeholders::_2));

  ros::spin();

  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  ros::shutdown();

  return 0;
}