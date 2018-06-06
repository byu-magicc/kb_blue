#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#define _USE_MATH_DEFINES
#include <math.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Pose2D.h>

#include<opencv2/core/core.hpp>

// ORB_SLAM2 includes
#include <System.h>

namespace orb_slam2_ros {

  class ImageGrabber
  {
  public:
    ImageGrabber(ORB_SLAM2::System* pSLAM);

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;

  private:
    ros::Publisher pub_frame_;
    ros::Publisher pub_transform_;
    ros::Publisher pub_pose_;

    // Is the camera mounted backwards from the front of the car?
    bool backwards_ = false;

    tf::Transform cvMatToTF(cv::Mat Tcw);
  };

}