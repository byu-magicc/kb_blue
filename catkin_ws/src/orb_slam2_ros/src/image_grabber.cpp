#include "orb_slam2_ros/image_grabber.h"

namespace orb_slam2_ros {

ImageGrabber::ImageGrabber(ORB_SLAM2::System* pSLAM)
  : mpSLAM(pSLAM), it_(nh_)
{

  // Setup an image transport publisher
  pub_frame_ = it_.advertise("/orb/frame", 1);

  pub_transform_ = nh_.advertise<geometry_msgs::Transform>("/orb/transform", 1);
  pub_pose_ = nh_.advertise<geometry_msgs::Transform>("/orb/pose", 1);

}

// ----------------------------------------------------------------------------

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try
  {
    cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try
  {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Main slam routine. Extracts new pose
  // (from ORB_SLAM2 API)
  // Proccess the given monocular frame
  // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
  // Returns the camera pose (empty if tracking fails).
  cv::Mat Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());

  geometry_msgs::Pose2D pose;
  if (!Tcw.empty())
  {

    // global pose from ORB_SLAM Origin
    tf::Transform tf_pose = cvMatToTF(Tcw);

    geometry_msgs::Transform transformMsg;
    tf::transformTFToMsg(tf_pose, transformMsg);

    pub_transform_.publish(transformMsg);

    tf::Matrix3x3 m(tf_pose.getRotation());
    double roll, pitch, yaw; // rotation about x, y, z <-- we want camera y to be FLU z
    m.getRPY(roll, pitch, yaw);

    // convert camera coordinates to FLU body (camera facing forward)
    pose.x =  transformMsg.translation.z;
    pose.y = -transformMsg.translation.x;
    pose.theta = pitch;

    // If camera is facing backwards
    if (backwards_)
    {
      pose.x = -pose.x;
      pose.y = -pose.y;
      pose.theta += M_PI;
    }

  }
  pub_pose_.publish(pose);

  // publish the image frame with tracking info from ORB_SLAM2
  cv::Mat frame = mpSLAM->GetTrackingFrame();
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
  pub_frame_.publish(msg);
}

// ----------------------------------------------------------------------------

tf::Transform ImageGrabber::cvMatToTF(cv::Mat Tcw)
{
  tf::Transform cam_to_first_keyframe_transform;
  // invert since Tcw (transform from world to camera)
  cv::Mat pose = Tcw.inv();

  // Extract transformation from the raw orb SLAM pose
  tf::Vector3 origin;
  //tf::Quaternion transform_quat;
  tf::Matrix3x3 transform_matrix;

  origin.setValue(pose.at<float>(0,3), pose.at<float>(1, 3), pose.at<float>(2, 3));

  transform_matrix.setValue(pose.at<float>(0, 0), pose.at<float>(0, 1), pose.at<float>(0, 2),
                            pose.at<float>(1, 0), pose.at<float>(1, 1), pose.at<float>(1, 2),
                            pose.at<float>(2, 0), pose.at<float>(2, 1), pose.at<float>(2, 2));

  //transform_matrix.getRotation(transform_quat);
  cam_to_first_keyframe_transform.setOrigin(origin);
  cam_to_first_keyframe_transform.setBasis(transform_matrix);

  return cam_to_first_keyframe_transform;
}

}