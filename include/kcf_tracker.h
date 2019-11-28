#ifndef _KCF_ROS_KCF_TRACKER_
#define _KCF_ROS_KCF_TRACKER_

#include <stdlib.h>
#include <string>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/tracking/tldDataset.hpp>
#include <jsk_recognition_msgs/RectArray.h>



namespace tracking
{
  class KcfTracker : public nodelet::Nodelet
  {
  public:
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Image,
      jsk_recognition_msgs::RectArray
      > SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,
      jsk_recognition_msgs::RectArray
      > ApproximateSyncPolicy;

  protected:

    virtual void onInit();
    virtual void callback(const sensor_msgs::Image::ConstPtr& raw_image_msg,
                          const jsk_recognition_msgs::RectArray::ConstPtr& detected_boxes);
    virtual void load_image(cv::Mat& image, const sensor_msgs::Image::ConstPtr& image_msg);
    virtual void infoCallback(
      const sensor_msgs::CameraInfo::ConstPtr& info_msg);

    cv::Ptr<cv::Tracker> tracker_;
    cv::Mat image_;
    cv::TrackerKCF::Params params;
    std_msgs::Header header_;
    sensor_msgs::CameraInfo::ConstPtr camera_info_;
    std::string params_path_ = "~/catkin_ws/src/tracking/config/params.yml";
    std::string input_camera_info_ = "/head_mount_kinect/hd/camera_info";
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher image_pub_;
    ros::Publisher pub_camera_info_;
    // ros::Subscriber<sensor_msgs::CameraInfo, KcfTracker> sub_info_;
    ros::Subscriber sub_info_;
    cv::Rect2d roi_;
    boost::mutex mutex_;
    boost::shared_ptr<
      message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<
      message_filters::Synchronizer<ApproximateSyncPolicy> > approximate_sync_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    message_filters::Subscriber<jsk_recognition_msgs::RectArray> sub_boxes_;
    bool missing_ = true;
    bool is_approximate_sync_ = true;

  private:
  }; // class KcfTracker
} // namespace kcf_ros

#endif
