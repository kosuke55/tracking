#ifndef _KCF_ROS_KCF_TRACKER_
#define _KCF_ROS_KCF_TRACKER_

#include <stdlib.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
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
    cv::Ptr<cv::Tracker> tracker_;

    cv::Mat image_;

    std_msgs::Header header_;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher image_pub_;
    cv::Rect2d roi_;
    bool missing_ = true;

    boost::mutex mutex_;

    bool is_approximate_sync_ = true;
    std::string params_path_ = "~/catkin_ws/src/tracking/config/params.yml";
    cv::TrackerKCF::Params params;

    boost::shared_ptr<
      message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<
      message_filters::Synchronizer<ApproximateSyncPolicy> > approximate_sync_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    message_filters::Subscriber<jsk_recognition_msgs::RectArray> sub_boxes_;

    virtual void onInit();
    virtual void callback(const sensor_msgs::Image::ConstPtr& raw_image_msg,
                          const jsk_recognition_msgs::RectArray::ConstPtr& detected_boxes);

    virtual void load_image(cv::Mat& image, const sensor_msgs::Image::ConstPtr& image_msg);

  private:
  }; // class KcfTracker
} // namespace kcf_ros

#endif
