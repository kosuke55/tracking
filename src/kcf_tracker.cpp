
#include "kcf_tracker.h"

namespace tracking
{
void KcfTracker::onInit()
{
  params.detect_thresh = 0.0001f;
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();
  pnh_.getParam("approximate_sync", is_approximate_sync_);
  pnh_.getParam("params_path", params_path_);
  pnh_.getParam("input_camera_info", input_camera_info_);
  ROS_INFO("params_path_:%s", params_path_.c_str());
  tracker_ = cv::TrackerKCF::create(params);
  cv::FileStorage fs(params_path_, cv::FileStorage::READ);
  cv::FileNode features = fs["my_object"];
  // tracker_->read(features);
  tracker_->save("/home/kosuke/catkin_ws/src/tracking/launch/fuga.yml");
  image_pub_ = pnh_.advertise<sensor_msgs::Image>("output_image", 1);
  pub_camera_info_ = pnh_.advertise<sensor_msgs::CameraInfo>(
      "output/camera_info", 1);
  sub_image_.subscribe(pnh_,
                       "/apply_mask_image_in_gripper/output", 100);
  sub_boxes_.subscribe(pnh_,
                       "/mask_rcnn_instance_segmentation/output/rects", 100);
  sub_info_ = nh_.subscribe(input_camera_info_, 1,
                            &KcfTracker::infoCallback, this);
  if (is_approximate_sync_)
  {
    approximate_sync_ =
      boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(100);
    approximate_sync_->connectInput(sub_image_,
                                    sub_boxes_);
    approximate_sync_->registerCallback(boost::bind
                                        (&KcfTracker::callback, this, _1, _2));
    ROS_INFO("approximate_sync");
  }
  else
  {
    sync_  =
      boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(1);
    sync_->connectInput(sub_image_,
                        sub_boxes_);
    sync_->registerCallback(boost::bind
                            (&KcfTracker::callback, this, _1, _2));
    ROS_INFO("sync");
  }
}

void KcfTracker::load_image(cv::Mat& image, const sensor_msgs::Image::ConstPtr& image_msg)
{
  try
  {
    cv_bridge::CvImagePtr cv_image =
      cv_bridge::toCvCopy(image_msg, "bgr8");
    image = cv_image->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("failed convert image from sensor_msgs::Image to cv::Mat");
    return;
  }
}

void KcfTracker::infoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& info_msg)
{
  boost::mutex::scoped_lock lock(mutex_);
  camera_info_ = info_msg;
}

void KcfTracker::callback(const sensor_msgs::Image::ConstPtr& raw_image_msg,
                          const jsk_recognition_msgs::RectArray::ConstPtr& detected_rects)
{
  header_ = raw_image_msg->header;
  image_.release();
  load_image(image_, raw_image_msg);
  auto rects = detected_rects->rects;
  if (rects.size() != 0)
  {
    ROS_INFO("redetect");
    roi_ = cv::Rect2d(rects[0].x - 10,
                      rects[0].y - 10,
                      rects[0].width + 20,
                      rects[0].height + 20);

    tracker_ = cv::TrackerKCF::create(params);
    tracker_->init(image_, roi_);
    missing_ = ! tracker_->update(image_, roi_);
    cv::rectangle(image_, roi_,
                  CV_RGB(0,255,0), 2);
    cv::putText(image_, "redetect", cv::Point(10,200),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,200), 2, CV_AA);
    // missing_ = false;
  }
  else if (! missing_)
  {
    ROS_INFO("tracking");
    missing_ = ! tracker_->update(image_, roi_);
    cv::rectangle(image_, roi_,
                  CV_RGB(0,255,0), 2);
    cv::putText(image_, "tracking", cv::Point(10,200),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,200), 2, CV_AA);
  }
  else if (missing_)
  {
    ROS_INFO("misisng");
    // missing_ = ! tracker_->update(image_, roi_);
    cv::putText(image_, "missing", cv::Point(10,200),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,200), 2, CV_AA);
  }
  image_pub_.publish(cv_bridge::CvImage(header_,
                                        sensor_msgs::image_encodings::BGR8,
                                        image_).toImageMsg());
  if (camera_info_)
  {
    sensor_msgs::CameraInfo ci_roi(*camera_info_);
    ci_roi.roi.x_offset = roi_.x;
    ci_roi.roi.y_offset = roi_.y;
    ci_roi.roi.width = roi_.width;
    ci_roi.roi.height = roi_.height;
    ci_roi.roi.do_rectify = true;
    ci_roi.header = header_;
    pub_camera_info_.publish(ci_roi);
  }
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tracking::KcfTracker, nodelet::Nodelet)
