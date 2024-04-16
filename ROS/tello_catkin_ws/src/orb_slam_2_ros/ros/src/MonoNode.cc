#include "MonoNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport (node_handle);

    MonoNode node (ORB_SLAM2::System::MONOCULAR, node_handle, image_transport);

    ros::spin();

    ros::shutdown();

    return 0;
}


MonoNode::MonoNode (ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : Node (sensor, node_handle, image_transport) {
  image_subscriber = image_transport.subscribe ("/camera/image_raw", 1, &MonoNode::ImageCallback, this);
  config_subscriber = node_handle.subscribe ("/orb_slam2_mono/camera_config", 1, &MonoNode::ImageConfigCallback, this);
  bounding_box_subscriber = node_handle.subscribe ("object_detection_box", 1, &MonoNode::BoundingBoxCallback, this);
}


MonoNode::~MonoNode () {
}

void MonoNode::BoundingBoxCallback (const sensor_msgs::CameraInfo::ConstPtr& msg) {
    orb_slam_->mpTracker->UpdateBoundingBox(msg->binning_x, msg->binning_y, msg->width, msg->height, msg->distortion_model);

    // cout<<"received boudning box"<<msg->binning_x<<msg->binning_y<<msg->height<<msg->width<<", type="<<msg->distortion_model<<endl;

        // box_msg.binning_x = x
        // box_msg.binning_y = y
        // box_msg.width = w
        // box_msg.height = h
        // box_msg.distortion_model = name
        // self.bounding_box_publisher.publish(box_msg)

}


void MonoNode::ImageCallback (const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImageConstPtr cv_in_ptr;
  try {
      cv_in_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msg->header.stamp;

  orb_slam_->TrackMonocular(cv_in_ptr->image,cv_in_ptr->header.stamp.toSec());

  Update ();
}

void MonoNode::ImageConfigCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received Camera Configuation: %s", msg->data.c_str());
    orb_slam_->mpTracker->ChangeCalibration(msg->data.c_str());
    orb_slam_->mpViewer->ChangeConfiguration(msg->data.c_str());
    orb_slam_->mpMapDrawer->ChangeConfiguration(msg->data.c_str());
}
