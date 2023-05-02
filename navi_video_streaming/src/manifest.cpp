#include <pluginlib/class_list_macros.hpp>
#include <navi_video_streaming/resized_publisher.h>
#include <navi_video_streaming/resized_subscriber.h>

PLUGINLIB_EXPORT_CLASS(ResizedPublisher, image_transport::PublisherPlugin)

PLUGINLIB_EXPORT_CLASS(ResizedSubscriber, image_transport::SubscriberPlugin)
