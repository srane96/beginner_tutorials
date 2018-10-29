/**
 *  @copyright (c) BSD
 *  @file    listener.cpp
 *  @author  Siddhesh Rane
 *
 *  @brief subscriber node;
 *
 *  @section DESCRIPTION
 *
 *  C++ program to implement subscriber node that subscribes to
 *  the 'chatter_topic' and displays the message sent by publisher.
 */
#include <ros/ros.h>
#include <std_msgs/String.h>
/**
 * @brief A callback function to respond to messages
 * @param msg string published by publisher node
 * @return void
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("Message Received: {%s}", msg->data.c_str());
}
int main(int argc, char **argv) {
  /// initialize the ros node
  ros::init(argc, argv, "subscriber");
  /// create an instance of NodeHandle
  ros::NodeHandle nh;
  /**
   *  create a subscriber that subscribes to the
   *  chatter topic with queue size of 1000
   */
  ros::Subscriber sub = nh.subscribe("chatter_topic", 1000,
chatterCallback);
  /*
   * ask ROS to wait for and execute callbacks until
   * the node shuts down.
   */
  ros::spin();
  return 0;
}
