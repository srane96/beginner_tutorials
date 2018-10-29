/**
 *  @copyright (c) BSD
 *  @file    talker.cpp
 *  @author  Siddhesh Rane
 *
 *  @brief publisher node;
 *
 *  @section DESCRIPTION
 *
 *  C++ program to implement publisher node that publishes
 *  custom string messages on 'chatter_topic'
 */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
int main(int argc, char **argv) {
  /// initialize the ros node
  ros::init(argc, argv, "publisher");
  /// create an instance of NodeHandle
  ros::NodeHandle nh;
  /**
   *  create a publisher node that publishes standard
   *  string messages to the chatter topic with queue
   *  size of 1000
   */
  ros::Publisher chatter_pub = nh.advertise < std_msgs::String
      > ("chatter_topic", 1000);
  /// loop at 10Hz rate
  ros::Rate loop_rate(10);
  /// initiate the count
  int count = 0;
  /// loop until node is shutdown
  while (ros::ok()) {
    /// create String message
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Hello ENPM808X ROS: " << count;
    /// store custom string to message data
    msg.data = ss.str();
    /// display the message
    ROS_INFO("%s", msg.data.c_str());
    /// publish message using publisher object
    chatter_pub.publish(msg);
    /**
     * ask ROS to execute all of the pending callbacks
     * from all of the node’s subscriptions
     */
    ros::spinOnce();
    /// wait until it's time for another iteration
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
