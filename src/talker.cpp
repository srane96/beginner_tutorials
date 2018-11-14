/**
 * BSD 3-Clause License
 * @copyright (c) 2018, Siddhesh Rane
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * @file    talker.cpp
 * @author  Siddhesh Rane
 * @version 1.2
 * @brief publisher node;
 *
 * @section DESCRIPTION
 *
 * C++ program to implement publisher node that publishes
 * custom string messages on 'chatter_topic'
 */
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <sstream>
#include "beginner_tutorials/strManipulator.h"
/// declare a global struct variable base_string
struct global_string {
  std::string content;
} base_string;
/// std::string base_string = "Hello ROS";
/**
 * @brief function to manipulate base number using client request
 * @param req int64 number requested by client
 * @param res int64 result 10 times req number
 * @return boolean
 */
bool manipulate(beginner_tutorials::strManipulator::Request &req,
                beginner_tutorials::strManipulator::Response &res) {
  int number = req.num;
  std::string str;
  ROS_INFO("Received number: %d", number);
  if (number < 0)
    ROS_ERROR("Please enter positive number");
  if (number % 2 == 0)
    str = "Even number entered";
  else
    str = "Odd number entered";
  ROS_DEBUG("Result generated");
  ROS_WARN("Warning: base_string will be modified");
  base_string.content = str;
  return true;
}
int main(int argc, char **argv) {
  /// initialize the base string
  base_string.content = "Hello ROS";
  /// initialize the ros node
  ros::init(argc, argv, "publisher");
  /// create an instance of NodeHandle
  ros::NodeHandle nh;
  /// create TransformBroadcaster object
  static tf::TransformBroadcaster br;
  /// create Transform object
  tf::Transform transform;
  /// create Quaternion object
  tf::Quaternion q;
  /// default frequency
  int frq = 5;
  /// check if user has entered argument
  if (argc == 2) {
    /// if argument is found change the frequency
    if (atoi(argv[1]) < 0) {
      /// set fatal log if rate is negative
      ROS_FATAL("Fatal: Invalid rate");
      return 1;
    } else if (atoi(argv[1]) > 1000) {
      /// set error log if rate is too high
      ROS_ERROR("Rate too large");
    }
    frq = atoi(argv[1]);
  } else {
    /// if no argument is found, exit the program
    ROS_INFO("Fatal: No argument passed for frequency");
  }
  /**
   *  create a publisher node that publishes standard
   *  string messages to the chatter topic with queue
   *  size of 1000
   */
  ros::Publisher chatter_pub = nh.advertise < std_msgs::String
      > ("chatter_topic", 1000);
  /**
   * define a ServiceServer object to call manipulate
   */
  ros::ServiceServer server = nh.advertiseService("manipulate_service",
                                                 manipulate);
  /// default loop at 10Hz rate
  ros::Rate loop_rate(frq);
  /// initiate the count
  int count = 0;
  /// loop until node is shutdown
  while (ros::ok()) {
    /// create String message
    std_msgs::String msg;
    std::stringstream ss;
    ss << base_string.content << count;
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
    double x = cos(ros::Time::now().toSec());
    double y = sin(ros::Time::now().toSec());
    double z = 0.0;
    double yaw = 1.0;
    /// set translational element of the transform
    transform.setOrigin(tf::Vector3(x, y, z));
    /// set row pitch and yaw angle
    q.setRPY(0, 0, yaw);
    /// set rotational element of the transform
    transform.setRotation(q);
    /// broadcast the transform information
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));
    /// call all the callbacks waiting to be called
    ros::spinOnce();
    /// se
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
