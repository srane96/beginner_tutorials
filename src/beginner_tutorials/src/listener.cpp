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
