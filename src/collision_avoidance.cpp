/**
 *
 * MIT License
 *
 * Copyright (c) 2019 Ashwin Varghese Kuruttukulam
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file collision_avoidance.cpp
 * @author Ashwin Varghese Kuruttukulam
 * @Copyright (c) 2019 Ashwin Varghese Kuruttukulam
 * @brief Implementation of the collision_avoidance class.
 */

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <iterator>
#include <algorithm>

// #include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "../include/turtlebot_sim/collision_avoidance.hpp"

/**
 * @brief Constructor to initialize the values of the class
 * @param None.
 * @return None.
 */
collision_avoidance::collision_avoidance() {
  // Initialize the obstacle status value
  obstacleDetect = false;
  // Initialize the min distance value.
  minDist = 0.0;
}
/**
 * @brief Initialize the publisher for the twist node
 * @param None.
 * @return None.
 */
void collision_avoidance::initializePubTwist() {
  // Initialize the publisher to publish the twist messages
  pub = n.advertise < geometry_msgs::Twist
      > ("/mobile_base/commands/velocity", 100);
  // Stream the update message
  ROS_INFO_STREAM("Started the Publisher");
}
/**
 * @brief Initialize he Subscriber for the laser scan node
 * @param None.
 * @return None.
 */
void collision_avoidance::initializeSubScan() {
  // Initialize the subsciber to get the values from the Laser scan.
  sub = n.subscribe("scan", 100, &collision_avoidance::scanCallback, this);
  // Stream the updated message.
  ROS_INFO_STREAM("Started the Subscriber");
}
/**
 * @brief Call back function for the laser scan and check for the minimum
 * distance of the object and the bot and generate movement functions accordingly.
 * @param scanVals - The scan values from the laser scan.
 * @return None.
 */
void collision_avoidance::scanCallback(
    const sensor_msgs::LaserScan::ConstPtr &scanVals) {
  obstacleDetect = false;
  minDist = *(scanVals->ranges.begin());
  // To get the nearest value of the obstacle by iterating through the values.
  for (auto i : scanVals->ranges) {
    if (i < minDist && !std::isnan(i)) {
      // Set the smallest value to the minDist.
      minDist = i;
    }
  }
  // Check if the obstacle not in the search space.
  if (std::isnan(minDist)) {
    ROS_INFO_STREAM("No Obstacle");
  } else {
    ROS_INFO_STREAM("The distance thrshod is : \t" << minDist);
  }
  // Collision distance check for the robot.
  if (minDist < scanVals->range_min + 0.5 && !std::isnan(minDist)) {
    obstacleDetect = true;
    ROS_WARN_STREAM("An object is in the path");
    ROS_INFO_STREAM("Turning ...");
  }
  // Function to run the robot based on the object point.
  moveBot(obstacleDetect);
}
/**
 * @brief Function to move the robot according to the object status.
 * @param obstacleDetect - Collison status with respect to the robot.
 * @return None.
 */
void collision_avoidance::moveBot(bool obstcaleDetect) {
  if (obstcaleDetect) {
    // Angular movement of the robot.
    twist.linear.x = 0;
    twist.angular.z = 1.0;
  } else {
    // Move in the forward direction.
    twist.linear.x = 0.4;
    twist.angular.z = 0.0;
  }
  // publish the values to the robot to follow.
  pub.publish(twist);
}
/**
 * @brief Destructor to the class
 * @param None.
 * @return None.
 */
collision_avoidance::~collision_avoidance() {
}
