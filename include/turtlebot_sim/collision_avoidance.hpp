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

/*
 * @file collision_avoidance.hpp
 * @Copyright (c) 2019 Ashwin Varghese Kuruttukulam
 * @author Ashwin Varghese Kuruttukulam
 * @brief This file contains a class which implements
 * collision avoidance in the turtle bot sim
 *
 */
#ifndef INCLUDE_TURTLEBOT_SIM_COLLISION_AVOIDANCE_HPP_
#define INCLUDE_TURTLEBOT_SIM_COLLISION_AVOIDANCE_HPP_
#pragma once

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

/**
 * @brief A class which implements
 * collision avoidance in the turtle bot sim

 */

class collision_avoidance {
 private:
  // ROS node handle
  ros::NodeHandle n;
  // ROS subscriber for scan
  ros::Subscriber sub;
  // ROS publisher to the twist function
  ros::Publisher pub;
  // Boolean operator for the obstacle occupancy
  bool obstacleDetect;
  // Float value with the nearest obstacle distance
  float minDist;
  // Twist message
  geometry_msgs::Twist twist;

 public:
  /**
   * @brief Constructor for the collision_avoidance class.
   * @param None.
   * @return None.
   */
  collision_avoidance();
  /**
   * @brief Destructor for the collision_avoidance class.
   * @param None.
   * @return None.
   */
  virtual ~collision_avoidance();

  /**
   * @brief Initialize function for the twist publisher.
   * @param None.
   * @return None.
   */
  void initializePubTwist();
  /**
   * @brief Initialize function for the scan subscriber.
   * @param None.
   * @return None.
   */
  void initializeSubScan();
  /**
   * @brief Function to check the obstacle point, status and
   * moves according to the obstacle present.
   * @param obstacleDetect Status of obstacle value with respect
   * to the bot.
   * @return None.
   */
  void moveBot(bool obstacleDetect);
  /**
   * @brief Callback for the laserscan of the robot and changes the angular velocity
   * @param scanVals Scan value pointer from the message.
   * @return None.
   */
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scanVals);
};
#endif /* INCLUDE_TURTLEBOT_SIM_COLLISION_AVOIDANCE_HPP_ */
