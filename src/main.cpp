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
 * @file main.cpp
 * @author Ashwin Varghese Kuruttukulam
 * @Copyright (c) 2019 Ashwin Varghese Kuruttukulam 
 * @brief The Main class implementation of the class and code.
 */

#include <sstream>
#include <string>
#include <iostream>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "../include/turtlebot_sim/collision_avoidance.hpp"

/**
 * @brief Main block to run the nodes.
 * @param argv-Pointer value to the command line arguments.
 * @param argc-Number of values in the command line arguments.
 * @return Run status
 */

int main(int argc, char **argv) {
  ros::init(argc, argv, "collision_avoidance");
  collision_avoidance move;
//  move.nodeinit(argc,argv);
  move.initializeSubScan();
  move.initializePubTwist();
  ros::spin();
  return 0;
}



