/*
 *MIT License
 *
 *  Copyright (c) 2017 Banuprathap Anandan
 *
 *  AUTHOR : BANUPRATHAP ANANDAN
 *  AFFILIATION : UNIVERSITY OF MARYLAND, MARYLAND ROBOTICS CENTER
 *  EMAIL : BPRATHAP@UMD.EDU
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *
 *
 *
 *  Program: Simulator for a rectangular robot
 *
 */
#include "Actor.hpp"
/**
 * @brief      Constructs the object.
 */
RobotSimulator::RobotSimulator(void) {
  // initialize robot position
  _robot._x = _robot._y = 100;
  _robot._theta = 0;
  InitializeRobot();
  // initialize goal X
  _circles.push_back(800);
  // initialize goal Y
  _circles.push_back(500);
  // initialize goal Radius
  _circles.push_back(20.0);
}
/**
 * @brief      Destroys the object.
 */
RobotSimulator::~RobotSimulator(void) {
}

/**
 * @brief      Returns closest point on the obstacle i, from given point
 *
 * @param[in]  i     Obstacle number
 * @param[in]  x     x coordinate of the point of interest
 * @param[in]  y     y coordinate of the point of interest
 *
 * @return     Returns a structure variable
 */
Point RobotSimulator::ClosestPointOnObstacle(const int i, const double x,
    const double y) {
  const double cx = _circles[3 + 3 * i];
  const double cy = _circles[4 + 3 * i];
  const double r  = _circles[5 + 3 * i];
  const double d  = sqrt((cx - x) * (cx - x) + (cy - y) * (cy - y));
  Point p;
  p._x = cx + r * (x - cx) / d;
  p._y = cy + r * (y - cy) / d;
  return p;
}
/**
 * @brief      Initializes the robot
 */
void RobotSimulator::InitializeRobot() {
  _robot._initVertices.push_back(80.0);
  _robot._initVertices.push_back(85);
  _robot._initVertices.push_back(120.0);
  _robot._initVertices.push_back(85);
  _robot._initVertices.push_back(120.0);
  _robot._initVertices.push_back(115);
  _robot._initVertices.push_back(80.0);
  _robot._initVertices.push_back(115);
  _robot._currVertices = _robot._initVertices;
}

/**
 * @brief      Determines if it has robot reached goal.
 *
 * @return     True if has robot reached goal, False otherwise.
 */
bool RobotSimulator::HasRobotReachedGoal(void) {
  const double gx = GetGoalCenterX();
  const double gy = GetGoalCenterY();
  const double rx = GetRobotX();
  const double ry = GetRobotY();
  return
    sqrt((gx - rx) * (gx - rx) + (gy - ry) * (gy - ry)) <= GetGoalRadius();
}
