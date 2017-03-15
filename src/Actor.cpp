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
 *
 */
/**
*  @file Actor.cpp
*  @brief Contains Implementation of simulator API
*
*  This file contains simulator's funtionalities.
*  It can be accessed by the objects of this class.
*
*
*
*
*
*
*  @author Banuprathap Anandan
*  @date   03/14/2017
*/
#include "Actor.hpp"
/**
 * @brief      Default constructor
 *
 * Constructs the object.
 * @param  none
 * @return none
 */
RobotSimulator::RobotSimulator(void) {
  // initialize robot position
  _robot._x = _robot._y = 50; /**< Initial robot position in x and y. */
  _robot._sizeX = 20; /**< Robot length. */
  _robot._sizeY = 20; /**< Robot height. */
  _robot._theta = 0;   /**< Initial robot orientation. */
  _robot._maxSpeed = 5;  /**< Maximum safe speed for the robot. */
  _robot._maxAccel = 5;  /**< Maximum safe acceleration for the robot. */
  _robot._maxTurn = 10 * PI /
                    180;  /**< Maximum safe turning angle for the robot. */
  double halfDiag = sqrt(_robot._sizeX * _robot._sizeX + _robot._sizeY *
                         _robot._sizeY) / 2;
  _robot._currVertices.push_back(_robot._x + halfDiag * cos(
                                   _robot._theta - PI / 4));
  _robot._currVertices.push_back(_robot._y + halfDiag * sin(
                                   _robot._theta - PI / 4));
  _robot._currVertices.push_back(_robot._x + halfDiag * cos(
                                   _robot._theta + PI / 4));
  _robot._currVertices.push_back(_robot._y + halfDiag * sin(
                                   _robot._theta + PI / 4));
  _robot._currVertices.push_back(_robot._x + halfDiag * cos(
                                   _robot._theta - PI / 4 +
                                   PI));
  _robot._currVertices.push_back(_robot._y + halfDiag * sin(
                                   _robot._theta - PI / 4 +
                                   PI));
  _robot._currVertices.push_back(_robot._x + halfDiag * cos(
                                   _robot._theta + PI / 4 +
                                   PI));
  _robot._currVertices.push_back(_robot._y + halfDiag * sin(
                                   _robot._theta + PI / 4 +
                                   PI));
  // initialize goal X
  _circles.push_back(300);
  // initialize goal Y
  _circles.push_back(30);
  // initialize goal Radius
  _circles.push_back(30.0);
}
/**
 * @brief      Destroys the object.
 * @param  none
 * @return none
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
 * @brief      Determines if colliding.
 *
 * @param[in]  x     { parameter_description }
 * @param[in]  y     { parameter_description }
 *
 * @return     True if colliding, False otherwise.
 */
bool RobotSimulator::isColliding(const double x, const double y) {
  double sizeDiag = sqrt(_robot._sizeX * _robot._sizeX + _robot._sizeY *
                         _robot._sizeY) / 2;
  bool col = false;
  if ( x < 1 + sizeDiag || x > 500 - sizeDiag || y < 1 +  sizeDiag
       || y > 500 - sizeDiag ) {
    col = true;
  } else {
    for (int i = 3; i < _circles.size(); i += 3) {
      const double _x = _circles[i];
      const double _y = _circles[i + 1];
      const double _r = _circles[i + 2];
      const double dist = sqrt((x - _x) * (x - _x) + (y - _y) * (y - _y));
      if (dist <= _r - sizeDiag) {
        col = true;
        std::cout << col;
        break;
      }
    }
  }
  return col;
}

/**
 * @brief      Updates robot's corner positions
 *
 * @param[in]  _R    struct of type Robot.
 * @return     none
 */

/*
void RobotSimulator::UpdateRobotVertices(struct RobotSimulator::Robot _R) {
  double sizeDiag = sqrt(_R._sizeX * _R._sizeX + _R._sizeY * _R._sizeY);
  _robot._initVertices.push_back(_R._x + sizeDiag * sin(_R._theta - PI / 4));
  _robot._initVertices.push_back(_R._y + sizeDiag * cos(_R._theta - PI / 4));
  _robot._initVertices.push_back(_R._x + sizeDiag * sin(_R._theta + PI / 4));
  _robot._initVertices.push_back(_R._y + sizeDiag * cos(_R._theta + PI / 4));
  _robot._initVertices.push_back(_R._x + sizeDiag *
                                 sin(PI + _R._theta - PI / 4));
  _robot._initVertices.push_back(_R._y + sizeDiag *
                                 cos(PI + _R._theta - PI / 4));
  _robot._initVertices.push_back(_R._x + sizeDiag *
                                 sin(PI + _R._theta + PI / 4));
  _robot._initVertices.push_back(_R._y + sizeDiag *
                                 cos(PI + _R._theta + PI / 4));
  _robot._currVertices.assign(_robot._initVertices.begin(),
                              _robot._initVertices.end());
}
*/
/**
 * @brief      Determines if robot has reached goal.
 * @param      none
 * @return     True if has robot reached goal, False otherwise.
 */
bool RobotSimulator::HasRobotReachedGoal(void) {
  const double gx = GetGoalCenterX();
  const double gy = GetGoalCenterY();
  const double rx = GetRobotX();
  const double ry = GetRobotY();
  return
    sqrt((gx - rx) * (gx - rx) + (gy - ry) * (gy - ry)) <
    GetGoalRadius() - GetRobotSizeX() / 2;
}
/**
 * @brief      Adds changes to robot configuration.
 *
 * @param[in]  dx      move along x axis by dx
 * @param[in]  dy      move along y axis by dy
 * @param[in]  dtheta  rotate by  dtheta
 * @return none
 */
void RobotSimulator::AddToRobotConfiguration(const double dx,
    const double dy, const double dtheta) {
  _robot._x += dx;
  _robot._y += dy;
  _robot._theta += dtheta;
  if (isColliding(_robot._x, _robot._y)) {
    //  throw "Robot Collided";
  }
  RenderRobot();
}
void RobotSimulator::RenderRobot() {
  double halfDiag = sqrt(_robot._sizeX * _robot._sizeX + _robot._sizeY *
                         _robot._sizeY) / 2;
  _robot._currVertices[0] = _robot._x + halfDiag * cos(_robot._theta - PI / 4);
  _robot._currVertices[1] = _robot._y + halfDiag * sin(_robot._theta - PI / 4);
  _robot._currVertices[2] = _robot._x + halfDiag * cos(_robot._theta + PI / 4);
  _robot._currVertices[3] = _robot._y + halfDiag * sin(_robot._theta + PI / 4);
  _robot._currVertices[4] = _robot._x + halfDiag * cos(_robot._theta - PI / 4 +
                            PI);
  _robot._currVertices[5] = _robot._y + halfDiag * sin(_robot._theta - PI / 4 +
                            PI);
  _robot._currVertices[6] = _robot._x + halfDiag * cos(_robot._theta + PI / 4 +
                            PI);
  _robot._currVertices[7] = _robot._y + halfDiag * sin(_robot._theta + PI / 4 +
                            PI);
}

