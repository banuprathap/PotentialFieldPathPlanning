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
/**
*  @file Actor.hpp
*  @brief Contains declarations of simulator API
*
*  This file contains simulators's funtionalities.
*  Refer Actor.cpp for implementation of these functions.
*
*
*
*
*
*  @author Banuprathap Anandan
*  @date   03/14/2017
*/

#ifndef INCLUDES_ACTOR_HPP_
#define INCLUDES_ACTOR_HPP_
#define UNUSED(x) (void)(x)
#include <iostream>
#include <cmath>
#include <vector>
#define PI 3.1415926535897932384626433832795
/**
 * @brief      Structure to store a 2D point
 */
struct Point {
  double _x;
  double _y;
};
/**
 * @brief      Class for RobotSimulator.
 */
class RobotSimulator {
 public:
    /**
     * A vector to store obstacles information
     */

    std::vector<double> _circles;
    RobotSimulator(void);

    ~RobotSimulator(void);
    /**
     * @brief      Returns the goal center x.
     *
     * @return     The goal center x.
     */
    double GetGoalCenterX(void) {
      return _circles[0];
    }
    /**
     * @brief      Returns the goal center y.
     *
     * @return     The goal center y.
     */
    double GetGoalCenterY(void) {
      return _circles[1];
    }
    /**
     * @brief      Returns the goal radius.
     *
     * @return     The goal radius.
     */
    double GetGoalRadius(void)  {
      return _circles[2];
    }

    /**
     *@brief Returns closest point from (x,y) for a given i'th obstacle
     */
    Point ClosestPointOnObstacle(const int i, const double x, const double y);
    /**
     * @brief      Returns the number of obstacles.
     *
     * @return     The number obstacles in the environment.
     */
    int GetNrObstacles(void) const {
      return _circles.size() / 3 - 1;
    }

    /**
     * @brief      Returns the robot's x coordinate.
     *
     * @return     The robot x.
     */
    double GetRobotX(void) {
      return _robot._x;
    }
    /**
     * @brief      Returns the robot's y coordinate.
     *
     * @return     The robot y.
     */
    double GetRobotY(void) {
      return _robot._y;
    }
    /**
     * @brief      Returns the robot's angle of orientation.
     *
     * @return     The robot theta.
     */
    double GetRobotTheta(void) {
      return _robot._theta;
    }
    /**
     * @brief      Returns the robot length.
     *
     * @return     The robot size x.
     */
    double GetRobotSizeX(void) {
      return _robot._sizeX;
    }
    /**
     * @brief      Returns the robot height.
     *
     * @return     The robot size y.
     */
    double GetRobotSizeY(void) {
      return _robot._sizeY;
    }
    /**
     * @brief      Gets the robot speed.
     *
     * @return     The robot speed.
     */
    double GetRobotSpeed(void) {
      return _robot._maxSpeed;
    }
    /**
     * @brief      Gets the robot accel.
     *
     * @return     The robot accel.
     */
    double GetRobotAccel(void) {
      return _robot._maxAccel;
    }
    /**
     * @brief      Gets the robot turn.
     *
     * @return     The robot turn.
     */
    double GetRobotTurn(void) {
      return _robot._maxTurn;
    }
    /**
     * @brief      Returns the robot vertices.
     *
     * @return     The robot vertices as a vector
     */
    std::vector<double> GetRobotVertices(void) const {
      return _robot._currVertices;
    }
    /**
     * @brief      Returns the obstacles.
     *
     * @return     The obstacles as a vector.
     */
    std::vector<double> GetObstacles(void) const {
      return _circles;
    }
    /**
         * @brief      Determines if it has robot reached goal.
         *
         * @return     True if has robot reached goal, False otherwise.
         */
    bool HasRobotReachedGoal(void);
    /**
     * @brief      Determines if given point lies in the obstacle region
     *
     * @param[in]  x     x coordinate of POI
     * @param[in]  y     y coordinate of POI
     *
     * @return     True if colliding, False otherwise.
     */
    bool isColliding(const double x, const double y);
    /**
     * @brief      Structure to store robot information
     */
    struct Robot {
      std::vector<double> _initVertices; /**< Initial Robot vertices. */
      std::vector<double> _currVertices; /**< Current Robot vertices. */
      double              _x; /**< Robot position along X axis. */
      double              _y;   /**< Robot position along Y axis. */
      double              _theta;   /**< Robot orientation. */
      double              _sizeX;  /**< Robot Length. */
      double              _sizeY;   /**< Robot height. */
      double              _maxSpeed;  /**< Maximum safe speed for the Robot. */
      double        _maxAccel; /**< Maximum safe acceleration for the Robot. */
      double        _maxTurn; /**< Maximum safe turning angle for the Robot. */
    };
    /**
     * @brief      Adds changes to robot configuration.
     *
     * @param[in]  dx      move along x axis by dx
     * @param[in]  dy      move along y axis by dy
     * @param[in]  dtheta  rotate by  dtheta
     * @return none
     */
    void AddToRobotConfiguration(const double dx, const double dy,
                                 const double dtheta);
    /**
     * @brief      Renders robot corners for display
     */
    void RenderRobot();

 private:
    Robot _robot;
};

#endif  // INCLUDES_ACTOR_HPP_
