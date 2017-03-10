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

#ifndef INCLUDES_ACTOR_HPP_
#define INCLUDES_ACTOR_HPP_
#include <iostream>
#include <cmath>
#include <vector>
/**
 * @brief      Structure to store a 2D point
 */
struct Point {
  double _x;
  double _y;
};
/**
 * @brief      Class for robot simulator.
 */
class RobotSimulator {
  public:
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
     * @brief      Returns the number obstacles.
     *
     * @return     The number obstacles.
     */
    int GetNrObstacles(void) const {
      return _circles.size() / 3 - 1;
    }

    /**
     * @brief      Returns the robot x coordinate.
     *
     * @return     The robot x.
     */
    double GetRobotX(void) {
      return _robot._x;
    }
    /**
     * @brief      Returns the robot y coordinate.
     *
     * @return     The robot y.
     */
    double GetRobotY(void) {
      return _robot._y;
    }
    /**
     * @brief      Returns the robot angle.
     *
     * @return     The robot theta.
     */
    double GetRobotTheta(void) {
      return _robot._theta;
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


  private:
    void InitializeRobot();
    /**
     * A vector to store obstacles information
     */

    std::vector<double> _circles;
    /**
     * @brief      Structure to store Robot information
     */
    struct Robot {
      std::vector<double> _initVertices;
      std::vector<double> _currVertices;
      double              _x;
      double              _y;
      double              _theta;
    };

    Robot _robot;
};

#endif  // INCLUDES_ACTOR_HPP_
