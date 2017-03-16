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
 *  Program: Potential field path planner
 *
 */
/**
 *  @file Planner.hpp
 *  @brief Contains declarations of planner API
 *
 *  This file contains path planner's funtionalities.
 *  Refer Planner.cpp for implementation of these functions.
 *
 *
 *
 *
 *
 *  @author Banuprathap Anandan
 *  @date   03/14/2017
*/

#ifndef INCLUDES_PLANNER_HPP_
#define INCLUDES_PLANNER_HPP_
#include <limits>
#include "Actor.hpp"

/**
 * @brief      Structure to store a move command
 */
struct RobotMove {
  double m_dx;  /**< Movement along X axis. */
  double m_dy;  /**< Movement along Y axis. */
  double m_dtheta;  /**< Rotation by dtheta. */
  double m_speed;  /**< Linear speed of the robot. */
};
/**
 * @brief      Class for RobotPlanner.
 */
class RobotPlanner {
public:
    /**< Obstacles beyond this are omitted in calculating potentials. */
    int distThreshold = 50;
    /**<  Degree of calculating potential. */
    int k = 3;
    /**< Scaling factor for attractive potential. */
    double attPotScaling = 20000;
    /**< Scaling factor for repulsive potential. */
    double repPotScaling = 30000;
    /**< Minimum attractive potential at any point. */
    double minAttPot = 0.5;



    explicit RobotPlanner(RobotSimulator * const simulator);

    ~RobotPlanner(void);

    RobotMove NextMove(void);

 private:
    RobotSimulator *p_simulator;
};

#endif  // INCLUDES_PLANNER_HPP_
