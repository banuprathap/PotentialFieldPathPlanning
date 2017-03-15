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
 *  @file Planner.cpp
 *  @brief Contains Implementation of planner API
 *
 *  This file contains path planner's funtionalities.
 *  It can be accessed by the objects of this class.
 *  This class assumes no initial knowledge of the environment.
 *
 *
 *
 *
 *
 *  @author Banuprathap Anandan
 *  @date   03/14/2017
*/
#include "Planner.hpp"

/**
 * @brief      Destroys the object.
 */
RobotPlanner::~RobotPlanner(void) {
}
/**
 * @brief      Constructs the object.
 */
RobotPlanner::RobotPlanner(RobotSimulator* simulator) {
  p_simulator = simulator;
}
/**
 * @brief      Returns the next move to the simulator
 *
 * @return     Returns a structure variable
 */
double Distance(Point One, Point Two) {
  return sqrt((One._x - Two._x) * (One._x - Two._x) + (One._y - Two._y) *
              (One._y - Two._y));
}

/*RobotMove RobotPlanner::NextMove(void) {
  RobotMove move;
  Point closest;
  Point R, G, O;
  R._x = p_simulator->GetRobotX();
  R._y = p_simulator->GetRobotY();
  _theta = p_simulator->GetRobotTheta();
  G._x = p_simulator->GetGoalCenterX();
  G._y = p_simulator->GetGoalCenterY();
  goalRad = p_simulator->GetGoalRadius();
  int n = p_simulator->GetNrObstacles();
  distanceGoal = Distance(R, G);
  angleGoal = atan2(G._y - R._y, G._x - R._x);
  for (int i = 0; i <= n; i++) {
    O = p_simulator->ClosestPointOnObstacle(i, R._x, R._y);
    if (Distance(R, O) < 50) {

    }
  }
  return move;
}
*/
RobotMove RobotPlanner::NextMove(void) {
  RobotMove move;
  if (!p_simulator->HasRobotReachedGoal()) {
    double distanceGoal, currDirection, angleGoal;
    double steer, robotSpeed, speed;
    double maxTurn, maxAcceleration, maxRobotSpeed;
    double sizeX, sizeY, sizeHalfDiag;
    double i;
    double repPotX, repPotY, attPotX, attPotY, totPotX, totPotY;
    Point R, G;
    R._x = p_simulator->GetRobotX();
    R._y = p_simulator->GetRobotY();
    currDirection = p_simulator->GetRobotTheta();
    maxTurn = p_simulator->GetRobotTurn();
    maxAcceleration = p_simulator->GetRobotAccel();
    maxRobotSpeed = p_simulator->GetRobotSpeed();
    G._x = p_simulator->GetGoalCenterX();
    G._y = p_simulator->GetGoalCenterY();
    sizeX  = p_simulator->GetRobotSizeX();
    sizeY = p_simulator->GetRobotSizeY();
    sizeHalfDiag = sqrt(pow(sizeX / 2, 2) + pow(sizeY / 2, 2));
    //  speed
    robotSpeed = 10;
    //  Calculate distance from obstacle at front
    i = sizeX / 2 + 1;
    while (1) {
      //  std::cout << i << std::endl;
      double _x = (R._x + i * sin(currDirection));
      double _y = (R._y + i * cos(currDirection));
      //  std::cout << _x << "\t" << _y << std::endl;
      if (p_simulator->isColliding(_x, _y)) {
        std::cout << _x << "\t" << _y << std::endl;
        break;
      }
      i++;
    }  //  std::cout << "check 1" << std::endl;
    double distanceFront = i - sizeX / 2;
    //  std::cout << distanceFront << std::endl;
    //  Calculate distance from obstacle at left
    i = sizeY / 2 + 1;
    while (1) {
      //  std::cout << i << std::endl;
      double _x = (R._x + i * sin(currDirection - PI / 2));
      double _y = (R._y + i * cos(currDirection - PI / 2));
      //  std::cout << _x << "\t" << _y << std::endl;
      if (p_simulator->isColliding(_x, _y)) {
        std::cout << _x << "\t" << _y << std::endl;
        break;
      }
      i++;
    }   //  std::cout << "check 2" << std::endl;
    double distanceLeft = i - sizeY / 2;
    //  std::cout << distanceLeft << std::endl;
    //  Calculate distance from obstacle at Right
    i = sizeY / 2 + 1;
    while (1) {
      double _x = (R._x + i * sin(currDirection + PI / 2));
      double _y = (R._y + i * cos(currDirection + PI / 2));
      //  std::cout << _x << "\t" << _y << std::endl;
      if (p_simulator->isColliding(_x, _y)) {
        std::cout << _x << "\t" << _y << std::endl;
        break;
      }
      i++;
    }   //  std::cout << "check 3" << std::endl;
    double distanceRight = i - sizeY / 2;
    //  std::cout << distanceRight << std::endl;
    //  Calculate distance from obstacle at Front-left-diagonal
    i = sizeHalfDiag  + 1;
    while (1) {
      double _x = (R._x + i * sin(currDirection - PI / 4));
      double _y = (R._y + i * cos(currDirection - PI / 4));
      if (p_simulator->isColliding(_x, _y)) {
        std::cout << _x << "\t" << _y << std::endl;
        break;
      }
      i++;
    }  //  std::cout << "check 4" << std::endl;
    double distanceFrontLeftDiagonal = i - sizeHalfDiag;
    //  std::cout << distanceFrontLeftDiagonal << std::endl;
    //  Calculate distance from obstacle at Front-left-diagonal
    i = sizeHalfDiag + 1;
    while (1) {
      double _x = (R._x + i * sin(currDirection + PI / 4));
      double _y = (R._y + i * cos(currDirection + PI / 4));
      if (p_simulator->isColliding(_x, _y)) {
        //  std::cout << _x << "\t" << _y << std::endl;
        break;
      }
      i++;
    }  //  std::cout << "check 5" << std::endl;
    double distanceFrontRightDiagonal = i -  sizeHalfDiag;
    //  std::cout << distanceFrontRightDiagonal << std::endl;
    //  Goal
    distanceGoal = sqrt((R._x - G._x) * (R._x - G._x) + (R._y - G. _y) *
                        (R._y - G._y));
    angleGoal = atan2(G._y - R._y, G._x - R._x);
    //  Potential
    repPotY = 1.0 / pow(distanceFront, k) * sin(currDirection) +
              1.0 / pow((distanceLeft), k) * sin(currDirection - PI / 2) +
              1.0 / pow((distanceRight), k) * sin(currDirection + PI / 2) +
              1.0 / pow((distanceFrontLeftDiagonal), k) * sin(currDirection - PI / 4) +
              1.0 / pow((distanceFrontRightDiagonal), k) * sin(currDirection + PI / 4);
    repPotX = 1.0 / pow((distanceFront), k) * cos(currDirection) +
              1.0 / pow((distanceLeft), k) * cos(currDirection - PI / 2) +
              1.0 / pow((distanceRight), k) * cos(currDirection + PI / 2) +
              1.0 / pow((distanceFrontLeftDiagonal), k) * cos(currDirection - PI / 4) +
              1.0 / pow((distanceFrontRightDiagonal), k) * cos(currDirection + PI / 4);
    attPotY = std::max(pow((1.0 / distanceGoal), k),
                       minAttPot) * sin(angleGoal)  * attPotScaling;
    attPotX = std::max(pow((1.0 / distanceGoal), k) ,
                       minAttPot) * cos(angleGoal) * attPotScaling ;
    totPotX = attPotX - (repPotScaling * repPotX);
    totPotY = attPotY - (repPotScaling * repPotY);
    //  dTheta
    steer = atan2(robotSpeed * cos(currDirection) + totPotX,
                  robotSpeed * sin(currDirection) + totPotY) - currDirection;
    while (steer > PI) {
      steer -= 2 * PI;
    }  //  check to get the angle between -pi and pi
    while (steer < -PI) {
      steer += 2 * PI;
    }  //  check to get the angle between - pi and pi
    steer = std::min(maxTurn, steer);
    steer = std::max(-maxTurn, steer);
//  set new speed
    speed = sqrt((robotSpeed * sin(currDirection) + totPotY) *
                 (robotSpeed * sin(currDirection) + totPotY) +
                 (robotSpeed * cos(currDirection) + totPotX) *
                 (robotSpeed * cos(currDirection) + totPotX));
    speed = std::min(robotSpeed + maxAcceleration, speed);
    robotSpeed = std::max(robotSpeed - maxAcceleration, speed);
    robotSpeed = std::min(robotSpeed, maxRobotSpeed);
    robotSpeed = std::max(robotSpeed, 0.0);
    if (robotSpeed == 0)
      std::cout << "robot had to stop to avoid collission" << std::endl;
    move.m_dx = robotSpeed * sin(currDirection);
    move.m_dy = robotSpeed * cos(currDirection);
    move.m_speed = robotSpeed;
    move.m_dtheta = steer;
  } else {
    std::cout << "DONE" << std::endl;
  }
  return move;
}
