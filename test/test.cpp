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
 *  Program: Potential field Path Planning in C++
 *
 */
/**
 * @file test.cpp
 *
 *
 * @brief Unit test for path planner project
 *
 *  This file contains implementation of unit tests for this project
 *
 * @author Banuprathap Anandan
 * @version 1.0.1
 */

#include <gtest/gtest.h>
#include <Planner.hpp>

TEST(NoObstacleCollisionTest, Collide) {
  RobotSimulator t_simulator;
  ASSERT_TRUE(t_simulator.isColliding(100, 501));
  ASSERT_TRUE(t_simulator.isColliding(-1, 501));
  ASSERT_TRUE(t_simulator.isColliding(-10, -10));
  ASSERT_TRUE(t_simulator.isColliding(0, 0));
  ASSERT_TRUE(t_simulator.isColliding(100, 5001));
}

TEST(NoObstacleCollisionTest, NotCollide) {
  RobotSimulator t_simulator;
  ASSERT_FALSE(t_simulator.isColliding(100, 101));
  ASSERT_FALSE(t_simulator.isColliding(300, 101));
  ASSERT_FALSE(t_simulator.isColliding(50, 16));
  ASSERT_FALSE(t_simulator.isColliding(70, 120));
  ASSERT_FALSE(t_simulator.isColliding(100, 51));
}

TEST(ObstacleCollisionTest, Collide) {
  RobotSimulator t_simulator;
  t_simulator._circles.push_back(50);
  t_simulator._circles.push_back(50);
  t_simulator._circles.push_back(50);
  ASSERT_TRUE(t_simulator.isColliding(75, 75));
  ASSERT_TRUE(t_simulator.isColliding(50, 50));
  ASSERT_TRUE(t_simulator.isColliding(60, 50));
}


TEST(ReachedGoal, NotReach) {
  RobotSimulator t_simulator;
  ASSERT_FALSE(t_simulator.HasRobotReachedGoal());
}


TEST(ReachedGoal, Reach) {
  RobotSimulator t_simulator;
  // _planner = new RobotPlanner(&_simulator);
  double dx =  t_simulator.GetGoalCenterX() - t_simulator.GetRobotX();
  double dy = t_simulator.GetGoalCenterY() - t_simulator.GetRobotY();
  double dtheta = 0;
  t_simulator.AddToRobotConfiguration(dx, dy , dtheta);
  ASSERT_TRUE(t_simulator.HasRobotReachedGoal());
}
