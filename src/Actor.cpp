/* Copyright 2017 Banuprathap Anandan*/
#include "Actor.hpp"

RobotSimulator::RobotSimulator(void) {
}

RobotSimulator::~RobotSimulator(void) {
}


Point RobotSimulator::ClosestPointOnObstacle(const int i, const double x,
    const double y) {
}

void RobotSimulator::InitializeRobot() {
}

/**
     *@brief Returns true if the robot is inside the goal circle
     */
bool RobotSimulator::HasRobotReachedGoal(void) {
  const double gx = GetGoalCenterX();
  const double gy = GetGoalCenterY();
  const double rx = GetRobotX();
  const double ry = GetRobotY();
  return
    sqrt((gx - rx) * (gx - rx) + (gy - ry) * (gy - ry)) <= GetGoalRadius();
}
