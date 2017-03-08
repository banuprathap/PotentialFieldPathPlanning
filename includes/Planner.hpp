/* Copyright 2017 Banuprathap Anandan*/

#ifndef INCLUDES_PLANNER_HPP_
#define INCLUDES_PLANNER_HPP_

#include "Actor.hpp"

struct RobotMove {
  double m_dx;
  double m_dy;
  double m_dtheta;
};

class RobotPlanner {
 public:
    RobotPlanner(void);

    ~RobotPlanner(void);

    RobotMove NextMove(void);
};

#endif  // INCLUDES_PLANNER_HPP_
