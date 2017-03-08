/* Copyright 2017 Banuprathap Anandan*/
#ifndef INCLUDES_ACTOR_HPP_
#define INCLUDES_ACTOR_HPP_

#include <cmath>
#include <vector>

struct Point {
  double _x;
  double _y;
};

class RobotSimulator {
 public:
    RobotSimulator(void);

    ~RobotSimulator(void);

    double GetGoalCenterX(void) {
      return _circles[0];
    }

    double GetGoalCenterY(void) {
      return _circles[1];
    }

    double GetGoalRadius(void)  {
      return _circles[2];
    }

    /**
     *@brief Returns closest point from (x,y) for a given i'th obstacle
     */
    Point ClosestPointOnObstacle(const int i, const double x, const double y);


    double GetRobotX(void) {
      return _robot._x;
    }

    double GetRobotY(void) {
      return _robot._y;
    }

    double GetRobotTheta(void) {
      return _robot._theta;
    }


    bool HasRobotReachedGoal(void);


 private:
    void InitializeRobot();

    std::vector<double> _circles;

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
