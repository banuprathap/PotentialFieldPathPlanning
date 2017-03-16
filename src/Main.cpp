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
 *  @file Main.cpp
 *  @brief Initial file of path planner program
 *
 *  This file contains path planner's
 *  main() function. All GUI functions are implemented here.
 *
 *
 *
 *
 *  @author Banuprathap Anandan
 *  @date   03/14/2017
*/
#include <GL/glut.h>
#include <Actor.hpp>
#include <Planner.hpp>

#define ATTEMPTS 50000

RobotPlanner   *_planner;
RobotSimulator _simulator;

int _selectedCircle;
int attempt;
bool _plan;
/**
 * @brief      Draws a circle.
 *
 * @param[in]  cx    x coordinate of circle center
 * @param[in]  cy    y coordinate of circle center
 * @param[in]  r     radius of circle
 */
void DrawCircle(const double cx, const double cy, const double r) {
  const int    nsides = 50;
  const double angle  = 2 * PI / nsides;
  glBegin(GL_POLYGON);
  for (int i = 0; i <= nsides; i++)
    glVertex2d(cx + r * cos(i * angle), cy + r * sin(i * angle));
  glEnd();
}

/**
 * @brief      Handle for display call back
 * @param  none
 * @return none
 */
void HandleEventOnDisplay(void) {
  std::vector<double> vert = _simulator.GetRobotVertices();
  glColor3f(0, 0.5, 1);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glBegin(GL_QUADS);
  glVertex2f(vert[0], vert[1]);
  glVertex2f(vert[2], vert[3]);
  glVertex2f(vert[4], vert[5]);
  glVertex2f(vert[6], vert[7]);
  glEnd();
  glColor3f(0, 1, 0);
  DrawCircle(_simulator.GetGoalCenterX(), _simulator.GetGoalCenterY(),
             _simulator.GetGoalRadius());
  glColor3f(1, 0, 0);
  for (int i = 0; i < _simulator.GetNrObstacles(); ++i)
    DrawCircle(_simulator._circles[3 + 3 * i],
               _simulator._circles[4 + 3 * i],
               _simulator._circles[5 + 3 * i]);
  glutSwapBuffers();
}

/**
 * @brief      Function to handle an event on mouse click. It also checks if the cursor is inside any obstacle region during the event
 *
 * @param[in]  whichBtn   The which button
 * @param[in]  mousePosX  The mouse position x
 * @param[in]  mousePosY  The mouse position y
 */
void HandleEventOnMouseClick(const int whichBtn, const double mousePosX,
                             const double mousePosY) {
  if (whichBtn == GLUT_LEFT_BUTTON) {
    _selectedCircle = -1;
    for (size_t i = 0; i < _simulator._circles.size()
         && _selectedCircle == -1; i += 3) {
      const double _x = _simulator._circles[i];
      const double _y = _simulator._circles[i + 1];
      const double _r  = _simulator._circles[i + 2];
      const double _d  = sqrt((mousePosX - _x) * (mousePosX - _x) +
                              (mousePosY - _y) * (mousePosY - _y));
      if (_d <= _r)
        _selectedCircle = i / 3;
    }
    if (_selectedCircle == -1) {
      _simulator._circles.push_back(mousePosX);
      _simulator._circles.push_back(mousePosY);
      _simulator._circles.push_back(40.0);
    }
  }
}
/**
 * @brief      Call back for an event on display
 * @param  none
 * @return none
 */
void CallbackEventOnDisplay(void) {
  glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
  glClearDepth(1.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  glShadeModel(GL_SMOOTH);
  glViewport(0, 0, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  //  glOrtho(-22, 22, -14, 14, -1.0, 1.0);
  glOrtho(0.0f, 500, 500, 0.0f, 0.0f, 1.0f);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  HandleEventOnDisplay();
  glutSwapBuffers();
}
/**
 * @brief      Call back for an event on mouse click
 *
 * @param[in]  whichBtn   The which button
 * @param[in]  state      The state
 * @param[in]  mousePosX  The mouse position x
 * @param[in]  mousePosY  The mouse position y
 */
void CallbackEventOnMouseClick(int whichBtn, int state, int mousePosX,
                               int mousePosY) {
  if (state == GLUT_DOWN) {
    HandleEventOnMouseClick(whichBtn, mousePosX , mousePosY);
    glutPostRedisplay();
  }
}
/**
 * @brief      Call back for an event on mouse movement with a click
 *
 * @param[in]  x     x position of mouse coordinate
 * @param[in]  y     y position of mouse coordinate
 */
void CallbackEventOnMouseMove(int mousePosX, int mousePosY) {
  if (_selectedCircle >= 0) {
    _simulator._circles[3 * _selectedCircle] = mousePosX;
    _simulator._circles[3 * _selectedCircle + 1] = mousePosY;
  }
  glutPostRedisplay();
}
/**
 * @brief      Handle for Timer Overflow
 * @param  none
 * @return none
 */
void HandleEventOnTimer(void) {
  if (_plan && _simulator.HasRobotReachedGoal()) {
    std::cout << "\n\n Goal Reached!!!!!!!!! \n\n Press escape key to exit";
  }
  if (_plan && !_simulator.HasRobotReachedGoal() && attempt > ATTEMPTS) {
    std::cout << "\n\nStuck in local minima!!!!!\n\nPress escape key to exit";
  }
  if (_plan && !_simulator.HasRobotReachedGoal()) {
    attempt++;
    RobotMove move = _planner->NextMove();
    _simulator.AddToRobotConfiguration(move.m_dx, move.m_dy, move.m_dtheta);
  }
}
/**
 * @brief      Call back for an event on timer overflow, for animation.
 *
 * @param[in]  dummy  It's required to match timers, when multiple timers
 *                    are in use
 */
void CallbackEventOnTimer(int dummy) {
  HandleEventOnTimer();
  glutTimerFunc(30, CallbackEventOnTimer, dummy);
  glutPostRedisplay();
}
/**
 * @brief      Call back for an event on keyboard press
 *
 * @param[in]  key        The key
 * @param[in]  mousePosX  The mouse position x at keypress
 * @param[in]  mousePosY  The mouse position y at keypress
 */
void CallbackEventOnKeyPress(const unsigned char key, const int mousePosX,
                             const int mousePosY) {
  UNUSED(mousePosX);
  UNUSED(mousePosY);
  std::cout << "pressed key = " << key << std::endl;
  switch (key) {
  case 27:  //  escape key
    exit(0);
  case 's':
    _plan = !_plan;
    for (size_t i = 3; i < _simulator._circles.size(); i += 3) {
      const double _x = _simulator._circles[i];
      const double _y = _simulator._circles[i + 1];
      //  const double _r  = _simulator._circles[i + 2];
      std::cout << _x << "\t" << _y << std::endl;
    }
    break;
  }
}
/**
 * @brief      Main display loop
 * @param  none
 * @return nones
 */
void DisplayLoop(void) {
//  create window
  static int    argc = 1;
  static char  *args = const_cast<char*>("args");
  glutInit(&argc, &args);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowSize(500, 500);
  glutInitWindowPosition(100, 100);
  glutCreateWindow("Potenial Field Path Planner");
  // register callback functions
  glutDisplayFunc(CallbackEventOnDisplay);
  // Callback for mouse click
  glutMouseFunc(CallbackEventOnMouseClick);
  // Callback for mouse cursor movement
  glutMotionFunc(CallbackEventOnMouseMove);
  // Callback for timer. To refresh the window
  glutTimerFunc(30, CallbackEventOnTimer, 0);
  glutIdleFunc(NULL);
  glutKeyboardFunc(CallbackEventOnKeyPress);
  // enter GLUT event processing cycle
  glutMainLoop();
}
/**
 * @brief      program entrypoint
 *
 * @param[in]  argc  The argc
 * @param      argv  The argv
 *
 * @return     integer 0 upon exit success \n
             integer -1 upon exit failure
 */
int main() {
  attempt = 0;
  _planner = new RobotPlanner(&_simulator);
  DisplayLoop();
  return 0;
}
