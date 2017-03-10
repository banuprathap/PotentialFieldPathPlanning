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
 *  Program: Potentail field Path Planning in C++
 *
 */
#include <GL/glut.h>
#include <Actor.hpp>
#include <Planner.hpp>
#define PI 3.1415926535897932384626433832795

RobotSimulator _simulator;
RobotPlanner _planner;
/**
 * @brief      Draws a circle.
 *
 * @param[in]  cx    x coordinate of circle center
 * @param[in]  cy    y coordinate of circle center
 * @param[in]  r     radius of circle
 */
void DrawCircle(const double cx, const double cy, const double r) {
  const int    nsides = 50;
  const double angle  = 2 * M_PI / nsides;
  glBegin(GL_POLYGON);
  for (int i = 0; i <= nsides; i++)
    glVertex2d(cx + r * cos(i * angle), cy + r * sin(i * angle));
  glEnd();
}

/**
 * @brief      Handle for display call back
 */
void HandleEventOnDisplay(void) {
  std::vector<double> vert = _simulator.GetRobotVertices();
  glColor3f(1, 0, 0);
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
  glColor3f(0, 0, 1);
  std::vector<double> _obst = _simulator.GetObstacles();
  for (int i = 0; i < _simulator.GetNrObstacles(); ++i)
    DrawCircle(_obst[3 + 3 * i],
               _obst[4 + 3 * i],
               _obst[5 + 3 * i]);
  glutSwapBuffers();
}
/**
 * @brief      Call back for an event on display
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
  glOrtho(-22, 22, -14, 14, -1.0, 1.0);
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
}
/**
 * @brief      Call back for an event on mouse movement with a click
 *
 * @param[in]  x     x position of mouse coordinate
 * @param[in]  y     y position of mouse coordinate
 */
void CallbackEventOnMouseMove(int x, int y) {
}

/**
 * @brief      Call back for an event on timer overflow, for animation.
 *
 * @param[in]  dummy  I still don't understand the use of this, but it's required
 */
void CallbackEventOnTimer(int dummy) {
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
}

/**
 * @brief      Main display loop
 */
void DisplayLoop(void) {
//  create window
  static int    argc = 1;
  static char  *args = const_cast<char*>("args");
  glutInit(&argc, &args);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowSize(1000, 600);
  glutInitWindowPosition(100, 100);
  glutCreateWindow("Potenial Field Path Planner");
  // register callback functions
  glutDisplayFunc(CallbackEventOnDisplay);
  // Callback for mouse click
  glutMouseFunc(CallbackEventOnMouseClick);
  // Callback for mouse cursor movement
  glutMotionFunc(CallbackEventOnMouseMove);
  // Callback for timer. To refresh the window
  glutTimerFunc(15, CallbackEventOnTimer, 0);
  glutIdleFunc(NULL);
  glutKeyboardFunc(CallbackEventOnKeyPress);
  // enter GLUT event processing cycle
  glutMainLoop();
}

/**
 * @brief      Main function in the program. Execution starts here.
 *
 * @param[in]  argc  The argc
 * @param      argv  The argv
 *
 * @return     returns 0 upon sucessful execution
 */
int main(int argc, char **argv) {
  DisplayLoop();
  return 0;
}


