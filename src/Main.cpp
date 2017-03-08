/* Copyright 2017 Banuprathap Anandan*/
#include <GL/glut.h>

void CallbackEventOnDisplay() {
}

int main(int argc, char **argv) {
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
  glutInitWindowSize(1000, 600);
  glutInitWindowPosition(100, 100);
  glutCreateWindow("Potenial Field Path Planner");
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glutSwapBuffers();
  // register callback functions
  glutDisplayFunc(CallbackEventOnDisplay);
  // enter GLUT event processing cycle
  glutMainLoop();
  return 0;
}


