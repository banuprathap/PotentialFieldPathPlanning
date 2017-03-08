PROJECT(Planner)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

#############################################################################
#Find OpenGL and GLUT
#############################################################################
INCLUDE(FindOpenGL)

INCLUDE(FindGLUT)

INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})

SET(INTERACTIVE_LIBS ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} ${GLUT_glut_LIBRARY})

#############################################################################
#Message to console
#############################################################################

MESSAGE("
*****************************************************************************
* REQUIRED EXTERNAL LIBRARIES
*   GL   : ${MsgOpenGL} ${OPENGL_gl_LIBRARY}
*   GLU  : ${MsgOpenGLU} ${OPENGL_glu_LIBRARY}
*   GLUT : ${MsgGLUT} ${GLUT_glut_LIBRARY}
*****************************************************************************
")

#############################################################################
INCLUDE_DIRECTORIES(src includes)

AUX_SOURCE_DIRECTORY(src SRC_FILES)

ADD_EXECUTABLE(Planner ${SRC_FILES})
TARGET_LINK_LIBRARIES(Planner ${INTERACTIVE_LIBS})
