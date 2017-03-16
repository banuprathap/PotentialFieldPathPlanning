PROJECT(Planner)

CMAKE_MINIMUM_REQUIRED(VERSION 3.2.1)

include(CMakeToolsHelpers OPTIONAL)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic")
set(CMAKE_CXX_STANDARD 14)

#############################################################################
#Find OpenGL and GLUT
#############################################################################
INCLUDE(FindOpenGL)

INCLUDE(FindGLUT)

INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})

SET(INTERACTIVE_LIBS ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} ${GLUT_glut_LIBRARY})

#############################################################################
add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(vendor/googletest/googletest)

###############################################################################