


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
#
#############################################################################


add_library(Planner OBJECT Planner.cpp)
add_library(Actor OBJECT Actor.cpp)
add_executable(demo Main.cpp Planner Actor)
include_directories(
    ${CMAKE_SOURCE_DIR}/includes
)
#############################################################################
#Find OpenGL and GLUT
#############################################################################
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

target_link_libraries(demo ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )
