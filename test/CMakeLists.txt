set(GTEST_SHUFFLE 1)



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
#
#############################################################################
add_executable(
    cpp-test
    main.cpp
    test.cpp
    $<TARGET_OBJECTS:Planner>
    $<TARGET_OBJECTS:Actor>
)

include_directories(${CMAKE_SOURCE_DIR}/includes)
target_include_directories(cpp-test PUBLIC ../vendor/googletest/googletest/include)
target_link_libraries(cpp-test PUBLIC gtest)