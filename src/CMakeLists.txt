
add_executable(GulliView GulliView.cpp CameraUtil.h CameraUtil.cpp TagFamily.cpp TagFamily.h)
target_link_libraries(GulliView ${OPENCV_LDFLAGS})
target_link_libraries(GulliView ${Boost_LIBRARIES})
target_link_libraries(GulliView ${APRILTAGS_LDFLAGS})

if (GLUT_LIBRARY)

  add_executable(gltest gltest.cpp)
  target_link_libraries(gltest ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${AT_LIBS})

endif()
