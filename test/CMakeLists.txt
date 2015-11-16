rock_testsuite(robot_frames_test suite.cpp
   test_RobotFrames.cpp
   DEPS robot_frames)

install(FILES kuka_lbr_no_meshes.urdf DESTINATION share/robot_frames/test)
