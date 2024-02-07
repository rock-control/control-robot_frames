rock_library(robot_frames
    SOURCES RobotFrames.cpp
    HEADERS RobotFrames.hpp robot_framesTypes.hpp
    DEPS_PKGCONFIG base-types base-logging eigen3 kdl_parser)

