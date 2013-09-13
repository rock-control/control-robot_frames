rock_library(robot_frames
    SOURCES RobotFrames.cpp
    HEADERS RobotFrames.hpp
    DEPS_PKGCONFIG base-types eigen3 kdl_parser)

rock_executable(make_robot_transformer_file make_robot_transformer_file.cpp
    DEPS robot_frames
    DEPS_PKGCONFIG base-types eigen3 kdl_parser)

