rock_library(robot_frames
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(robot_frames_bin Main.cpp
    DEPS robot_frames)

