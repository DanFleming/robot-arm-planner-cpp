cmake_minimum_required(VERSION 3.10)
project(robot_arm_planner_cpp VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

add_executable(robot_arm_planner
    src/main.cpp
)

add_executable(test_vectors
    src/test_vectors.cpp
)

add_executable(test_matrices
    src/test_matrices.cpp
)

add_executable(test_se2
    src/test_se2.cpp
)

add_executable(test_se3
    src/test_se3.cpp
)

add_executable(test_robot_arm_2d
    src/test_robot_arm_2d.cpp
)