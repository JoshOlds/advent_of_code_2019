cmake_minimum_required(VERSION 2.8.3)

project(advent_of_code_2019)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED
    roscpp
    roslib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS} 
)

catkin_package(
    INCLUDE_DIRS 
        include
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
)

# Library Sources
set(${PROJECT_NAME}_SRCS
    src/adventlib/input_helper.cpp
    src/adventlib/int_code_cpu/int_code_cpu.cpp
    src/adventlib/geometry/grid_coord.cpp
    src/adventlib/geometry/grid_line.cpp
)

# Master package library
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Executables

add_executable(day1 src/day1.cpp)
target_link_libraries(day1 ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(day1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(day2 src/day2.cpp)
target_link_libraries(day2 ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(day2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(day3 src/day3.cpp)
target_link_libraries(day3 ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(day3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(day4 src/day4.cpp)
target_link_libraries(day4 ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(day4 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(day5 src/day5.cpp)
target_link_libraries(day5 ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(day5 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

