cmake_minimum_required(VERSION 3.5)

project(SemesterProjekt1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_PREFIX_PATH "/home/mads/Qt/5.15.2/gcc_64")
set(CMAKE_AUTOMOC ON)

list( APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} )

find_package(Qt5 COMPONENTS Network REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core)
find_package(OpenCV REQUIRED)
find_package(ur_rtde REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

#Pylon
find_package(Pylon REQUIRED)
include_directories(${Pylon_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(SemesterProjekt1 main.cpp locate.h locate.cpp calibrate.h calibrate.cpp kastebane.h kastebane.cpp cam.h cam.cpp Socket.h Socket.cpp)
target_link_libraries(SemesterProjekt1 ${OpenCV_LIBS} ur_rtde::rtde Eigen3::Eigen ${Pylon_LIBRARIES} Qt5::Network Qt5::Core)

install(TARGETS SemesterProjekt1
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
