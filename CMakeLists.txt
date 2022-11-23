cmake_minimum_required(VERSION 3.5)

project(SemesterProjekt1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_executable(SemesterProjekt1 main.cpp locate.h locate.cpp calibrate.h calibrate.cpp multipliser.h multipliser.cpp multicam.h multicam.cpp)

find_package(OpenCV REQUIRED)
find_package(ur_rtde REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories( ${OpenCV_INCLUDE_DIRS} )

target_link_libraries(SemesterProjekt1 ${OpenCV_LIBS} ur_rtde::rtde Eigen3::Eigen)






install(TARGETS SemesterProjekt1
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
