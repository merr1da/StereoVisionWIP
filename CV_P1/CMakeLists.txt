cmake_minimum_required(VERSION 3.5)

project(CV_P1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DOPENCV_ENABLE_NONFREE=ON")

add_executable(CV_P1 main.cpp calibrate.cpp StereoVision.cpp)

include_directories($CMAKE_SOURCE_DIR)

find_package(OpenCV REQUIRED)
target_link_libraries(CV_P1 ${OpenCV_LIBS})

find_package(Qt5Widgets REQUIRED)
target_link_libraries(CV_P1 Qt5::Widgets)
