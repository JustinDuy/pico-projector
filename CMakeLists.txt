cmake_minimum_required(VERSION 2.8.7 FATAL_ERROR)
project(handprojector_calibration)
find_package(ROS REQUIRED)
find_package(OpenCV REQUIRED)
if( OpenCV_FOUND )
list( APPEND ThirdParty_LIBS ${OpenCV_LIBS} )
    include_directories( ${OpenCV_INCLUDE_DIRS} )
endif( OpenCV_FOUND )


find_package(Qt4 REQUIRED)

include(${QT_USE_FILE})

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
    src/graycode.cpp
)


add_executable(${PROJECT_NAME}_main src/main.cpp)
target_link_libraries(${PROJECT_NAME}_main ${PROJECT_NAME} ${OpenCV_LIBS} ${QT_LIBRARIES})

install(DIRECTORY data DESTINATION ${CMAKE_BINARY_DIR})
