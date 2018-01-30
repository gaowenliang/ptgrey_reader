cmake_minimum_required(VERSION 2.8.3)
project(ptgrey_reader)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native ")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC ")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
#  sys_utils
)

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
include_directories(${Boost_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

set(LIBUSB_1_INCLUDE_DIR
    /usr/local/include/libusb-1.0
)
message(STATUS "Found libusb.h: ${LIBUSB_1_INCLUDE_DIR}")

set(LIBUSB_1_LIBRARY
    /usr/local/lib/libusb-1.0.so
)
message(STATUS "Found libusb 1.0 library: ${LIBUSB_1_LIBRARY}")

if (LIBUSB_1_INCLUDE_DIR AND LIBUSB_1_LIBRARY)
   set(LIBUSB_1_FOUND TRUE)
   set(LIBUSB_1_LIBRARIES ${LIBUSB_1_LIBRARY})
else (LIBUSB_1_INCLUDE_DIR AND LIBUSB_1_LIBRARY)
   set(LIBUSB_1_FOUND FALSE)
endif (LIBUSB_1_INCLUDE_DIR AND LIBUSB_1_LIBRARY)

if (LIBUSB_1_FOUND)
      message(STATUS "Found LibLIBUSB_1: ${LIBUSB_1_LIBRARIES}")
else (LIBUSB_1_FOUND)
           message(STATUS "don't find LibLIBUSB_1")
endif (LIBUSB_1_FOUND)

MARK_AS_ADVANCED(LIBUSB_1_INCLUDE_DIR LIBUSB_1_LIBRARIES LIBUSB_1_LIBRARY)

FIND_PATH(POINTGREY_INCLUDE_DIR FlyCapture2.h
    /usr/include/flycapture
    /usr/local/include/flycapture
)
message(STATUS "Found POINTGREYCamera: ${POINTGREY_INCLUDE_DIR}")

FIND_LIBRARY(POINTGREY_LIBRARY NAMES libflycapture.so
   /usr/lib
)
message(STATUS "Found POINTGREYCamera library: ${POINTGREY_LIBRARY}")

if (POINTGREY_INCLUDE_DIR AND POINTGREY_LIBRARY)
   set(POINTGREY_FOUND TRUE)
   set(POINTGREY_LIBRARIES ${POINTGREY_LIBRARY})
else (POINTGREY_INCLUDE_DIR AND POINTGREY_LIBRARY)
   set(POINTGREY_FOUND FALSE)
endif (POINTGREY_INCLUDE_DIR AND POINTGREY_LIBRARY)

if (POINTGREY_FOUND)
      message(STATUS "Found LibPOINTGREY: ${POINTGREY_LIBRARIES}")
else (POINTGREY_FOUND)
           message(STATUS "don't find LibPOINTGREY")
endif (POINTGREY_FOUND)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES pointgrey
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(single_reader    src/single_reader.cc    src/ptgrey_lib/singleCamera.cpp    src/ptgrey_lib/singleCameraReader.cpp  src/preprocess/process.cc)
target_link_libraries( single_reader    ${catkin_LIBRARIES}    ${OpenCV_LIBS}    ${POINTGREY_LIBRARIES} ${LIBUSB_1_LIBRARIES}   dw  )

add_library(ptgrey_reader  src/milti_reader.cc   src/ptgrey_lib/singleCameraReader.cpp  src/ptgrey_lib/multiCamera.cpp    src/ptgrey_lib/multiCameraReader.cpp  src/preprocess/process.cc src/ptgrey_lib/singleCamera.cpp)
target_link_libraries(ptgrey_reader    ${catkin_LIBRARIES}    ${OpenCV_LIBS}    ${POINTGREY_LIBRARIES}    ${LIBUSB_1_LIBRARIES}  )

add_executable(multi_reader    src/milti_reader.cc    src/ptgrey_lib/multiCamera.cpp    src/ptgrey_lib/multiCameraReader.cpp  src/ptgrey_lib/singleCamera.cpp src/preprocess/process.cc  )
target_link_libraries( multi_reader   dw ${catkin_LIBRARIES}    ${OpenCV_LIBS}    ${POINTGREY_LIBRARIES}    ${LIBUSB_1_LIBRARIES}   )

add_executable( camera_list    src/camera_list.cc)
target_link_libraries( camera_list    ${catkin_LIBRARIES}    ${OpenCV_LIBS}    ${POINTGREY_LIBRARIES}    ${LIBUSB_1_LIBRARIES}   )
