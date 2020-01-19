cmake_minimum_required(VERSION 3.10.2)
project(neuro_car)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    std_srvs
    message_generation
)

find_package(CUDA 10 REQUIRED)

#Optimize or add debug symbols
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_definitions(-O3)
else()
  add_definitions(-g)
endif()

catkin_package(
  INCLUDE_DIRS include ${CMAKE_CURRENT_SOURCE_DIR}/../../../
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../thundersvm/build/
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../thundersvm/include/
    ${CUDA_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

# add libsvm
message(WARNING ${CMAKE_CURRENT_SOURCE_DIR})
link_directories()

####################
##      Util      ##
####################

add_library(util
    src/sigint_handler.cpp
)
target_link_libraries(util ${catkin_LIBRARIES})

####################
##   SVM Node     ##
####################

## Create Detector Handler
add_executable(svm src/svm.cpp)
add_dependencies(svm
  util
  std_msgs_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(svm
  util
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../thundersvm/build/lib/libthundersvm.so
  ${catkin_LIBRARIES}
)

####################
##   Car Node     ##
####################

add_executable(car src/car.cpp)
add_dependencies(car
  util
  std_msgs_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(car
  util
  ${catkin_LIBRARIES}
)
