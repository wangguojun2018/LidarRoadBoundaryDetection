cmake_minimum_required(VERSION 2.8)

project(boundary_detection)


set(CMAKE_CXX_FLAGS "-std=c++11 -march=native")
set(CMAKE_BUILD_TYPE Release CACHE STRING "set build type to release")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(LIMBO_INCLUDE_DIRS ./limbo/src)


find_package(OpenMP REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Boost)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Glog REQUIRED) 


include_directories(/home/wgj/source_code/cpp/limbo/src
    ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}  
    ${Qt5Charts_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} 
    ${LIMBO_INCLUDE_DIRS} ${Protobuf_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS} .)

link_directories(${PCL_LIBRARY_DIRS} /home/wgj/source_code/cpp/limbo/build/src)

# protobuf_generate_cpp(road_boundary.pb.cpp road_boundary.pb.h road_boundary.proto)

add_definitions(${PCL_DEFINITIONS})

add_subdirectory(./lib)

#add_library(extract ${SRC})
#target_link_libraries(extract ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)

add_executable(demo_frame src/main.cpp)

target_link_libraries(demo_frame ${PCL_LIBRARIES} ${OpenCV_LIBS} extract 
${Protobuf_LIBRARIES} ${GLOG_LIBRARIES})


add_executable(demo_video src/main_thread.cpp)

target_link_libraries(demo_video ${PCL_LIBRARIES} ${OpenCV_LIBS} extract 
${Protobuf_LIBRARIES} ${GLOG_LIBRARIES})
