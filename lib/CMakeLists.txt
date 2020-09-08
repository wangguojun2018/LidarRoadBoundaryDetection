file(GLOB SRC . *.cpp)
file(GLOB HEAD . *.h)
set(PROTO_SRC ../proto/road_boundary.pb.cc ../proto/road_boundary.pb.h)
add_library(extract ${SRC} ${HEAD} ${PROTO_SRC})
target_link_libraries(extract ${PCL_LIBRARIES} ${OpenCV_LIBS} 
                        OpenMP::OpenMP_CXX 
                        ${Protobuf_LIBRARIES}
                        ${GLOG_LIBRARIES})
