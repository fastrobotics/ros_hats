add_library(gpsHatNodeProcess GPSHatNodeProcess.cpp)
target_link_libraries(gpsHatNodeProcess ${catkin_LIBRARIES} baseNodeProcess gpsHatDriver utmConversion)

add_executable(gps_hat_node GPSHatNode.cpp) 
target_link_libraries(gps_hat_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} gpsHatNodeProcess baseNode   )
add_dependencies(gps_hat_node ${eros_EXPORTED_TARGETS})
add_subdirectory(test)
