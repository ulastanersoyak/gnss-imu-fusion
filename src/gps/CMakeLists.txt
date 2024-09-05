add_library(gps STATIC gps.cpp)
target_link_libraries(gps utm rmc serial_port)
target_include_directories(gps PRIVATE ${CMAKE_SOURCE_DIR}/includes)
