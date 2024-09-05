add_library(serial_port STATIC serial_port.cpp)
target_link_libraries(serial_port Boost::headers Threads::Threads)
target_include_directories(serial_port PRIVATE ${CMAKE_SOURCE_DIR}/includes)
