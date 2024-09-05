add_library(xbus STATIC data_packet_parser.cpp xbus_packet.cpp)
target_link_libraries(xbus time_and_location)
target_include_directories(xbus PRIVATE ${CMAKE_SOURCE_DIR}/includes)

if(Boost_FOUND)
  add_library(mti_630 STATIC mti_630.cpp)
  target_link_libraries(mti_630 xbus serial_port)
  target_include_directories(mti_630 PRIVATE ${CMAKE_SOURCE_DIR}/includes)
endif()
