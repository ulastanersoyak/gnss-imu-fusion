add_library(rmc STATIC rmc.cpp)
target_include_directories(rmc PRIVATE ${CMAKE_SOURCE_DIR}/includes)
target_link_libraries(rmc time_and_location)
