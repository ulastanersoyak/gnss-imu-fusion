add_executable(tests test.cpp)
target_include_directories(tests PRIVATE ${CMAKE_SOURCE_DIR}/includes ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(tests rmc time_and_location)
add_test(NAME tests COMMAND tests)
