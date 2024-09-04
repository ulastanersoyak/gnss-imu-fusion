add_library(kalman STATIC position_estimator.cpp)
target_link_libraries(kalman utm xbus)
target_include_directories(kalman PRIVATE ${CMAKE_SOURCE_DIR}/includes)
