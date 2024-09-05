# The serial port based parts need Boost.Asio, everything else builds without
# it so the tests can run on any machine. Prefer the BoostConfig.cmake shipped
# by Boost itself, fall back to CMake's FindBoost module for header only
# installs.
find_package(Boost 1.70 CONFIG QUIET)
if(NOT Boost_FOUND)
  find_package(Boost 1.70 MODULE)
endif()

add_subdirectory(time_and_location)
add_subdirectory(rmc)
add_subdirectory(utm)
add_subdirectory(kalman)

if(Boost_FOUND)
  add_subdirectory(utility)
  add_subdirectory(gps)
endif()

add_subdirectory(mti_630)

if(Boost_FOUND)
  add_executable(main main.cpp)
  target_link_libraries(main gps mti_630 kalman Threads::Threads)
  target_include_directories(main PRIVATE ${CMAKE_SOURCE_DIR}/includes/)
else()
  message(WARNING "Boost not found, only building the libraries and tests")
endif()
