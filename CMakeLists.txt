cmake_minimum_required(VERSION 3.22)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

if (NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

project(gps_location_estimator VERSION 0.0.0 LANGUAGES CXX)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(
  -Wall -Wextra -Wpedantic -Wunused -Wuninitialized -Wshadow -Wconversion -Wduplicated-cond -Wduplicated-branches
  -Wlogical-op -Wuseless-cast -Wsign-conversion -Wfloat-equal -Weffc++ -Wold-style-cast -Wvexing-parse -Wformat=2 -Wvla 
  -Wsuggest-override -Wsuggest-final-methods -Wsuggest-final-types -Wnon-virtual-dtor -Wctor-dtor-privacy
)

if(CMAKE_COMPILER_IS_GNUCXX)
  if(CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 14.1.1)
    add_compile_options(-Wnrvo)
  endif()
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wweak-vtables -Wglobal-constructors -Wexit-time-destructors -Wused-but-marked-unused -Wcovered-switch-default
    -Wmissing-prototypes -Woverloaded-virtual -Wdelete-non-virtual-dtor
  )
endif()

find_package(Threads REQUIRED)

add_subdirectory(src)

enable_testing()
add_subdirectory(test/cpp)
