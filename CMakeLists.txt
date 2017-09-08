project(physics-based-simulation CXX)
cmake_minimum_required(VERSION 2.8.6)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
if (COMPILER_SUPPORTS_CXX14)
  add_compile_options(-std=c++14)
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfatal-errors")

if (CMAKE_BUILD_TYPE MATCHES "Deb")
  add_compile_options(-Wall -g -ggdb)
endif()

if (CMAKE_BUILD_TYPE MATCHES "Rel")
  add_compile_options(-O3 -march=native -ffast-math)
endif()

if (CMAKE_BUILD_TYPE MATCHES "Info")
  add_compile_options(-fopt-info-vec-optimized -fopt-info-inline-optimized -ftree-vectorizer-verbose=2)
endif()

add_subdirectory(third_party)
add_subdirectory(src/flint)
add_subdirectory(src/flint_viewport)
add_subdirectory(examples)
