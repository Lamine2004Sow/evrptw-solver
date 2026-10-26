cmake_minimum_required(VERSION 3.20)
project(evrptw_solver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(WITH_GUROBI "Build the MILP model (requires a Gurobi installation and license)" ON)
option(BUILD_TESTS "Build unit tests (fetches GoogleTest)" ON)

add_compile_options(-Wall -Wextra)

# ---------------------------------------------------------------- core library
file(GLOB_RECURSE EVRPTW_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_SOURCE_DIR}/src/core/*.cpp
  ${CMAKE_SOURCE_DIR}/src/meta/*.cpp)
add_library(evrptw ${EVRPTW_SOURCES})
target_include_directories(evrptw PUBLIC ${CMAKE_SOURCE_DIR}/include)

# ---------------------------------------------------------------- Gurobi (optional)
if(WITH_GUROBI)
  find_path(GUROBI_INCLUDE_DIR gurobi_c++.h HINTS $ENV{GUROBI_HOME}/include)
  file(GLOB GUROBI_C_LIBS $ENV{GUROBI_HOME}/lib/libgurobi[0-9][0-9][0-9].so)
  find_library(GUROBI_CXX_LIBRARY NAMES gurobi_c++ HINTS $ENV{GUROBI_HOME}/lib)
  if(GUROBI_INCLUDE_DIR AND GUROBI_C_LIBS AND GUROBI_CXX_LIBRARY)
    list(GET GUROBI_C_LIBS 0 GUROBI_C_LIBRARY)
    message(STATUS "Gurobi found: ${GUROBI_C_LIBRARY}")
    file(GLOB MILP_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/src/milp/*.cpp)
    target_sources(evrptw PRIVATE ${MILP_SOURCES})
    target_include_directories(evrptw PRIVATE ${GUROBI_INCLUDE_DIR})
    target_link_libraries(evrptw PUBLIC ${GUROBI_CXX_LIBRARY} ${GUROBI_C_LIBRARY})
    target_compile_definitions(evrptw PUBLIC EVRPTW_WITH_GUROBI)
  else()
    message(WARNING "Gurobi not found (set GUROBI_HOME); building without the MILP model")
  endif()
endif()

# ---------------------------------------------------------------- executables
add_executable(evrptw_cli apps/evrptw_cli.cpp)
target_link_libraries(evrptw_cli PRIVATE evrptw)
set_target_properties(evrptw_cli PROPERTIES OUTPUT_NAME evrptw)

# ---------------------------------------------------------------- tests
if(BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
  enable_testing()
  file(GLOB TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/tests/*.cpp)
  add_executable(evrptw_tests ${TEST_SOURCES})
  target_link_libraries(evrptw_tests PRIVATE evrptw GTest::gtest_main)
  target_compile_definitions(evrptw_tests PRIVATE EVRPTW_DATA_DIR="${CMAKE_SOURCE_DIR}/data/instances")
  include(GoogleTest)
  gtest_discover_tests(evrptw_tests)
endif()
