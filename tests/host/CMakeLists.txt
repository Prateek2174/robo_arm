# Host unit tests for HAL-free firmware code.
#
#   cmake -S tests/host -B build/host
#   cmake --build build/host
#   ctest --test-dir build/host --output-on-failure
#
# Every test_<name>.c here that calls UNITY_BEGIN becomes a test, linked with
# <name>.c if one exists in SOURCE_DIRS. Add a test by writing the file; this
# list doesn't need editing. Stubs without UNITY_BEGIN are skipped.

cmake_minimum_required(VERSION 3.20)
project(robo_arm_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  unity
  GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
  GIT_TAG v2.6.0
)
FetchContent_MakeAvailable(unity)

set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(SOURCE_DIRS
  ${REPO_ROOT}/firmware/joint/app
  ${REPO_ROOT}/firmware/power_node/app
  ${REPO_ROOT}/shared/can_protocol
)

enable_testing()

file(GLOB TEST_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test_*.c)

foreach(test_file ${TEST_FILES})
  # Re-run configure when a stub gains UNITY_BEGIN.
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${test_file})

  file(READ ${test_file} test_text)
  if(NOT test_text MATCHES "UNITY_BEGIN")
    continue()
  endif()

  get_filename_component(test_name ${test_file} NAME_WE)
  string(REGEX REPLACE "^test_" "" module ${test_name})

  set(module_source "")
  foreach(dir ${SOURCE_DIRS})
    if(EXISTS ${dir}/${module}.c)
      set(module_source ${dir}/${module}.c)
      break()
    endif()
  endforeach()

  add_executable(${test_name} ${test_file} ${module_source})
  target_include_directories(${test_name} PRIVATE ${SOURCE_DIRS})
  # -Wdouble-promotion flags float code that silently falls into double,
  # which is software-emulated on the G431's single-precision FPU.
  target_compile_options(${test_name} PRIVATE -Wall -Wextra -Wdouble-promotion)
  target_link_libraries(${test_name} PRIVATE unity m)
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
