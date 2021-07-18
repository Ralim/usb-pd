cmake_minimum_required(VERSION 3.7)
project(cmakeCppUTestDemo)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-std=c++11 -Wall -Werror)

set(APP_NAME USBPD)
set(APP_LIB_NAME USBPDLib)

# (2) Include application build instructions

add_library(${APP_LIB_NAME} SHARED
    src/policy_engine.cpp
    src/policy_engine_states.cpp
    src/fusb302b.cpp
)

target_include_directories(${APP_LIB_NAME} PRIVATE include src)

# (3) include tests build instructions   
option(COMPILE_TESTS "Compile the tests" OFF)
if(COMPILE_TESTS)
  add_subdirectory(tests)
endif(COMPILE_TESTS)
