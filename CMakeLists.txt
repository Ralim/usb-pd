cmake_minimum_required(VERSION 3.7)
project(cmakeCppUTestDemo)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-std=c++11 -Wall -Werror)

set(APP_NAME fooApp)
set(APP_LIB_NAME fooAppLib)

# (2) Include application build instructions
add_subdirectory(lib)

# (3) include tests build instructions   
option(COMPILE_TESTS "Compile the tests" OFF)
if(COMPILE_TESTS)
  add_subdirectory(tests)
endif(COMPILE_TESTS)
