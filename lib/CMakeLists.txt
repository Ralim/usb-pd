cmake_minimum_required(VERSION 3.9)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-std=c++11 -Wall -Werror)

add_library(${APP_LIB_NAME} SHARED
    Logic/int_n.cpp
    Logic/policy_engine.cpp
    Logic/policy_engine_states.cpp
    FUSB302/fusb302b.cpp
)

target_include_directories(${APP_LIB_NAME} PRIVATE Inc FUSB302)