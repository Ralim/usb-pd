cmake_minimum_required(VERSION 3.9)

project(mylib VERSION 1.0.1 DESCRIPTION "mylib description")

add_library(mylib SHARED
Logic/int_n.cpp
Logic/policy_engine.cpp
Logic/policy_engine_states.cpp
FUSB302/fusb302b.cpp
)

target_include_directories(mylib PRIVATE Inc FUSB302)