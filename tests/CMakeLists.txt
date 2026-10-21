if(DEFINED ENV{CPPUTEST_HOME})
    message(STATUS "Using CppUTest home: $ENV{CPPUTEST_HOME}")
    set(CPPUTEST_INCLUDE_DIRS $ENV{CPPUTEST_HOME}/include)
    set(CPPUTEST_LIBRARIES $ENV{CPPUTEST_HOME}/lib)
    set(CPPUTEST_LDFLAGS CppUTest CppUTestExt)
else()
    find_package(PkgConfig REQUIRED)
    pkg_search_module(CPPUTEST REQUIRED cpputest>=3.8)
    message(STATUS "Found CppUTest version ${CPPUTEST_VERSION}")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(TEST_APP_NAME ${APP_NAME}_tests)
set(TEST_SOURCES
    test_fusb302.cpp
    main.cpp
    mock_fusb302.cpp
    test_pd_policy_engine.cpp
    user_functions.cpp
    test_ringbuffer.cpp
)

include_directories(${CPPUTEST_INCLUDE_DIRS} PRIVATE ../src ../include )
include_directories(${CPPUTEST_INCLUDE_DIRS} .)
link_directories(${CPPUTEST_LIBRARIES})

# (4) Build the unit tests objects and link then with the app library
add_executable(${TEST_APP_NAME} ${TEST_SOURCES})
target_link_libraries(${TEST_APP_NAME} ${APP_LIB_NAME} ${CPPUTEST_LDFLAGS})

# (5) Run the test once the build is done
add_custom_command(TARGET ${TEST_APP_NAME} COMMAND ./${TEST_APP_NAME} POST_BUILD)

# Run recovery cases separately to reset the policy engine's static state.
add_executable(USBPD_recovery_tests pd_recovery_regression.cpp)
target_link_libraries(USBPD_recovery_tests ${APP_LIB_NAME})
target_compile_options(USBPD_recovery_tests PRIVATE -UNDEBUG)
foreach(scenario received_reset_timeout outgoing_reset interleaved_requests mismatched_ack
                 pd2_pd3_completion received_reset_ready wrapped_full_queue duplicate_ack pd2_received_reset)
  add_test(NAME USBPD_${scenario} COMMAND USBPD_recovery_tests ${scenario})
endforeach()
add_test(NAME USBPD_existing_tests COMMAND ${TEST_APP_NAME})
