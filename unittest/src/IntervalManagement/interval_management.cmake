cmake_minimum_required(VERSION 3.14)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

set(IMALGORITHM_TEST_SOURCE
        ${UNITTEST_DIR}/src/IntervalManagement/imalgo_tests.cpp
        ${UNITTEST_DIR}/src/IntervalManagement/predicted_wind_evaluator_tests.cpp
        ${UNITTEST_DIR}/src/IntervalManagement/clearance_tests.cpp
        )

add_executable(imalgs_test
        ${IMALGORITHM_TEST_SOURCE}
        ${PUBLIC_TEST_SUPPORT_SOURCE}
        ${UNITTEST_DIR}/src/main.cpp
)
target_link_libraries(imalgs_test
        gtest
        imalgs
)
target_include_directories(imalgs_test PUBLIC
        ${aaesim_INCLUDE_DIRS}
        ${UNITTEST_DIR}/src
        ${geolib_idealab_INCLUDE_DIRS})
set_target_properties(imalgs_test PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/unittest/bin
        EXCLUDE_FROM_ALL TRUE)
add_custom_target(run_imalgs_test
        ${CMAKE_SOURCE_DIR}/unittest/bin/imalgs_test --gtest_output=xml:interval_management_unit_test_results.xml
        DEPENDS ${CMAKE_SOURCE_DIR}/unittest/bin/imalgs_test
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/unittest/
)
