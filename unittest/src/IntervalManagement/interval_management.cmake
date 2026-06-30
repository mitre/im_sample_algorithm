set(IMALGORITHM_TEST_SOURCE
        ${CMAKE_CURRENT_LIST_DIR}/imalgo_tests.cpp
        ${CMAKE_CURRENT_LIST_DIR}/predicted_wind_evaluator_tests.cpp
        # ${CMAKE_CURRENT_LIST_DIR}/clearance_tests.cpp
        )

add_executable(imalgs_test
        ${IMALGORITHM_TEST_SOURCE}
        ${CMAKE_CURRENT_LIST_DIR}/../main.cpp
)
target_link_libraries(imalgs_test
        PRIVATE
        GTest::gtest
        mitre::oss::fim_sample_algorithm
)
target_compile_options(imalgs_test
        PRIVATE
        -fno-strict-aliasing
        -Wunused-result
        $<$<CONFIG:Debug>:-Wall -Wno-sign-compare -O1 -g3>)
target_include_directories(imalgs_test PRIVATE ${CMAKE_CURRENT_LIST_DIR}/..)
set_target_properties(imalgs_test PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_test(
        NAME imalgs_test
        COMMAND imalgs_test --gtest_output=xml:${CMAKE_CURRENT_BINARY_DIR}/interval_management_unit_test_results.xml)
set_tests_properties(imalgs_test PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

add_custom_target(run_imalgs_test
        COMMAND $<TARGET_FILE:imalgs_test> --gtest_output=xml:${CMAKE_CURRENT_BINARY_DIR}/interval_management_unit_test_results.xml
        DEPENDS imalgs_test
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)
