cmake_minimum_required(VERSION 3.26)
project(im_package_consumer LANGUAGES CXX)

find_package(im_sample_algorithm CONFIG REQUIRED)
get_target_property(im_features mitre::oss::fim_sample_algorithm INTERFACE_COMPILE_FEATURES)
if(NOT im_features STREQUAL "cxx_std_17")
    message(FATAL_ERROR "The installed IM target must export cxx_std_17: ${im_features}")
endif()

add_executable(package_consumer main.cpp)
set_target_properties(package_consumer PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)
target_link_libraries(package_consumer PRIVATE mitre::oss::fim_sample_algorithm)

enable_testing()
add_test(NAME installed_package_consumer COMMAND package_consumer)
