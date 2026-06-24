cmake_minimum_required(VERSION 3.26...4.2)
project(im_sample_algorithm VERSION 5.3.1.0 LANGUAGES CXX) # the nano value is a boolean. 1 == SNAPSHOT, 0 == release

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

if(NOT CMAKE_CONFIGURATION_TYPES AND (NOT DEFINED CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL ""))
    set(CMAKE_BUILD_TYPE "Release")
endif()
message(STATUS "${PROJECT_NAME}: Build type is ${CMAKE_BUILD_TYPE}")

include(CTest)
option(IM_SAMPLE_ALGORITHM_BUILD_TESTING "Enable building ${PROJECT_NAME} tests" ${PROJECT_IS_TOP_LEVEL})
if(NOT BUILD_TESTING)
    set(IM_SAMPLE_ALGORITHM_BUILD_TESTING OFF)
endif()

if(NOT COMMAND CPMAddPackage)
    include(${CMAKE_CURRENT_LIST_DIR}/.cmake/get_cpm.cmake)
endif()

if(${PROJECT_VERSION_TWEAK})
    # Append -SNAPSHOT to the version name if this is a pre-release version
    set(msg_VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}-SNAPSHOT)
else()
    set(msg_VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH})
endif()
message(STATUS "${PROJECT_NAME}: Generating build version ${msg_VERSION}")

# Begin project dependencies ------------------------------------------------------
option(IM_SAMPLE_ALGORITHM_FETCH_FMACM "Fetch FMACM when the pub target is not already available" ON)
set(IM_SAMPLE_ALGORITHM_FMACM_TAG "v5.3.1" CACHE STRING "FMACM tag used for standalone builds")
set(IM_SAMPLE_ALGORITHM_FSLOADER_TAG "1.1.0" CACHE STRING "fsloader tag used for im_sample_algorithm builds")
set(IM_SAMPLE_ALGORITHM_NLOHMANN_JSON_VERSION "3.11.3" CACHE STRING "nlohmann_json version used for standalone builds")
set(IM_SAMPLE_ALGORITHM_LOG4CPLUS_TAG "REL_2_1_2" CACHE STRING "log4cplus tag used for standalone builds")

if(NOT TARGET nlohmann_json::nlohmann_json)
    CPMAddPackage(
        NAME nlohmann_json
        GITHUB_REPOSITORY nlohmann/json
        VERSION ${IM_SAMPLE_ALGORITHM_NLOHMANN_JSON_VERSION}
    )
endif()

if(NOT TARGET log4cplus::log4cplusS AND NOT TARGET log4cplusS)
    if(TARGET log4cplus::log4cplus)
        message(FATAL_ERROR "${PROJECT_NAME}: static log4cplus target log4cplus::log4cplusS is required, but only log4cplus::log4cplus is available.")
    endif()

    CPMAddPackage(
        NAME log4cplus
        GITHUB_REPOSITORY log4cplus/log4cplus
        GIT_TAG ${IM_SAMPLE_ALGORITHM_LOG4CPLUS_TAG}
        OPTIONS
            "LOG4CPLUS_BUILD_TESTING OFF"
            "LOG4CPLUS_BUILD_LOGGINGSERVER OFF"
            "BUILD_SHARED_LIBS FALSE"
    )
endif()

if(NOT TARGET log4cplus::log4cplusS AND TARGET log4cplusS)
    add_library(log4cplus::log4cplusS ALIAS log4cplusS)
endif()

if(NOT TARGET log4cplus::log4cplusS)
    message(FATAL_ERROR "${PROJECT_NAME}: static log4cplus target log4cplus::log4cplusS is required.")
endif()

if(NOT TARGET log4cplus::log4cplus)
    if(TARGET log4cplusS)
        add_library(log4cplus::log4cplus ALIAS log4cplusS)
    else()
        add_library(im_sample_algorithm_log4cplus_compat INTERFACE)
        target_link_libraries(im_sample_algorithm_log4cplus_compat INTERFACE log4cplus::log4cplusS)
        add_library(log4cplus::log4cplus ALIAS im_sample_algorithm_log4cplus_compat)
    endif()
endif()

if(NOT TARGET mitre::fsloader)
    CPMAddPackage(
        NAME fsloader
        GIT_REPOSITORY https://github.com/mitre/fsloader.git
        GIT_TAG ${IM_SAMPLE_ALGORITHM_FSLOADER_TAG}
        OPTIONS
            "FSLOADER_BUILD_EXAMPLE OFF"
            "BUILD_SHARED_LIBS FALSE"
    )
endif()

if(NOT TARGET pub)
    if(NOT IM_SAMPLE_ALGORITHM_FETCH_FMACM)
        message(FATAL_ERROR "${PROJECT_NAME}: target pub is required. Add FMACM first or enable IM_SAMPLE_ALGORITHM_FETCH_FMACM.")
    endif()

    CPMAddPackage(
        NAME fmacm
        GITHUB_REPOSITORY mitre/FMACM
        GIT_TAG ${IM_SAMPLE_ALGORITHM_FMACM_TAG}
        DOWNLOAD_ONLY TRUE
    )

    # FMACM's Public CMake expects this variable from the FMACM root project.
    set(aaesim_INCLUDE_DIRS ${fmacm_SOURCE_DIR}/include)
    add_subdirectory(${fmacm_SOURCE_DIR}/Public ${CMAKE_BINARY_DIR}/_deps/fmacm-public-build)
endif()

add_subdirectory(IntervalManagement)

if(IM_SAMPLE_ALGORITHM_BUILD_TESTING)
    add_subdirectory(unittest)
else()
    message(STATUS "${PROJECT_NAME}: skipping test targets")
endif()
