cmake_minimum_required(VERSION 3.26...4.1.2)
project(im_sample_algorithm VERSION 5.2.0.0) # the nano value is a boolean. 1 == SNAPSHOT, 0 == release

set (CMAKE_CXX_STANDARD 20)

if (NOT DEFINED CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set (CMAKE_BUILD_TYPE "Release")
endif()
message (STATUS "${PROJECT_NAME}: Build type is ${CMAKE_BUILD_TYPE}")

option(BUILD_TESTING "Enable building ${PROJECT_NAME} tests" ON)
if(NOT ${BUILD_TESTING})
    message(STATUS "${PROJECT_NAME}: skipping test targets")
endif()


include(${PROJECT_SOURCE_DIR}/.cmake/get_cpm.cmake)

if (${PROJECT_VERSION_TWEAK})
    # Append -SNAPSHOT to the version name if this is a pre-release version
    set(msg_VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}-SNAPSHOT)
else()
    set(msg_VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH})
endif()
message(STATUS "${PROJECT_NAME}: Generating build version ${msg_VERSION}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-strict-aliasing")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wno-unused-function -Wno-sign-compare -O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -Wno-unused-function -Wno-sign-compare -O3 -g3")

# Begin project dependencies ------------------------------------------------------
CPMAddPackage(
    NAME fmacm
    GITHUB_REPOSITORY mitre/FMACM
    GIT_TAG v5.2.0
    OPTIONS
     "BUILD_SHARED_LIBS FALSE"
     "BUILD_TESTING ${BUILD_TESTING}"
)

# CPMAddPackage(
#     NAME unitslib
#     GIT_REPOSITORY https://mustache.mitre.org/scm/aaes/cpp-uom.git # FIXME use external repo when available
#     GIT_TAG 07445f7ec0f98b55cdd6e6188083145a293415a7
#     DOWNLOAD_ONLY TRUE
# )

CPMAddPackage(
    NAME log4cplus
    GITHUB_REPOSITORY log4cplus/log4cplus
    GIT_TAG REL_2_1_2
    OPTIONS
     "BUILD_SHARED_LIBS FALSE"
     "LOG4CPLUS_BUILD_TESTING FALSE"
     "LOG4CPLUS_BUILD_LOGGINGSERVER FALSE"
)

# Create some common paths
if (log4cplus_ADDED)
    set (LOG4CPLUS_DIRS 
        ${log4cplus_BINARY_DIR}/include
        ${log4cplus_SOURCE_DIR}/include)
endif ()

set (unitslib_INCLUDE_DIRS ${fmacm_SOURCE_DIR}/units-2.1/src/)
set (LOADER_DIR            ${fmacm_SOURCE_DIR}/Loader)
set (PUBLIC_DIR            ${fmacm_SOURCE_DIR}/Public)
set (IM_DIR                ${CMAKE_CURRENT_SOURCE_DIR}/IntervalManagement)
set (fmacm_INCLUDE_DIRS    ${CMAKE_CURRENT_SOURCE_DIR}/include 
                           ${fmacm_SOURCE_DIR}/include)
set (UNITTEST_DIR          ${CMAKE_CURRENT_SOURCE_DIR}/unittest)

add_subdirectory(${IM_DIR})
add_subdirectory(${UNITTEST_DIR})
include(${UNITTEST_DIR}/unittest.cmake)
