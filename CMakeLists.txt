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

include(${PROJECT_SOURCE_DIR}/.cmake/get_cpm.cmake)

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
set(IM_SAMPLE_ALGORITHM_FSLOADER_TAG "1.1.0" CACHE STRING "fsloader tag required by FMACM ${IM_SAMPLE_ALGORITHM_FMACM_TAG}")

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

    CPMAddPackage(
        NAME fsloader
        GIT_REPOSITORY https://github.com/mitre/fsloader.git
        GIT_TAG ${IM_SAMPLE_ALGORITHM_FSLOADER_TAG}
        OPTIONS
            "FSLOADER_BUILD_EXAMPLE OFF"
            "BUILD_SHARED_LIBS FALSE"
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
