cmake_minimum_required(VERSION 3.26...4.1.2)

include(${CMAKE_CURRENT_LIST_DIR}/../.cmake/get_cpm.cmake)

CPMAddPackage(
      NAME googletest
      GITHUB_REPOSITORY google/googletest
      VERSION 1.15.2
      OPTIONS
      "INSTALL_GTEST OFF"
      "GTEST_FORCE_SHARED_CRT ON"
      "BUILD_GMOCK ON"
)

include(${CMAKE_CURRENT_LIST_DIR}/src/IntervalManagement/interval_management.cmake)
