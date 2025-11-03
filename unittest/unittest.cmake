if(${BUILD_TESTING})
   # *************************** UNIT TESTS ******************************** #
   # Link all the actual test code along with main.cpp to the executable, 
   # so as much of test infrastructure is built into
   # the /unittest library as possible.
   include(${UNITTEST_DIR}/src/IntervalManagement/interval_management.cmake)
endif()
