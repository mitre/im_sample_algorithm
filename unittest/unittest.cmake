if(BUILD_TESTING)
   # *************************** UNIT TESTS ******************************** #
   # Link all the actual test code along with main.cpp to the executable, 
   # so as much of test infrastructure is built into
   # the /unittest library as possible.
   include(${CMAKE_CURRENT_LIST_DIR}/src/IntervalManagement/interval_management.cmake)
endif()
