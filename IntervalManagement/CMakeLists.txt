# This file causes creation of the IM algorithms library (libimalgs.a).
cmake_minimum_required(VERSION 3.14)

if(NOT TARGET mitre::oss::simcore)
    message(FATAL_ERROR "imalgs requires the mitre::oss::simcore target. Configure from the repository root or add aircraft_simulation_core before this directory.")
endif()
if(NOT TARGET mitre::fsloader)
    message(FATAL_ERROR "imalgs requires the mitre::fsloader target. Configure from the repository root or add fsloader before this directory.")
endif()
if(NOT TARGET nlohmann_json::nlohmann_json)
    message(FATAL_ERROR "imalgs requires the nlohmann_json::nlohmann_json target. Configure from the repository root or add nlohmann_json before this directory.")
endif()
if(NOT TARGET log4cplus::log4cplusS)
    message(FATAL_ERROR "imalgs requires the static log4cplus::log4cplusS target. Configure from the repository root or add static log4cplus before this directory.")
endif()

get_filename_component(IM_SAMPLE_ALGORITHM_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

set(OBSERVER_SOURCE_FILES
        AchieveObserver.cpp
        InternalObserver.cpp
        CrossTrackObserver.cpp
        MergePointMetric.cpp
        NMObserver.cpp
        NMObserverEntry.cpp
        MaintainMetric.cpp
        ClosestPointMetric.cpp
)
set(IMALGS_SOURCE_FILES
        AircraftState.cpp
        AircraftIntentLoader.cpp
        FIMAlgorithmDataWriter.cpp
        FIMAlgorithmAdapter.cpp
        FIMAlgorithmInitializer.cpp
        FIMConfiguration.cpp
        FIMSpeedQuantizer.cpp
        FIMSpeedLimiter.cpp
        IMAlgorithm.cpp
        IMClearance.cpp
        IMClearanceLoader.cpp
        IMMaintain.cpp
        IMKinematicTimeBasedMaintain.cpp
        IMKinematicDistBasedMaintain.cpp
        IMAchieve.cpp
        IMKinematicAchieve.cpp
        IMDistBasedAchieve.cpp
        IMTimeBasedAchieve.cpp
        IMTimeBasedAchieveMutableASG.cpp
        IMUtils.cpp
        AchievePointCalcs.cpp
        MOPSPredictedWindEvaluatorVersion1.cpp
        MOPSPredictedWindEvaluatorVersion2.cpp
        Statistics.cpp
        TrueDistances.cpp
        PredictionFileKinematic.cpp
        InternalObserverScenarioWriter.cpp
        WaypointLoader.cpp
)

add_library(imalgs STATIC ${IMALGS_SOURCE_FILES} ${OBSERVER_SOURCE_FILES})
add_library(mitre::oss::fim_sample_algorithm ALIAS imalgs)

set_target_properties(imalgs PROPERTIES
        CXX_EXTENSIONS ON
        EXPORT_NAME fim_sample_algorithm
        POSITION_INDEPENDENT_CODE ON)
target_compile_features(imalgs PUBLIC cxx_std_20)
target_compile_options(imalgs
        PRIVATE
        -fno-strict-aliasing
        $<$<CONFIG:Debug>:-Wall -Wno-unused-function -Wno-sign-compare -O0 -g>
        $<$<CONFIG:Release>:-Wall -Wno-unused-function -Wno-sign-compare -O3 -g3>
        $<$<CXX_COMPILER_ID:Clang,AppleClang>:-Wno-inconsistent-missing-destructor-override -Wno-inconsistent-missing-override -Wno-overloaded-virtual>)
target_include_directories(imalgs
        PUBLIC
        $<BUILD_INTERFACE:${IM_SAMPLE_ALGORITHM_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(imalgs
        PUBLIC
        mitre::oss::simcore
        mitre::fsloader
        nlohmann_json::nlohmann_json
        log4cplus::log4cplusS)
