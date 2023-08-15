# This file causes creation of the IM algorithms library (libimalgs.a).
cmake_minimum_required(VERSION 3.14)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

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
)

set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_library(imalgs STATIC ${IMALGS_SOURCE_FILES} ${OBSERVER_SOURCE_FILES})
target_compile_options(imalgs PUBLIC "-Wno-inconsistent-missing-destructor-override" "-Wno-inconsistent-missing-override" "-Wno-overloaded-virtual")
target_include_directories(imalgs PUBLIC
        ${aaesim_INCLUDE_DIRS})
target_link_libraries(imalgs
        pub)


