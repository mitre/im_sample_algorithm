# Mapping DO-361A Content to Code

[RTCA's DO-361A](https://my.rtca.org/nc__store?search=do-361) contains two appendices which document algorithms and models associated with the research for Interval Management.

* Appendix C: SAMPLE ALGORITHM provides detailed technical documentation of a "sample" algorithm that meets the requirements in DO-361A. The software in this git repo is our own implementation of Appendix C.
* Appendix H: MOPS Aircraft and Control Model is an overview of a 3-degree-of-freedom aircraft dynamics & control model.

These appendices have been written by (MITRE)(https://www.mitre.org) and are copyright of RTCA. In this git repo, and our [associated FIM MOPS Aircraft & Control Model](https://mitre.github.io/FMACM), are provided our software implementations of the algorithms and models. Below is a general mapping between each section of DO-361A Appendix C & H and our code.

(If any mistakes are noticed in this mapping, please [open an issue](https://github.com/mitre/im_sample_algorithm/issues) to notify us of the mistake.)

Appendix Section | Appendix Title | Public Implementation
C | SAMPLE ALGORITHM | https://mitre.github.io/im_sample_algorithm
C.1 | Overview | No Applicable URL
C.2 | Generation of the Reference Trajectories | Not Available
C.2.1 | Calculation of the Horizontal Path | https://github.com/mitre/FMACM/blob/master/Public/HorizontalPath.cpp
C.2.1.1 | Determining the Waypoint Sequence | https://github.com/mitre/FMACM/blob/master/Public/AircraftIntent.cpp
C.2.1.2 | Conversion of Waypoint Latitudes and Longitudes to x;y Coordinates | https://github.com/mitre/FMACM/blob/master/Public/TangentPlaneSequence.cpp
C.2.1.3 | Determine Course Change Between Waypoints | Not Available
C.2.1.4 | Calculate Turn Radius | Not Available
C.2.1.5 | Calculate the Start; End; and Center of the Turn | Not Available
C.2.1.6 | Determine Horizontal Path Transitions and Distance-To-Go (DTG) | Not Available
C.2.1.7 | Data Format for the Horizontal Path | https://github.com/mitre/FMACM/blob/master/Public/HorizontalPath.cpp
C.2.2 | Ground Speed Assembly | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/KinematicTrajectoryPredictor.cpp
C.2.2.1 | Trajectory Segments | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/KinematicDescent4DPredictor.cpp
C.2.2.1.1 | Constant Calibrated Airspeed descent | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/KinematicDescent4DPredictor.cpp
C.2.2.1.2 | Constant Mach Descent | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/KinematicDescent4DPredictor.cpp
C.2.2.1.3 | Deceleration and Descent | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/KinematicDescent4DPredictor.cpp
C.2.2.1.4 | Constant Geometric Flight-Path Angle Descent; Maintain Constant Airspeed | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/KinematicDescent4DPredictor.cpp
C.2.2.1.5 | Constant Geometric Flight-Path Angle Descent; Decelerate Airspeed | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/KinematicDescent4DPredictor.cpp
C.2.2.1.6 | Level Flight; Maintain Constant Airspeed | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/KinematicDescent4DPredictor.cpp
C.2.2.1.7 | Level Flight; Decelerate Airspeed | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/KinematicDescent4DPredictor.cpp
C.2.2.2 | Calculation of the Mach/CAS Crossover Altitude | https://github.com/mitre/FMACM/blob/master/Public/Atmosphere.cpp
C.2.2.3 | Wind Information | No Applicable URL
C.2.2.3.1 | Format for Forecast and Sensed Wind Information | https://github.com/mitre/FMACM/blob/master/Public/Wind.cpp
C.2.2.3.2 | Spline-Fit Algorithms to Calculate Predicted Winds | https://github.com/mitre/FMACM/blob/master/Public/Wind.cpp
C.2.2.4 | Temperature Information | https://github.com/mitre/FMACM/blob/master/Public/StandardAtmosphere.cpp
C.2.2.5 | Updating Wind and Temperature Information and Recalculating Predicted Trajectories | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/MOPSPredictedWindEvaluatorVersion2.cpp; https://github.com/mitre/FMACM/blob/master/Public/Wind.cpp
C.2.2.6 | Use of Procedural and Default Limits | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/KinematicDescent4DPredictor.cpp
C.2.2.7 | Logic for Combining Trajectory Segments | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/KinematicDescent4DPredictor.cpp
C.2.2.8 | Calculating Along-path Positions from Current Position | https://github.com/mitre/FMACM/blob/master/Public/AircraftCalculations.cpp; https://github.com/mitre/FMACM/blob/master/Public/PositionCalculator.cpp
C.2.2.8.1 | Cross-Track Error Calculation | https://github.com/mitre/FMACM/blob/master/Public/AircraftCalculations.cpp
C.2.2.8.2 | Along-path Position Calculation | https://github.com/mitre/FMACM/blob/master/Public/AircraftCalculations.cpp; https://github.com/mitre/FMACM/blob/master/Public/AlongPathDistanceCalculator.cpp
C.2.2.9 | Calculating TTG from Along-path Distance | No Applicable URL
C.3 | Nomenclature | No Applicable URL
C.4 | Achieve Stage | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/IMAchieve.cpp
C.4.1 | Predicted Spacing Interval Calculation | No Applicable URL
C.4.1.1 | Time-based Predicted Spacing Interval | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/IMTimeBasedAchieve.cpp
C.4.1.2 | Distance-based Predicted Spacing Interval | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/IMDistBasedAchieve.cpp
C.4.2 | Default Traffic Reference Point | Not Available
C.4.3 | Time-to-Go Control Law for Time-based Assigned Spacing Goals | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/IMTimeBasedAchieve.cpp
C.4.4 | Time-to-Go Control Law for Distance-based Assigned Spacing Goals | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/IMDistBasedAchieve.cpp
C.4.5 | Error Thresholds | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/IMAchieve.cpp
C.4.6 | Hysteresis on New Speed Implementation | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/IMAlgorithm.cpp
C.4.7 | Discretization of the Control Law Speed | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/IMTimeBasedAchieve.cpp
C.5 | IM Turn Algorithm | No Applicable URL
C.5.1 | Calculation of the IM Turn Point | Not Available
C.5.2 | IM Turn Leg Lengths | Not Available
C.5.3 | Reference TTG for Ownship | Not Available
C.5.4 | Assumed Vertical and Airspeed Profiles | Not Available
C.5.5 | Description of the IM Turn Point | Not Available
C.5.6 | Transition to the Achieve Stage | Not Available
C.6 | Final Approach Spacing Algorithm | No Applicable URL
C.6.1 | Determine Whether an Aircraft is on a Vector to Intercept | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/IMKinematicAchieve.cpp
C.6.2 | Calculate Merge Point | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/IMUtils.cpp
C.7 | Maintain Stage Algorithm | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/IMMaintain.cpp
C.7.1 | Measured Spacing Interval Calculation | No Applicable URL
C.7.1.1 | Traffic Alignment | Not Available
C.7.1.2 | Time-based Measured Spacing Interval | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/IMKinematicTimeBasedMaintain.cpp
C.7.1.3 | Distance-based Measured Spacing Interval | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/IMKinematicDistBasedMaintain.cpp
C.7.2 | Time-History Control Law for Time-based Assigned Spacing Goals | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/IMKinematicTimeBasedMaintain.cpp
C.7.3 | Station-Keeping Algorithm for Distance-based Assigned Spacing Goals | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/IMKinematicDistBasedMaintain.cpp
C.7.4 | Discretization of the Control Law Speed | https://github.com/mitre/im_sample_algorithm/blob/master/IntervalManagement/IMKinematicTimeBasedMaintain.cpp
H | MOPS Aircraft and Control Model | https://mitre.github.io/FMACM
H.1 | Aircraft Equations | No Applicable URL
H.1.1 | Reference Frames | https://github.com/mitre/FMACM/blob/master/Public/ThreeDOFDynamics.cpp
H.1.2 | Equations of Motion | https://github.com/mitre/FMACM/blob/master/Public/ThreeDOFDynamics.cpp
H.2 | Guidance Control Laws | https://github.com/mitre/FMACM/blob/master/Public/AircraftControl.cpp
H.2.1 | Lateral Control | https://github.com/mitre/FMACM/blob/master/Public/AircraftControl.cpp
H.2.2 | Speed Control Using Thrust | https://github.com/mitre/FMACM/blob/master/Public/SpeedOnThrustControl.cpp
H.2.3 | Speed Control Using Pitch | https://github.com/mitre/FMACM/blob/master/Public/SpeedOnPitchControl.cpp
H.2.4 | Control Gains and Parameter Values | https://github.com/mitre/FMACM/blob/master/Public/AircraftControl.cpp
H.3 | Aircraft-Specific Modeling | https://github.com/mitre/FMACM/blob/master/include/aaesim/BadaWithCalc.h
