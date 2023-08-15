// ****************************************************************************
// NOTICE
//
// This work was produced for the U.S. Government under Contract 693KA8-22-C-00001
// and is subject to Federal Aviation Administration Acquisition Management System
// Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV (Oct. 1996).
//
// The contents of this document reflect the views of the author and The MITRE
// Corporation and do not necessarily reflect the views of the Federal Aviation
// Administration (FAA) or the Department of Transportation (DOT). Neither the FAA
// nor the DOT makes any warranty or guarantee, expressed or implied, concerning
// the content or accuracy of these views.
//
// For further information, please contact The MITRE Corporation, Contracts Management
// Office, 7515 Colshire Drive, McLean, VA 22102-7539, (703) 983-6000.
//
// 2023 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <map>
#include <vector>
#include <scalar/Time.h>
#include <scalar/Length.h>
#include "public/Logging.h"
#include "public/AircraftState.h"
#include "imalgs/AircraftState.h"
#include "public/HorizontalPath.h"
#include "public/AircraftIntent.h"
#include "public/AlongPathDistanceCalculator.h"
#include "utility/BoundedValue.h"

class IMUtils {
  public:
   static const int UNINITIALIZED_AIRCRAFT_ID;
   static const Units::Length BEYOND_END_OF_ROUTE_TOL;
   static const bool QUANTIZE_FLAG_DEFAULT;
   static const bool LIMIT_FLAG_DEFAULT;
   static const Units::NauticalMilesLength DIST_QUANTIZE_1_DEFAULT;
   static const Units::NauticalMilesLength DIST_QUANTIZE_2_DEFAULT;
   static const Units::KnotsSpeed SPEED_QUANTIZE_1_DEFAULT_1_KNOT;
   static const Units::KnotsSpeed SPEED_QUANTIZE_1_DEFAULT_5_KNOTS;
   static const Units::KnotsSpeed SPEED_QUANTIZE_2_DEFAULT;
   static const Units::KnotsSpeed SPEED_QUANTIZE_3_DEFAULT;

   enum IMAlgorithmTypes {
      TIMEBASEDACHIEVE,
      DISTANCEBASEDACHIEVE,
      KINETICACHIEVE,
      KINETICTARGETACHIEVE,
      TIMEBASEDACHIEVEMUTABLEASG,
      RTA,
      RTA_TOAC_NOT_IMALGORITHM,
      TESTSPEEDCONTROL,
      NONE,
   };

   /**
    * calculate the crossing time for the given position
    * @deprecated
    */
   static bool GetCrossingTime(
         const Units::Length current_dtg,
         const std::vector<interval_management::open_source::AircraftState> &aircraft_state_history,
         const std::vector<HorizontalPath> &horizontal_path, Units::Time &crossing_time);

   static bool GetCrossingTime(
         const Units::Length current_dtg,
         const std::vector<interval_management::open_source::AircraftState> &aircraft_state_history,
         const std::vector<HorizontalPath> &horizontal_path, Units::Time &crossing_time, Units::Length &projected_x,
         Units::Length &projected_y);

   static bool GetCrossingTime(
         const Units::Length current_dtg,
         const std::vector<interval_management::open_source::AircraftState> &aircraft_state_history,
         AlongPathDistanceCalculator &distance_calculator, Units::Time &crossing_time, Units::Length &projected_x,
         Units::Length &projected_y);

   static void CalculateMergePoint(const Units::Length x1, const Units::Length y1, const Units::Length x2,
                                   const Units::Length y2, const Units::Length x3, const Units::Length y3,
                                   Units::Length &x_merge, Units::Length &y_merge, const Units::Angle theta_merge);

   static void CalculateTimeBasedExtrapolate(const Units::Length &ownship_dtg,
                                             const interval_management::open_source::AircraftState &oldest_target_state,
                                             const std::vector<HorizontalPath> &ownship_horizontal_traj,
                                             Units::Time &extrapolated_target_time, Units::Length &projected_x,
                                             Units::Length &projected_y, Units::Length &projected_distance_to_go);

   static void CalculateTimeBasedExtrapolate(const Units::Length &ownship_dtg,
                                             const interval_management::open_source::AircraftState &oldest_target_state,
                                             AlongPathDistanceCalculator &distance_calculator,
                                             Units::Time &extrapolated_target_time, Units::Length &projected_x,
                                             Units::Length &projected_y, Units::Length &projected_distance_to_go);

   static bool CalculateTargetStateAtTime(const interval_management::open_source::AircraftState &target_state,
                                          const std::vector<HorizontalPath> &ownship_horizontal_traj,
                                          const Units::Time extrapolation_time, const Units::Angle ownship_true_heading,
                                          interval_management::open_source::AircraftState &extrapolation_state);

   static void GetTimeBasedExtrapolateState(
         const interval_management::open_source::AircraftState &current_ownship_state,
         const interval_management::open_source::AircraftState &oldest_target_state,
         const std::vector<HorizontalPath> &ownship_horizontal_traj,
         const std::vector<HorizontalPath> &target_horizontal_traj, Units::Time &measured_spacing_interval,
         interval_management::open_source::AircraftState &aircraft_state_to_return);

   static void GetPositionFromPathLength(const Units::Length distance_to_go_in,
                                         const std::vector<HorizontalPath> &horizontal_path_in,
                                         const Units::Angle ownship_true_heading, Units::Length &x_out,
                                         Units::Length &y_out, Units::UnsignedAngle &course_out,
                                         int &horizontal_path_index);

   static bool GetPathLengthFromPosition(const Units::Length x, const Units::Length y,
                                         const std::vector<HorizontalPath> &horizontal_path,
                                         Units::Length &distance_to_go, Units::Angle &track);

   // method to project target aircraft position onto ownship horizontal trajectory
   static bool ProjectTargetPosition(const Units::Length x_target, const Units::Length y_target,
                                     const std::vector<HorizontalPath> &ownship_horizontal_path,
                                     Units::Length &x_projected, Units::Length &y_projected, Units::Length &dtg);

   // method to get target aircraft position on ownship horizontal trajectory based upon distance to go
   static bool ProjectTargetPositionFromDistance(const Units::Length dtg,
                                                 const std::vector<HorizontalPath> &ownship_horizontal_path,
                                                 Units::Length &x_projected, Units::Length &y_projected);

   /**
    * @deprecated AAES-994 make this method unused. It will be removed.
    * @param target_state
    * @param ownship_horizontal_traj
    * @param extrapolation_distance
    * @param ownship_true_heading
    * @param extrapstate
    */
   static void CalculateTargetStateAtDistance(const interval_management::open_source::AircraftState &target_state,
                                              const std::vector<HorizontalPath> &ownship_horizontal_traj,
                                              const Units::Length extrapolation_distance,
                                              const Units::Angle ownship_true_heading,
                                              interval_management::open_source::AircraftState &extrapstate);

   /**
    * @deprecated AAES-994 has made this method unnecessary. It will be removed.
    * @param target_adsb_history
    * @param ownship_horizontal_path
    * @param target_distance
    * @param ownship_true_heading
    * @return
    */
   static interval_management::open_source::AircraftState GetTargetStateOnOwnshipPathForDtg(
         const std::vector<interval_management::open_source::AircraftState> &target_adsb_history,
         const std::vector<HorizontalPath> &ownship_horizontal_path, const Units::Length target_distance,
         const Units::Angle ownship_true_heading);

   static Units::Time InterpolateTimeAtDtg(const Units::Length target_dtg, const Units::Time time1,
                                           const Units::Time time2, const Units::Length dtg1, const Units::Length dtg2);

   static Units::Speed FindLargestSpeedConstraint(const AircraftIntent &aircraft_intent);

   /**
    * Will return a state that has been projected onto the ownship_horizontal_traj. If target_time is
    * between two states, linear interpolation will be used. If extrapolation is needed, constant
    * ground speed is assumed for linear extrapolation.
    *
    * @param target_state_history
    * @param ownship_horizontal_traj
    * @param target_time
    * @param target_state_is_valid
    * @return
    */
   static interval_management::open_source::AircraftState GetProjectedTargetState(
         const std::vector<interval_management::open_source::AircraftState> &target_state_history,
         const std::vector<HorizontalPath> &ownship_horizontal_traj, const Units::Time target_time,
         const Units::Angle ownship_true_heading, bool &target_state_is_valid);

   static interval_management::open_source::AircraftState GetProjectedTargetState(
         AlongPathDistanceCalculator &distance_calculator,
         const std::vector<interval_management::open_source::AircraftState> &target_state_history,
         const std::vector<HorizontalPath> &ownship_horizontal_traj, const Units::Time target_time,
         const Units::Angle ownship_true_heading, bool &target_state_is_valid);

   static Units::SignedAngle CalculateTrackAngle(const std::list<Units::Angle> angle_history);

   /**
    * This map is a simple container that provides retrieval of a string that represents an enum values.
    * e.g. algorithmTypeDictionary.at(IMUtils::IMAlgorithmTypes::NONE)
    */
   static std::map<IMUtils::IMAlgorithmTypes, std::string> algorithm_type_dictionary;

   static interval_management::open_source::AircraftState ConvertToIntervalManagementAircraftState(
         const aaesim::open_source::AircraftState &aircraft_state);

   static aaesim::open_source::AircraftState ConvertToAaesimAircraftState(
         const interval_management::open_source::AircraftState &aircraft_state);

  private:
   static log4cplus::Logger m_logger;
};
