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

#include <stdexcept>
#include <cfloat>
#include "imalgs/IMUtils.h"
#include "public/ScenarioUtils.h"
#include "public/AircraftCalculations.h"
#include "public/SimulationTime.h"

using namespace std;
using namespace aaesim::open_source;

log4cplus::Logger IMUtils::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("IMUtils"));
const int IMUtils::UNINITIALIZED_AIRCRAFT_ID = ScenarioUtils::AIRCRAFT_ID_NOT_IN_MAP;
const Units::Length IMUtils::BEYOND_END_OF_ROUTE_TOL = Units::MetersLength(-300);  // don't reduce. See AAES-745
const Units::NauticalMilesLength IMUtils::DIST_QUANTIZE_1_DEFAULT(0);
const Units::NauticalMilesLength IMUtils::DIST_QUANTIZE_2_DEFAULT(10);
const Units::KnotsSpeed IMUtils::SPEED_QUANTIZE_1_DEFAULT_1_KNOT(1);
const Units::KnotsSpeed IMUtils::SPEED_QUANTIZE_1_DEFAULT_5_KNOTS(5);
const Units::KnotsSpeed IMUtils::SPEED_QUANTIZE_2_DEFAULT(5);
const Units::KnotsSpeed IMUtils::SPEED_QUANTIZE_3_DEFAULT(10);
const bool IMUtils::LIMIT_FLAG_DEFAULT(true);
const bool IMUtils::QUANTIZE_FLAG_DEFAULT(true);

std::map<IMUtils::IMAlgorithmTypes, std::string> IMUtils::algorithm_type_dictionary = {
      {IMAlgorithmTypes::TIMEBASEDACHIEVEMUTABLEASG, "TIMEBASEDACHIEVEMUTABLEASG"},
      {IMAlgorithmTypes::TIMEBASEDACHIEVE, "TIMEBASEDACHIEVE"},
      {IMAlgorithmTypes::DISTANCEBASEDACHIEVE, "DISTANCEBASEDACHIEVE"},
      {IMAlgorithmTypes::KINETICACHIEVE, "KINETICACHIEVE"},
      {IMAlgorithmTypes::KINETICTARGETACHIEVE, "KINETICTARGETACHIEVE"},
      {IMAlgorithmTypes::NONE, "NONE"}};

/**
 * This signature is old. It intentionally shadows a method with more extensive returns.
 * @deprecated Use new signature
 * @param current_dtg
 * @param aircraft_state_history
 * @param horizontal_path
 * @param crossing_time
 * @return
 */
bool IMUtils::GetCrossingTime(
      const Units::Length current_dtg,
      const std::vector<interval_management::open_source::AircraftState> &aircraft_state_history,
      const std::vector<HorizontalPath> &horizontal_path, Units::Time &crossing_time) {
   Units::Length tmpx, tmpy;
   bool ret_val = GetCrossingTime(current_dtg, aircraft_state_history, horizontal_path, crossing_time, tmpx, tmpy);

   LOG4CPLUS_TRACE(m_logger, Units::MetersLength(tmpx).value()
                                   << "," << Units::MetersLength(tmpy).value() << std::endl);
   return ret_val;
}

/*
 * Calculates the merge point for an aircraft on an intercept vector to final, where
 *   x1, y1 is the current position of the aircraft on final,
 *   x2, y2 is the Achieve By point
 *   x3, y3 is the position of the intercepting aircraft
 *   thetaMerge is the course of the intercepting aircraft,
 *
 *   xMerge, yMerge is the calculated merge point.
 *
 * It is assumed that the intercept course is 30 degrees or less from the final course.  This is not checked.
 * Indeed, the final approach course is not known or calculated.
 *
 * see DO-361 section C.5.2 for details
 */
void IMUtils::CalculateMergePoint(const Units::Length x1, const Units::Length y1, const Units::Length x2,
                                  const Units::Length y2, const Units::Length x3, const Units::Length y3,
                                  Units::Length &x_merge, Units::Length &y_merge, const Units::Angle theta_merge) {

   LOG4CPLUS_TRACE(m_logger,
                   "Pos of AC on final:  (" << Units::MetersLength(x1) << "," << Units::MetersLength(y1) << ")");
   LOG4CPLUS_TRACE(m_logger,
                   "Pos of intercepting AC:  (" << Units::MetersLength(x3) << "," << Units::MetersLength(y3) << ")");
   LOG4CPLUS_TRACE(m_logger,
                   "Achieve-by point:  (" << Units::MetersLength(x2) << "," << Units::MetersLength(y2) << ")");

   Units::RadiansAngle theta_final(atan2(Units::MetersLength(y2 - y1).value(), Units::MetersLength(x2 - x1).value()));
   LOG4CPLUS_TRACE(m_logger, "thetaMerge = " << Units::DegreesAngle(theta_merge)
                                             << ", thetaFinal = " << Units::DegreesAngle(theta_final)
                                             << ", turn = " << Units::DegreesAngle(theta_final - theta_merge));

   Units::Length x4(x3 + Units::NauticalMilesLength(50) * cos(theta_merge)),
         y4(y3 + Units::NauticalMilesLength(50) * sin(theta_merge));
   Units::Area denominator(((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4))), factor1((x1 * y2) - (y1 * x2)),
         factor2((x3 * y4) - (y3 * x4));

   LOG4CPLUS_TRACE(m_logger, "denominator = " << Units::MetersArea(denominator)
                                              << ", factor1 = " << Units::MetersArea(factor1)
                                              << ", factor2 = " << Units::MetersArea(factor2));

   x_merge = (((x3 - x4) * factor1) - ((x1 - x2) * factor2)) / denominator;
   y_merge = (((y3 - y4) * factor1) - ((y1 - y2) * factor2)) / denominator;

   LOG4CPLUS_TRACE(m_logger, "Calculated merge point:  (" << Units::MetersLength(x_merge) << ","
                                                          << Units::MetersLength(y_merge) << ")");
}

/**
 * An algorithm that calculates an extrapolated time for a provided distance-to-go. Uses constant ground speed as an
 * assumption since only one state is provided.
 *
 * @param ownship_dtg: the distance-to-go on ownship's route
 * @param oldest_target_state: the target state to use for the extrapolation
 * @param ownship_horizontal_traj: ownship's horizontal path
 * @param extrapolated_target_time: returned time
 * @param projected_x, projected_y values on the horizontal path at time and dtg
 */
void IMUtils::CalculateTimeBasedExtrapolate(const Units::Length &ownship_dtg,
                                            const interval_management::open_source::AircraftState &oldest_target_state,
                                            const std::vector<HorizontalPath> &ownship_horizontal_traj,
                                            Units::Time &extrapolated_target_time, Units::Length &projected_x,
                                            Units::Length &projected_y, Units::Length &projected_distance_to_go) {
   Units::Length target_dtg = Units::ZERO_LENGTH, targetx, targety;
   bool foundTargPos =
         ProjectTargetPosition(Units::FeetLength(oldest_target_state.m_x), Units::FeetLength(oldest_target_state.m_y),
                               ownship_horizontal_traj,  // project onto _ownship's_ horizontal path
                               targetx, targety,
                               target_dtg);  // target_dtg needed on _ownship's_ horizontal path
   if (!foundTargPos) {
      string msg = "No target position could be found on ownship's horizontal path.";
      LOG4CPLUS_FATAL(m_logger, msg);
      throw logic_error(msg);
   }

   Units::Length delta_distance = ownship_dtg - target_dtg;  // there are valid scenarios in which target_dtg >
                                                             // ownship_dtg; negative is appropriate
   Units::Speed target_ground_speed = oldest_target_state.GetGroundSpeed();
   Units::Time delta_time = delta_distance / target_ground_speed;

   extrapolated_target_time = Units::SecondsTime(oldest_target_state.GetTimeStamp().value()) -
                              delta_time;  // time when target was at dtg_ownship
   interval_management::open_source::AircraftState extrapstate, projstate(oldest_target_state);
   projstate.m_x = Units::FeetLength(targetx).value();
   projstate.m_y = Units::FeetLength(targety).value();
   extrapstate.Extrapolate(projstate, extrapolated_target_time);
   projected_x = Units::FeetLength(extrapstate.m_x);
   projected_y = Units::FeetLength(extrapstate.m_y);
   projected_distance_to_go = target_dtg;

   return;
}

/**
 * An algorithm that calculates an extrapolated time for a provided distance-to-go. Uses constant ground speed as an
 * assumption since only one state is provided.  Uses AlongPathDistanceCalculator for projected position.
 *
 * @param ownship_dtg: the distance-to-go on ownship's route
 * @param oldest_target_state: the target state to use for the extrapolation
 * @param ownship_horizontal_traj: ownship's horizontal path
 * @param extrapolated_target_time: returned time
 * @param projected_x, projected_y values on the horizontal path at time and dtg
 */
void IMUtils::CalculateTimeBasedExtrapolate(const Units::Length &ownship_dtg,
                                            const interval_management::open_source::AircraftState &oldest_target_state,
                                            AlongPathDistanceCalculator &distance_calculator,
                                            Units::Time &extrapolated_target_time, Units::Length &projected_x,
                                            Units::Length &projected_y, Units::Length &projected_distance_to_go) {
   Units::Length target_dtg = Units::ZERO_LENGTH, targetx, targety;

   bool found_target_distance = distance_calculator.CalculateAlongPathDistanceFromPosition(
         Units::FeetLength(oldest_target_state.m_x), Units::FeetLength(oldest_target_state.m_y), target_dtg);

   if (found_target_distance) {
      found_target_distance =
            ProjectTargetPositionFromDistance(target_dtg, distance_calculator.GetHorizontalPath(), targetx, targety);
   }

   if (!found_target_distance) {
      string msg = "No target position could be found on ownship's horizontal path.";
      LOG4CPLUS_FATAL(m_logger, msg);
      throw logic_error(msg);
   }

   Units::Length delta_distance = ownship_dtg - target_dtg;  // there are valid scenarios in which target_dtg >
                                                             // ownship_dtg; negative is appropriate
   Units::Speed target_ground_speed = oldest_target_state.GetGroundSpeed();
   Units::Time delta_time = delta_distance / target_ground_speed;

   extrapolated_target_time = Units::SecondsTime(oldest_target_state.GetTimeStamp().value()) -
                              delta_time;  // time when target was at dtg_ownship
   interval_management::open_source::AircraftState extrapstate, projstate(oldest_target_state);
   projstate.m_x = Units::FeetLength(targetx).value();
   projstate.m_y = Units::FeetLength(targety).value();
   extrapstate.Extrapolate(projstate, extrapolated_target_time);
   projected_x = Units::FeetLength(extrapstate.m_x);
   projected_y = Units::FeetLength(extrapstate.m_y);
   projected_distance_to_go = target_dtg;

   return;
}

/**
 * Calculates an estimated state on target's horizontal path given the input parameters. Assumes constant groundspeed
 * of the target aircraft. Assumes the
 *
 * @see #calcTimeBasedExtrapolate()
 * @param current_ownship_state
 * @param oldest_target_state
 * @param ownship_horizontal_traj
 * @param target_horizontal_traj
 * @param measured_spacing_interval: returned estimated MSI in time
 * @param aircraft_state_to_return : returned estimated aircraft state on target's horizontal path
 */
void IMUtils::GetTimeBasedExtrapolateState(const interval_management::open_source::AircraftState &current_ownship_state,
                                           const interval_management::open_source::AircraftState &oldest_target_state,
                                           const std::vector<HorizontalPath> &ownship_horizontal_traj,
                                           const std::vector<HorizontalPath> &target_horizontal_traj,
                                           Units::Time &measured_spacing_interval,
                                           interval_management::open_source::AircraftState &aircraft_state_to_return) {
   Units::Length ownship_dtg = Units::ZERO_LENGTH;
   Units::UnsignedAngle ownship_track = Units::ZERO_ANGLE;
   AircraftCalculations::LegacyGetPathLengthFromPosition(Units::FeetLength(current_ownship_state.m_x),
                                                         Units::FeetLength(current_ownship_state.m_y),
                                                         ownship_horizontal_traj, ownship_dtg, ownship_track);
   Units::Time extrapolated_target_time;
   Units::Length projX, projY, projDTG;
   CalculateTimeBasedExtrapolate(ownship_dtg, oldest_target_state, ownship_horizontal_traj, extrapolated_target_time,
                                 projX, projY, projDTG);
   Units::Time msi = Units::SecondsTime(current_ownship_state.GetTimeStamp().value()) -
                     extrapolated_target_time;  // the spacing interval in time

   // Now calculate an estimated target state that is on _target's_ horizontal path at "extrapolated_target_time"
   Units::Length x_out = Units::ZERO_LENGTH;
   Units::Length y_out = Units::ZERO_LENGTH;
   Units::UnsignedAngle track_out = Units::ZERO_ANGLE;
   int traj_index_out = -1;
   IMUtils::GetPositionFromPathLength(
         ownship_dtg, target_horizontal_traj, Units::Angle(), x_out, y_out, track_out,
         traj_index_out);  // this will return even if ownship_dtg is off the back of the route

   EarthModel::LocalPositionEnu extrapolated_position;
   extrapolated_position.x = x_out;
   extrapolated_position.y = y_out;
   extrapolated_position.z = oldest_target_state.GetPositionZ();
   aircraft_state_to_return.Create(oldest_target_state.GetId(), extrapolated_target_time, extrapolated_position,
                                   oldest_target_state.GetSpeedXd(), oldest_target_state.GetSpeedYd(),
                                   oldest_target_state.GetSpeedZd(), Units::zero(), Units::zero(), Units::zero(),
                                   Units::zero(), Units::zero(), Units::zero(), Units::zero());
   aircraft_state_to_return.m_distance_to_go_meters = Units::MetersLength(ownship_dtg).value();

   measured_spacing_interval = msi;
}

/**
 * This intentionally shadows AircraftCalculations::getPosFromPathLength(). This method will handle the situation in
 * which the dist_in value is longer than the route.
 *
 * NOTE: only use this when you truly need this protection and not the error associated with being off the
 * HorizontalPath.
 *
 * @param distance_to_go_in
 * @param horizontal_path_in
 * @param x_out
 * @param y_out
 * @param reverse_course_out
 * @param horizontal_path_index
 */
void IMUtils::GetPositionFromPathLength(const Units::Length distance_to_go_in,
                                        const std::vector<HorizontalPath> &horizontal_path_in,
                                        const Units::Angle ownship_true_heading, Units::Length &x_out,
                                        Units::Length &y_out, Units::UnsignedAngle &course_out,
                                        int &horizontal_path_index) {
   if (distance_to_go_in > Units::MetersLength(horizontal_path_in.back().m_path_length_cumulative_meters)) {
      course_out = ownship_true_heading;

      horizontal_path_index = 0;
      Units::UnsignedRadiansAngle crs(course_out);
      crs.normalize();  // make sure it's on the correct interval

      const Units::Length diff(distance_to_go_in -
                               Units::MetersLength(horizontal_path_in.back().m_path_length_cumulative_meters));

      x_out = Units::MetersLength(horizontal_path_in.back().GetXPositionMeters()) + diff * cos(crs);
      y_out = Units::MetersLength(horizontal_path_in.back().GetYPositionMeters()) + diff * sin(crs);
   } else {
      AircraftCalculations::LegacyGetPositionFromPathLength(distance_to_go_in, horizontal_path_in, x_out, y_out,
                                                            course_out, horizontal_path_index);
   }
}

bool IMUtils::ProjectTargetPosition(const Units::Length x_target, const Units::Length y_target,
                                    const std::vector<HorizontalPath> &ownship_horizontal_path,
                                    Units::Length &x_projected, Units::Length &y_projected, Units::Length &dtg) {
   // Computes the projected position of target aircraft upon the horizontal trajectory
   // ownship.  In other words, calculates which segment of the horizontal trajectory
   // the target aircraft is adjacent to, then calculates the perpendicular from the
   // route segment through the target aircraft position.

   // return true if projected position can be found.
   // returns false if target position is more than 2.0 miles from ownship trajectory
   // returns true if not within route but within 80 meters of start point or end point
   const Units::NauticalMilesLength startOfRouteTol(2.0);
   x_projected = Units::NegInfinity();
   y_projected = Units::NegInfinity();
   Units::UnsignedRadiansAngle dummyTrack(0.0);

   const bool b0 = GetPathLengthFromPosition(x_target, y_target, ownship_horizontal_path, dtg, dummyTrack);
   if (!b0) {
      return false;
   } else {
      if (dtg < BEYOND_END_OF_ROUTE_TOL) {
         return false;
      } else if (dtg < Units::ZERO_LENGTH) {
         x_projected = Units::ZERO_LENGTH;
         y_projected = Units::ZERO_LENGTH;
         return true;
      } else {
         // Find projected position
         if (dtg - Units::MetersLength(ownship_horizontal_path.back().m_path_length_cumulative_meters) >
             startOfRouteTol) {
            LOG4CPLUS_ERROR(m_logger, "dtg - m_path_length_cumulative_meters: "
                                            << (Units::MetersLength(dtg).value() -
                                                ownship_horizontal_path.back().m_path_length_cumulative_meters));
            return false;
         } else {
            // Valid position. Now calculate
            if (Units::MetersLength(dtg).value() > ownship_horizontal_path.back().m_path_length_cumulative_meters) {
               x_projected = Units::MetersLength(ownship_horizontal_path.back().GetXPositionMeters());
               y_projected = Units::MetersLength(ownship_horizontal_path.back().GetYPositionMeters());
            } else {
               int indexTraj;
               GetPositionFromPathLength(dtg, ownship_horizontal_path, Units::Angle(), x_projected, y_projected,
                                         dummyTrack, indexTraj);
            }
            return true;
         }
      }
   }
}

bool IMUtils::ProjectTargetPositionFromDistance(const Units::Length dtg,
                                                const std::vector<HorizontalPath> &ownship_horizontal_path,
                                                Units::Length &x_projected, Units::Length &y_projected) {
   const Units::NauticalMilesLength startOfRouteTol(2.0);
   x_projected = Units::NegInfinity();
   y_projected = Units::NegInfinity();
   Units::UnsignedRadiansAngle dummyTrack(0.0);

   if (dtg < BEYOND_END_OF_ROUTE_TOL) {
      return false;
   } else if (dtg < Units::ZERO_LENGTH) {
      x_projected = Units::ZERO_LENGTH;
      y_projected = Units::ZERO_LENGTH;
      return true;
   } else {
      // Find projected position
      if (dtg - Units::MetersLength(ownship_horizontal_path.back().m_path_length_cumulative_meters) > startOfRouteTol) {
         LOG4CPLUS_ERROR(m_logger, "dtg - m_path_length_cumulative_meters: "
                                         << (Units::MetersLength(dtg).value() -
                                             ownship_horizontal_path.back().m_path_length_cumulative_meters));
         return false;
      } else {
         // Valid position. Now calculate
         if (Units::MetersLength(dtg).value() > ownship_horizontal_path.back().m_path_length_cumulative_meters) {
            x_projected = Units::MetersLength(ownship_horizontal_path.back().GetXPositionMeters());
            y_projected = Units::MetersLength(ownship_horizontal_path.back().GetYPositionMeters());
         } else {
            int indexTraj;
            GetPositionFromPathLength(dtg, ownship_horizontal_path, Units::Angle(), x_projected, y_projected,
                                      dummyTrack, indexTraj);
         }
         return true;
      }
   }
}

bool IMUtils::GetCrossingTime(
      const Units::Length current_dtg,
      const std::vector<interval_management::open_source::AircraftState> &aircraft_state_history,
      const std::vector<HorizontalPath> &horizontal_path, Units::Time &crossing_time, Units::Length &projected_x,
      Units::Length &projected_y) {
   // Computes crossing time for a current distance to go.  Calculations use positions
   // from aircraft states mapped into a horizontal trajectory.
   // returns true-crossing time calculation valid.
   //         false-crossing time calculation not valid.
   // returns crossingtime
   // returns x/y projected location associated with crossingtime

   // Developer's Note: the incoming aircraftstates vector is assumed to be for the target aircraft. Each state
   // must be projected onto ownship's path prior to use!

   /*
    * The logic will fall into these cases:
    * - A: currentdist is before aircraftstates[0]
    * - B: normal situation in which currentdist is between two aircraftstates elements
    * - C: currentdist is not bounded by aircraftstates, must be after last state (given Case A)
    * - D: one aircraftstates element
    * - E: aircraftstates has no elements
    */

   Units::Length projDTG = Units::ZERO_LENGTH;
   bool valid = false;
   crossing_time = Units::ZERO_TIME;

   // Case E returns immediately
   if (aircraft_state_history.empty()) {
      return valid;  // can't do anything
   }

   // Case D returns immediately
   if (aircraft_state_history.size() == 1) {
      // handle case D:  one aircraftstates element
      Units::Length ownshipdtg = current_dtg;
      CalculateTimeBasedExtrapolate(ownshipdtg, aircraft_state_history.back(), horizontal_path, crossing_time,
                                    projected_x, projected_y,
                                    projDTG);  // this will ensure that the state is projected onto ownship's route
      valid = true;
      return valid;
   }

   // check if the aircraft distance is greater than the starting distance of the history
   Units::MetersLength startDist;
   Units::Angle tempcourse;
   bool found = false;
   Units::MetersLength nextdistance;
   Units::MetersLength prevdistance;

   // find correct position on ownship's route
   int index = -1;
   Units::Length xProjected, yProjected, dtgProjected;
   const bool b0 = IMUtils::ProjectTargetPosition(
         Units::FeetLength(aircraft_state_history[0].m_x), Units::FeetLength(aircraft_state_history[0].m_y),
         horizontal_path, xProjected, yProjected,
         dtgProjected);  // if this returns false, do not allow case A to happen

   startDist = dtgProjected;
   if (current_dtg > startDist && b0) {
      // case A:  currentdist is before aircraftstates[0], index will be -1
      IMUtils::CalculateTimeBasedExtrapolate(current_dtg, aircraft_state_history[0], horizontal_path, crossing_time,
                                             projected_x, projected_y, projDTG);
      return true;
   }

   /*
    * NOTE: Early returns are above this! Do not add early return logic below!
    *
    * Case E, D, A have already been handled and returned. Below logic is for B and C.
    */
   Units::Length xProjectedBefore, yProjectedBefore, xProjectedAfter, yProjectedAfter;
   bool b1(false), b2(false);
   for (auto loop = 1; loop < aircraft_state_history.size() && !found; ++loop)  // AAES-459
   {
      // Calculate a distance-to-go for the next two states
      // This must be based on projected state positions and these can legitimately
      // fail when the states encompass a large achieve-by-then-maintain operation.
      if (b2) {
         // use calculation results from previous iteration
         b1 = b2;
         xProjectedBefore = xProjectedAfter;
         yProjectedBefore = yProjectedAfter;
         prevdistance = nextdistance;
      } else {
         // possibly first time through loop -- do a fresh calculation
         b1 = IMUtils::ProjectTargetPosition(Units::FeetLength(aircraft_state_history[loop - 1].m_x),
                                             Units::FeetLength(aircraft_state_history[loop - 1].m_y), horizontal_path,
                                             xProjectedBefore, yProjectedBefore, prevdistance);
      }
      b2 = IMUtils::ProjectTargetPosition(Units::FeetLength(aircraft_state_history[loop].m_x),
                                          Units::FeetLength(aircraft_state_history[loop].m_y), horizontal_path,
                                          xProjectedAfter, yProjectedAfter, nextdistance);

      const bool next_distance_is_valid = b2 || nextdistance < Units::zero();
      if (current_dtg >= nextdistance && current_dtg <= prevdistance && b1 && next_distance_is_valid) {
         // case B, currentdist is between aircraftstates[loop-1] and
         // aircraftstates[loop].  index will be set to loop.
         found = true;
      }
      index = loop;
   }
   // case C, fall through without setting found
   // index will be the last element in aircraftstates.
   // case D, aircraftstates has a single element, index is -1   (handled above)
   // case E, aircraftstates has no elements, index is -1   (handled above)

   // check if index is found and greater than 0
   if (index > 0 && found) {
      // handle case {A, B, C} using interpolation/extrapolation
      crossing_time = Units::SecondsTime((aircraft_state_history[index].GetTimeStamp().value() -
                                          aircraft_state_history[index - 1].GetTimeStamp().value()) /
                                               (nextdistance - prevdistance) * (current_dtg - prevdistance) +
                                         aircraft_state_history[index - 1].GetTimeStamp().value());

      // Calculate a projected state associated with the crossingtime
      interval_management::open_source::AircraftState crossingstate, beforestate(aircraft_state_history[index - 1]),
            afterstate(aircraft_state_history[index]);
      beforestate.m_x = Units::FeetLength(xProjectedBefore).value();
      beforestate.m_y = Units::FeetLength(yProjectedBefore).value();
      afterstate.m_x = Units::FeetLength(xProjectedAfter).value();
      afterstate.m_y = Units::FeetLength(yProjectedAfter).value();
      crossingstate.Interpolate(beforestate, afterstate, crossing_time);
      projected_x = Units::FeetLength(crossingstate.m_x);  // for return
      projected_y = Units::FeetLength(crossingstate.m_y);  // for return

      valid = true;
   } else {
      // Case C: The state vector does not encompass currentdist. Case A has already checked the zeroth state, so here
      // the decision must be to extrapolate from the last state to a crossing time in the future
      const bool b1 = IMUtils::ProjectTargetPosition(Units::FeetLength(aircraft_state_history.back().m_x),
                                                     Units::FeetLength(aircraft_state_history.back().m_y),
                                                     horizontal_path, xProjectedAfter, yProjectedAfter, nextdistance);
      interval_management::open_source::AircraftState extrapstate;
      if (b1) {
         extrapstate = aircraft_state_history.back();
      } else {
         string msg = "Could not determine a state to extrapolate from.";
         LOG4CPLUS_FATAL(m_logger, msg);
         throw logic_error(msg);
      }
      IMUtils::CalculateTimeBasedExtrapolate(current_dtg, extrapstate, horizontal_path, crossing_time, projected_x,
                                             projected_y, projDTG);
      valid = true;
   }
   return valid;
}

bool IMUtils::GetCrossingTime(
      const Units::Length current_dtg,
      const std::vector<interval_management::open_source::AircraftState> &aircraft_state_history,
      AlongPathDistanceCalculator &distance_calculator, Units::Time &crossing_time, Units::Length &projected_x,
      Units::Length &projected_y) {
   // Computes crossing time for a current distance to go.  Calculations use positions
   // from aircraft states mapped into a horizontal trajectory.
   // returns true-crossing time calculation valid.
   //         false-crossing time calculation not valid.
   // returns crossingtime
   // returns x/y projected location associated with crossingtime

   // Developer's Note: the incoming aircraftstates vector is assumed to be for the target aircraft. Each state
   // must be projected onto ownship's path prior to use!

   /*
    * The logic will fall into these cases:
    * - A: currentdist is before aircraftstates[0]
    * - B: normal situation in which currentdist is between two aircraftstates elements
    * - C: currentdist is not bounded by aircraftstates, must be after last state (given Case A)
    * - D: one aircraftstates element
    * - E: aircraftstates has no elements
    */

   std::vector<HorizontalPath> horizontal_path = distance_calculator.GetHorizontalPath();
   Units::Length projDTG = Units::ZERO_LENGTH;
   bool valid = false;
   crossing_time = Units::ZERO_TIME;

   // Case E returns immediately
   if (aircraft_state_history.empty()) {
      return valid;  // can't do anything
   }

   // Case D returns immediately
   if (aircraft_state_history.size() == 1) {
      // handle case D:  one aircraftstates element
      Units::Length ownshipdtg = current_dtg;
      CalculateTimeBasedExtrapolate(ownshipdtg, aircraft_state_history.back(), distance_calculator, crossing_time,
                                    projected_x, projected_y,
                                    projDTG);  // this will ensure that the state is projected onto ownship's route
      valid = true;
      return valid;
   }

   // Case A check if the aircraft distance is greater than the starting distance of the target history
   Units::MetersLength startDist;
   Units::Angle tempcourse;
   bool found = false;
   Units::MetersLength nextdistance;
   Units::MetersLength prevdistance;

   // project onto ownship's route (projection can fail depending on geometry)
   int index = -1;
   Units::Length xProjected, yProjected, dtgProjected;
   bool b0;
   try {
      b0 = distance_calculator.CalculateAlongPathDistanceFromPosition(Units::FeetLength(aircraft_state_history[0].m_x),
                                                                      Units::FeetLength(aircraft_state_history[0].m_y),
                                                                      dtgProjected);
      b0 = ProjectTargetPositionFromDistance(dtgProjected, distance_calculator.GetHorizontalPath(), xProjected,
                                             yProjected);
   } catch (logic_error &e) {
      b0 = false;
   }

   startDist = dtgProjected;
   if (current_dtg > startDist && b0) {
      // case A:  currentdist is before aircraftstates[0], index will be -1
      IMUtils::CalculateTimeBasedExtrapolate(current_dtg, aircraft_state_history[0], distance_calculator, crossing_time,
                                             projected_x, projected_y, projDTG);
      return true;
   }

   /*
    * NOTE: Early returns are above this! Do not add early return logic below!
    *
    * Case E, D, A have already been handled and returned. Below logic is for B and C.
    */
   Units::Length xProjectedBefore, yProjectedBefore, xProjectedAfter, yProjectedAfter;
   bool b1(false), b2(false);
   for (auto loop = 1; loop < aircraft_state_history.size() && !found; ++loop)  // AAES-459
   {
      // Calculate a distance-to-go for the next two states
      // This must be based on projected state positions and these can legitimately
      // fail when the states encompass a large achieve-by-then-maintain operation.
      if (b2) {
         // use calculation results from previous iteration
         b1 = b2;
         xProjectedBefore = xProjectedAfter;
         yProjectedBefore = yProjectedAfter;
         prevdistance = nextdistance;
      } else {
         // possibly first time through loop -- do a fresh calculation
         try {
            b1 = distance_calculator.CalculateAlongPathDistanceFromPosition(
                  Units::FeetLength(aircraft_state_history[loop - 1].m_x),
                  Units::FeetLength(aircraft_state_history[loop - 1].m_y), prevdistance);
         } catch (logic_error &e) {
            b1 = false;
         }
         if (b1) {
            b1 = ProjectTargetPositionFromDistance(prevdistance, distance_calculator.GetHorizontalPath(),
                                                   xProjectedBefore, yProjectedBefore);
         }
      }
      try {
         b2 = distance_calculator.CalculateAlongPathDistanceFromPosition(
               Units::FeetLength(aircraft_state_history[loop].m_x), Units::FeetLength(aircraft_state_history[loop].m_y),
               nextdistance);
      } catch (logic_error &e) {
         b2 = false;
      }
      if (b2) {
         b2 = ProjectTargetPositionFromDistance(nextdistance, distance_calculator.GetHorizontalPath(), xProjectedAfter,
                                                yProjectedAfter);
      }

      const bool next_distance_is_valid = b2 || nextdistance < Units::zero();
      if (current_dtg >= nextdistance && current_dtg <= prevdistance && b1 && next_distance_is_valid) {
         // case B, currentdist is between aircraftstates[loop-1] and
         // aircraftstates[loop].  index will be set to loop.
         found = true;
      }
      index = loop;
   }
   // case C, fall through without setting found
   // index will be the last element in aircraftstates.
   // case D, aircraftstates has a single element, index is -1   (handled above)
   // case E, aircraftstates has no elements, index is -1   (handled above)

   // check if index is found and greater than 0
   if (index > 0 && found) {
      // handle case {C} using interpolation
      crossing_time = Units::SecondsTime((aircraft_state_history[index].GetTimeStamp().value() -
                                          aircraft_state_history[index - 1].GetTimeStamp().value()) /
                                               (nextdistance - prevdistance) * (current_dtg - prevdistance) +
                                         aircraft_state_history[index - 1].GetTimeStamp().value());

      // Calculate a projected state associated with the crossingtime
      interval_management::open_source::AircraftState crossingstate, beforestate(aircraft_state_history[index - 1]),
            afterstate(aircraft_state_history[index]);
      beforestate.m_x = Units::FeetLength(xProjectedBefore).value();
      beforestate.m_y = Units::FeetLength(yProjectedBefore).value();
      afterstate.m_x = Units::FeetLength(xProjectedAfter).value();
      afterstate.m_y = Units::FeetLength(yProjectedAfter).value();
      crossingstate.Interpolate(beforestate, afterstate, crossing_time);
      projected_x = Units::FeetLength(crossingstate.m_x);  // for return
      projected_y = Units::FeetLength(crossingstate.m_y);  // for return

      valid = true;
   } else {
      // Case C: The state vector does not encompass currentdist. Case A has already checked the zeroth state, so here
      // the decision must be to extrapolate from the last state to a crossing time in the future
      bool b1;
      try {
         b1 = distance_calculator.CalculateAlongPathDistanceFromPosition(
               Units::FeetLength(aircraft_state_history.back().m_x),
               Units::FeetLength(aircraft_state_history.back().m_y), nextdistance);
      } catch (logic_error &e) {
         b1 = false;
      }
      if (b1) {
         b1 = ProjectTargetPositionFromDistance(nextdistance, distance_calculator.GetHorizontalPath(), xProjectedAfter,
                                                yProjectedAfter);
      }

      interval_management::open_source::AircraftState extrapstate;
      if (b1) {
         extrapstate = aircraft_state_history.back();
      } else {
         string msg = "Could not determine a state to extrapolate from.";
         LOG4CPLUS_FATAL(m_logger, msg);
         throw logic_error(msg);
      }
      IMUtils::CalculateTimeBasedExtrapolate(current_dtg, extrapstate, distance_calculator, crossing_time, projected_x,
                                             projected_y, projDTG);
      valid = true;
   }
   return valid;
}

/**
 * Calculate a state on ownship_horizontal_path at time extraptime. The returned state is the projection of target_state
 * onto ownship_horizontal_traj and then extrapolated to extraptime assuming constant ground speed.
 */
bool IMUtils::CalculateTargetStateAtTime(const interval_management::open_source::AircraftState &target_state,
                                         const std::vector<HorizontalPath> &ownship_horizontal_traj,
                                         const Units::Time extrapolation_time, const Units::Angle ownship_true_heading,
                                         interval_management::open_source::AircraftState &extrapolation_state) {

   interval_management::open_source::AircraftState projstate(target_state), stateattime;
   Units::FeetLength projx, projy, projdtg;
   const bool b0 =
         IMUtils::ProjectTargetPosition(Units::FeetLength(target_state.m_x), Units::FeetLength(target_state.m_y),
                                        ownship_horizontal_traj, projx, projy, projdtg);
   if (!b0) {
      return false;
   }

   Units::Time dt = Units::SecondsTime(target_state.GetTimeStamp().value()) -
                    extrapolation_time;  // sign is important, pos means going backwards
   Units::Length delta_dist = target_state.GetGroundSpeed() * dt;
   Units::Length dtg = projdtg + delta_dist;

   int index;
   Units::UnsignedAngle crs;
   Units::Length x, y;
   GetPositionFromPathLength(dtg, ownship_horizontal_traj, ownship_true_heading, x, y, crs, index);
   EarthModel::LocalPositionEnu extrapolated_position;
   extrapolated_position.x = x;
   extrapolated_position.y = y;
   extrapolated_position.z = target_state.GetPositionZ();
   extrapolation_state.Create(target_state.GetId(), extrapolation_time, extrapolated_position,
                              target_state.GetGroundSpeed() * Units::cos(crs),
                              target_state.GetGroundSpeed() * Units::sin(crs), Units::zero(), Units::zero(),
                              Units::zero(), Units::zero(), Units::zero(), Units::zero(), Units::zero(), Units::zero());

   return true;
}

/**
 * This method intentionally shadows AircraftCalculations::getPathLengthFromPos.
 *
 * This method protects against positions that are not on the hTraj and will handle that failure gracefully. Returns
 * false if the x/y position is not found on hTraj.
 *
 * Developers: Only used this method when the protection is warranted!
 */
bool IMUtils::GetPathLengthFromPosition(const Units::Length x, const Units::Length y,
                                        const std::vector<HorizontalPath> &horizontal_path,
                                        Units::Length &distance_to_go, Units::Angle &track) {

   try {
      /*
       * Note: Do not attempt to replace this call with an instance of AlongPathDistanceCalculator. The caller's
       * algorithm will not support this. To use AlongPathDistanceCalculator, the entire algoirthm in this method's
       * caller also needs to be redesigned.
       */
      AircraftCalculations::LegacyGetPathLengthFromPosition(x, y, horizontal_path, distance_to_go, track);
      return true;
   } catch (logic_error &e) {
      return false;
   }
}

Units::Speed IMUtils::FindLargestSpeedConstraint(const AircraftIntent &aircraft_intent) {
   Units::Speed max_ias_constraint = Waypoint::MIN_SPEED_CONSTRAINT;
   const Units::KnotsSpeed max_ias_input(Waypoint::MAX_SPEED_CONSTRAINT);
   const struct AircraftIntent::RouteData route_info = aircraft_intent.GetRouteData();
   for (auto spdidx = 0; spdidx < aircraft_intent.GetNumberOfWaypoints() - 1; ++spdidx) {
      const Units::Speed this_speed = route_info.m_high_speed_constraint[spdidx];
      if (this_speed < max_ias_input) {
         if (this_speed >= max_ias_constraint) {
            max_ias_constraint = this_speed;
         } else {
            break;
         }
      }
   }

   return max_ias_constraint;
}

Units::Time IMUtils::InterpolateTimeAtDtg(const Units::Length target_dtg, const Units::Time time1,
                                          const Units::Time time2, const Units::Length dtg1, const Units::Length dtg2) {
   Units::Length mu = dtg2 - dtg1;
   return time1 + ((time2 - time1) * (target_dtg - dtg1)) / mu;
}

interval_management::open_source::AircraftState IMUtils::GetTargetStateOnOwnshipPathForDtg(
      const vector<interval_management::open_source::AircraftState> &target_adsb_history,
      const std::vector<HorizontalPath> &ownship_horizontal_path, const Units::Length target_distance,
      const Units::Angle ownship_true_heading) {
   interval_management::open_source::AircraftState result;
   interval_management::open_source::AircraftState target_state_to_extrapolate_from;

   if (target_distance >= Units::MetersLength(target_adsb_history.front().m_distance_to_go_meters) &&
       Units::MetersLength(target_adsb_history.front().m_distance_to_go_meters).value() > -1) {
      target_state_to_extrapolate_from = target_adsb_history.front();
      IMUtils::CalculateTargetStateAtDistance(target_state_to_extrapolate_from, ownship_horizontal_path,
                                              target_distance, ownship_true_heading, result);
      return result;
   } else if (target_distance <= Units::MetersLength(target_adsb_history.back().m_distance_to_go_meters)) {
      target_state_to_extrapolate_from = target_adsb_history.back();
      IMUtils::CalculateTargetStateAtDistance(target_state_to_extrapolate_from, ownship_horizontal_path,
                                              target_distance, ownship_true_heading, result);
      return result;
   }

   int ix = 0;
   bool found = false;
   while (ix < target_adsb_history.size() && !found) {
      if (target_distance > Units::MetersLength(target_adsb_history[ix].m_distance_to_go_meters)) {
         ix--;
      } else if (ix < (target_adsb_history.size() - 1) &&
                 (target_distance < Units::MetersLength(target_adsb_history[ix + 1].m_distance_to_go_meters))) {
         ix++;
      } else if (target_distance < Units::MetersLength(target_adsb_history[ix].m_distance_to_go_meters)) {
         found = true;
         target_state_to_extrapolate_from = target_adsb_history[ix];
         ix++;
      }
   }

   // Interpolation
   if (ix != 0 && ix < target_adsb_history.size()) {
      interval_management::open_source::AircraftState before = target_adsb_history[ix - 1];
      interval_management::open_source::AircraftState after = target_adsb_history[ix];

      Units::Length beforex, beforey, beforedtg;
      Units::Length afterx, aftery, afterdtg;

      const bool b0 = IMUtils::ProjectTargetPosition(Units::FeetLength(before.m_x), Units::FeetLength(before.m_y),
                                                     ownship_horizontal_path, beforex, beforey, beforedtg);

      const bool b1 = IMUtils::ProjectTargetPosition(Units::FeetLength(after.m_x), Units::FeetLength(after.m_y),
                                                     ownship_horizontal_path, afterx, aftery, afterdtg);

      before.m_x = Units::FeetLength(beforex).value();
      before.m_y = Units::FeetLength(beforey).value();
      after.m_x = Units::FeetLength(afterx).value();
      after.m_y = Units::FeetLength(aftery).value();

      if (b0 && b1) {
         const Units::Time target_time_at_dtg =
               IMUtils::InterpolateTimeAtDtg(target_distance, Units::SecondsTime(before.GetTimeStamp().value()),
                                             Units::SecondsTime(after.GetTimeStamp().value()),
                                             Units::MetersLength(beforedtg), Units::MetersLength(afterdtg));

         result.Interpolate(before, after, target_time_at_dtg);
      } else {
         std::string msg = "Failure to project a state onto ownship's path";
         LOG4CPLUS_FATAL(m_logger, msg);
         throw std::logic_error(msg);
      }
   }
   return result;
}

/**
 * Calculate a state on ownship_horizontal_path at distance extrapolation_distance. The returned state is the projection
 * of target_state onto ownship_horizontal_traj and then extrapolated to extrapolation_distance assuming constant ground
 * speed.
 */
void IMUtils::CalculateTargetStateAtDistance(const interval_management::open_source::AircraftState &target_state,
                                             const std::vector<HorizontalPath> &ownship_horizontal_traj,
                                             const Units::Length extrapolation_distance,
                                             const Units::Angle ownship_true_heading,
                                             interval_management::open_source::AircraftState &extrapstate) {

   interval_management::open_source::AircraftState projstate(target_state);
   interval_management::open_source::AircraftState stateattime;
   Units::FeetLength projx, projy, projdtg;
   const bool b0 =
         IMUtils::ProjectTargetPosition(Units::FeetLength(target_state.m_x), Units::FeetLength(target_state.m_y),
                                        ownship_horizontal_traj, projx, projy, projdtg);
   if (!b0) {
      LOG4CPLUS_FATAL(m_logger, "Unable to extrapoloate a target state to time "
                                      << Units::MetersLength(extrapolation_distance).value() << ". Cannot proceed.");
      // target_state.DumpParms("target_state to extrapolate from:");

      throw logic_error("Cannot proceed.");
   }

   Units::Length delta_dist = extrapolation_distance - Units::MetersLength(target_state.m_distance_to_go_meters);
   Units::Length dtg = projdtg + delta_dist;

   Units::Time dt = delta_dist / target_state.GetGroundSpeed();
   Units::Time extrapolation_state_time = Units::SecondsTime(target_state.GetTimeStamp().value()) + dt;

   int index;
   Units::UnsignedAngle crs;
   Units::Length x, y;
   GetPositionFromPathLength(dtg, ownship_horizontal_traj, ownship_true_heading, x, y, crs, index);

   EarthModel::LocalPositionEnu extrapolated_position;
   extrapolated_position.x = x;
   extrapolated_position.y = y;
   extrapolated_position.z = target_state.GetPositionZ();
   extrapstate.Create(target_state.GetId(), extrapolation_state_time, extrapolated_position,
                      target_state.GetGroundSpeed() * Units::cos(crs), target_state.GetGroundSpeed() * Units::sin(crs),
                      Units::zero(), Units::zero(), Units::zero(), Units::zero(), Units::zero(), Units::zero(),
                      Units::zero(), Units::zero());
}

interval_management::open_source::AircraftState IMUtils::GetProjectedTargetState(
      const std::vector<interval_management::open_source::AircraftState> &target_state_history,
      const std::vector<HorizontalPath> &ownship_horizontal_traj, const Units::Time target_time,
      const Units::Angle ownship_true_heading, bool &target_state_is_valid) {
   interval_management::open_source::AircraftState target_state;
   interval_management::open_source::AircraftState extrapolateFromThis;

   bool found = false;
   target_state_is_valid = false;

   if (target_time < Units::SecondsTime(target_state_history.front().GetTimeStamp().value())) {
      found = true;
      target_state_is_valid = true;
      extrapolateFromThis = target_state_history.front();
   } else if (target_time == Units::SecondsTime(target_state_history.front().GetTimeStamp().value())) {
      found = true;
      target_state_is_valid = true;
      extrapolateFromThis = target_state_history.front();
   } else if (target_time == Units::SecondsTime(target_state_history.back().GetTimeStamp().value())) {
      found = true;
      target_state_is_valid = true;
      extrapolateFromThis = target_state_history.back();
   } else if (target_time > Units::SecondsTime(target_state_history.back().GetTimeStamp().value())) {
      found = true;
      extrapolateFromThis = target_state_history.back();
   }

   int ix = 0;
   while (ix < target_state_history.size() && !found) {
      if (target_time < Units::SecondsTime(target_state_history[ix].GetTimeStamp().value())) {
         ix--;
      } else if (ix < (target_state_history.size() - 1) &&
                 target_time > Units::SecondsTime(target_state_history[ix + 1].GetTimeStamp().value())) {
         ix++;
      } else if (target_time > Units::SecondsTime(target_state_history[ix].GetTimeStamp().value())) {
         found = true;
         extrapolateFromThis = target_state_history[ix];
         ix++;
      }
   }

   if (ix != 0 && ix < target_state_history.size()) {
      interval_management::open_source::AircraftState before(target_state_history[ix - 1]),
            after(target_state_history[ix]);
      Units::Length beforex, beforey, beforedtg;
      Units::Length afterx, aftery, afterdtg;
      const bool b0 = IMUtils::ProjectTargetPosition(Units::FeetLength(before.m_x), Units::FeetLength(before.m_y),
                                                     ownship_horizontal_traj, beforex, beforey, beforedtg);
      const bool b1 = IMUtils::ProjectTargetPosition(Units::FeetLength(after.m_x), Units::FeetLength(after.m_y),
                                                     ownship_horizontal_traj, afterx, aftery, afterdtg);
      before.m_x = Units::FeetLength(beforex).value();
      before.m_y = Units::FeetLength(beforey).value();
      after.m_x = Units::FeetLength(afterx).value();
      after.m_y = Units::FeetLength(aftery).value();

      if (b0 && b1) {
         target_state.Interpolate(before, after, target_time);
         target_state_is_valid = true;
      } else {
         // This probably means that the bounding states cannot be projected onto the
         // horizontal paht of ownship. This is mathematically valid. The method must return false.
         target_state_is_valid = false;
      }
   } else {
      // Extrapolate along ownship path. This can fail if no possible state on the horizontal_traj. See AAES-798 & test
      bool is_extrapolation_valid = IMUtils::CalculateTargetStateAtTime(
            extrapolateFromThis, ownship_horizontal_traj, target_time, ownship_true_heading, target_state);

      target_state_is_valid = target_state_is_valid && is_extrapolation_valid;
   }

   return target_state;
}

interval_management::open_source::AircraftState IMUtils::GetProjectedTargetState(
      AlongPathDistanceCalculator &distance_calculator,
      const std::vector<interval_management::open_source::AircraftState> &target_state_history,
      const std::vector<HorizontalPath> &ownship_horizontal_traj, const Units::Time target_time,
      const Units::Angle ownship_true_heading, bool &target_state_is_valid) {
   interval_management::open_source::AircraftState target_state;
   interval_management::open_source::AircraftState extrapolateFromThis;

   bool found = false;
   target_state_is_valid = false;

   if (target_time < Units::SecondsTime(target_state_history.front().GetTimeStamp().value())) {
      found = true;
      target_state_is_valid = true;
      extrapolateFromThis = target_state_history.front();
   } else if (target_time == Units::SecondsTime(target_state_history.front().GetTimeStamp().value())) {
      found = true;
      target_state_is_valid = true;
      extrapolateFromThis = target_state_history.front();
   } else if (target_time == Units::SecondsTime(target_state_history.back().GetTimeStamp().value())) {
      found = true;
      target_state_is_valid = true;
      extrapolateFromThis = target_state_history.back();
   } else if (target_time > Units::SecondsTime(target_state_history.back().GetTimeStamp().value())) {
      found = true;
      extrapolateFromThis = target_state_history.back();
   }

   int ix = 0;
   while (ix < target_state_history.size() && !found) {
      if (target_time < Units::SecondsTime(target_state_history[ix].GetTimeStamp().value())) {
         ix--;
      } else if (ix < (target_state_history.size() - 1) &&
                 target_time > Units::SecondsTime(target_state_history[ix + 1].GetTimeStamp().value())) {
         ix++;
      } else if (target_time > Units::SecondsTime(target_state_history[ix].GetTimeStamp().value())) {
         found = true;
         extrapolateFromThis = target_state_history[ix];
         ix++;
      }
   }

   if (ix != 0 && ix < target_state_history.size()) {
      interval_management::open_source::AircraftState before(target_state_history[ix - 1]),
            after(target_state_history[ix]);
      Units::Length beforex, beforey, beforedtg;
      Units::Length afterx, aftery, afterdtg;
      Units::Length before_dtg, after_dtg;
      Units::UnsignedRadiansAngle before_out_crs, after_out_crs;
      int dummy_index;

      bool b0 = distance_calculator.CalculateAlongPathDistanceFromPosition(Units::FeetLength(before.m_x),
                                                                           Units::FeetLength(before.m_y), before_dtg);
      if (b0) {
         b0 = AircraftCalculations::LegacyGetPositionFromPathLength(before_dtg, ownship_horizontal_traj, beforex,
                                                                    beforey, before_out_crs, dummy_index);
      }

      bool b1 = distance_calculator.CalculateAlongPathDistanceFromPosition(Units::FeetLength(after.m_x),
                                                                           Units::FeetLength(after.m_y), after_dtg);
      if (b1) {
         b1 = AircraftCalculations::LegacyGetPositionFromPathLength(after_dtg, ownship_horizontal_traj, afterx, aftery,
                                                                    after_out_crs, dummy_index);
      }

      before.m_x = Units::FeetLength(beforex).value();
      before.m_y = Units::FeetLength(beforey).value();
      after.m_x = Units::FeetLength(afterx).value();
      after.m_y = Units::FeetLength(aftery).value();

      if (b0 && b1) {
         target_state.Interpolate(before, after, target_time);
         target_state_is_valid = true;
      } else {
         // This probably means that the bounding states cannot be projected onto the
         // horizontal paht of ownship. This is mathematically valid. The method must return false.
         target_state_is_valid = false;
      }
   } else {
      // Extrapolate along ownship path. This can fail if no possible state on the horizontal_traj. See AAES-798 & test
      /*      bool is_extrapolation_valid = IMUtils::CalculateTargetStateAtTime(extrapolateFromThis,
                                                                              ownship_horizontal_traj,
                                                                              target_time, ownship_true_heading,
                                                                              target_state);
      */
      Units::Length ownpathdtg;
      int dummyindex;
      Units::FeetLength tgt_x, tgt_y;
      Units::UnsignedRadiansAngle dummyangle;
      bool is_extrapolation_valid = distance_calculator.CalculateAlongPathDistanceFromPosition(
            Units::FeetLength(extrapolateFromThis.m_x), Units::FeetLength(extrapolateFromThis.m_y), ownpathdtg);
      if (is_extrapolation_valid) {
         is_extrapolation_valid = AircraftCalculations::LegacyGetPositionFromPathLength(
               ownpathdtg, ownship_horizontal_traj, tgt_x, tgt_y, dummyangle, dummyindex);
      }
      if (is_extrapolation_valid) {
         Units::Time dt = Units::SecondsTime(extrapolateFromThis.GetTimeStamp().value()) - target_time;
         // sign is important, pos means going backwards
         Units::Length delta_dist = extrapolateFromThis.GetGroundSpeed() * dt;
         Units::Length dtg = ownpathdtg + delta_dist;
         GetPositionFromPathLength(dtg, ownship_horizontal_traj, ownship_true_heading, tgt_x, tgt_y, dummyangle,
                                   dummyindex);
         EarthModel::LocalPositionEnu extrapolated_position;
         extrapolated_position.x = tgt_x;
         extrapolated_position.y = tgt_y;
         extrapolated_position.z = extrapolateFromThis.GetPositionZ();
         target_state.Create(extrapolateFromThis.GetId(), Units::SecondsTime(target_time), extrapolated_position,
                             extrapolateFromThis.GetGroundSpeed() * Units::cos(dummyangle),
                             extrapolateFromThis.GetGroundSpeed() * Units::sin(dummyangle), Units::zero(),
                             Units::zero(), Units::zero(), Units::zero(), Units::zero(), Units::zero(), Units::zero(),
                             Units::zero());
      }

      target_state_is_valid = target_state_is_valid && is_extrapolation_valid;
   }

   return target_state;
}

Units::SignedAngle IMUtils::CalculateTrackAngle(const std::list<Units::Angle> angle_history) {

   // Average the track angles
   // The tricky part is avoiding a wrap effect.
   // Compute total using both signed and unsigned,
   // But discard elements in danger of wrapping
   int signed_warning_count(0), unsigned_warning_count(0);
   Units::RadiansAngle total_signed(0), total_unsigned(0);
   for (auto itr = angle_history.begin(); itr != angle_history.end(); ++itr) {
      Units::SignedDegreesAngle x_signed = *itr;
      Units::UnsignedDegreesAngle x_unsigned = *itr;
      if (x_signed < Units::DegreesAngle(45) && x_signed > Units::DegreesAngle(-45)) {
         unsigned_warning_count++;
      } else {
         total_unsigned += x_unsigned;
      }
      if (x_unsigned < Units::DegreesAngle(225) && x_unsigned > Units::DegreesAngle(135)) {
         signed_warning_count++;
      } else {
         total_signed += x_signed;
      }
   }

   Units::SignedAngle result;
   if (signed_warning_count < unsigned_warning_count) {
      result = total_signed / (angle_history.size() - signed_warning_count);
   } else {
      result = total_unsigned / (angle_history.size() - unsigned_warning_count);
   }

   // if both totals have discards, log an error
   if (signed_warning_count > 0 && unsigned_warning_count > 0) {
      LOG4CPLUS_ERROR(m_logger, "Track angle history has " << signed_warning_count << " angles near 180 and "
                                                           << unsigned_warning_count
                                                           << " near 0.  This is not normal.");
   }
   return result;
}

interval_management::open_source::AircraftState IMUtils::ConvertToIntervalManagementAircraftState(
      const aaesim::open_source::AircraftState &aircraft_state) {
   interval_management::open_source::AircraftState new_state;
   return new_state.Create(
         aircraft_state.m_id, Units::SecondsTime(aircraft_state.m_time),
         EarthModel::LocalPositionEnu::Of(aircraft_state.GetPositionX(), aircraft_state.GetPositionY(),
                                          aircraft_state.GetPositionZ()),
         aircraft_state.GetSpeedXd(), aircraft_state.GetSpeedYd(), aircraft_state.GetSpeedZd(),
         Units::RadiansAngle(aircraft_state.m_gamma), Units::MetersPerSecondSpeed(aircraft_state.m_Vwx),
         Units::MetersPerSecondSpeed(aircraft_state.m_Vwy), Units::MetersPerSecondSpeed(aircraft_state.m_Vw_para),
         Units::MetersPerSecondSpeed(aircraft_state.m_Vw_perp), aircraft_state.m_sensed_temperature,
         aircraft_state.m_psi);
}

aaesim::open_source::AircraftState IMUtils::ConvertToAaesimAircraftState(
      const interval_management::open_source::AircraftState &aircraft_state) {
   aaesim::open_source::AircraftState return_state;
   return_state.m_x = Units::FeetLength(aircraft_state.GetPositionX()).value();
   return_state.m_y = Units::FeetLength(aircraft_state.GetPositionY()).value();
   return_state.m_z = Units::FeetLength(aircraft_state.GetPositionZ()).value();
   return_state.m_xd = Units::FeetPerSecondSpeed(aircraft_state.m_xd).value();
   return_state.m_yd = Units::FeetPerSecondSpeed(aircraft_state.m_yd).value();
   return_state.m_zd = Units::FeetPerSecondSpeed(aircraft_state.m_zd).value();
   return_state.m_id = aircraft_state.GetId();
   return_state.m_time = aircraft_state.GetTimeStamp().value();
   return_state.m_psi = aircraft_state.GetPsi();
   return_state.m_gamma = aircraft_state.GetGamma().value();
   return_state.m_Vwx = Units::MetersPerSecondSpeed(aircraft_state.GetSensedWindEastComponent()).value();
   return_state.m_Vwy = Units::MetersPerSecondSpeed(aircraft_state.GetSensedWindNorthComponent()).value();
   return_state.m_Vw_para = Units::MetersPerSecondSpeed(aircraft_state.GetSensedWindParallelComponent()).value();
   return_state.m_Vw_perp = Units::MetersPerSecondSpeed(aircraft_state.GetSensedWindPerpendicularComponent()).value();
   return_state.m_sensed_temperature = aircraft_state.GetSensedTemperature();
   return return_state;
}