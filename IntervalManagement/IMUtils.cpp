// ****************************************************************************
// NOTICE
//
// This is the copyright work of The MITRE Corporation, and was produced
// for the U. S. Government under Contract Number DTFAWA-10-C-00080, and
// is subject to Federal Aviation Administration Acquisition Management
// System Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV
// (Oct. 1996).  No other use other than that granted to the U. S.
// Government, or to those acting on behalf of the U. S. Government,
// under that Clause is authorized without the express written
// permission of The MITRE Corporation. For further information, please
// contact The MITRE Corporation, Contracts Office, 7515 Colshire Drive,
// McLean, VA  22102-7539, (703) 983-6000. 
//
// Copyright 2020 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "imalgs/IMUtils.h"
#include <stdexcept>
#include <imalgs/IMUtils.h>
#include <aaesim/ads_types.h>

#include "public/Scenario.h"
#include "public/AircraftCalculations.h"
#include "public/SimulationTime.h"

using namespace std;

log4cplus::Logger IMUtils::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("IMUtils"));

const int IMUtils::UNINITIALIZED_AIRCRAFT_ID = Scenario::AIRCRAFT_ID_NOT_IN_MAP; // link to Core definition
const Units::Length IMUtils::BEYOND_END_OF_ROUTE_TOL = Units::MetersLength(-300);  // don't reduce. See AAES-745

std::map<IMUtils::IMAlgorithmTypes, std::string> IMUtils::algorithm_type_dictionary =
      {{IMAlgorithmTypes::TIMEBASEDACHIEVEMUTABLEASG, "TIMEBASEDACHIEVEMUTABLEASG"},
       {IMAlgorithmTypes::TIMEBASEDACHIEVE,           "TIMEBASEDACHIEVE"},
       {IMAlgorithmTypes::DISTANCEBASEDACHIEVE,       "DISTANCEBASEDACHIEVE"},
       {IMAlgorithmTypes::KINETICACHIEVE,             "KINETICACHIEVE"},
       {IMAlgorithmTypes::KINETICTARGETACHIEVE,       "KINETICTARGETACHIEVE"},
       {IMAlgorithmTypes::NONE,                       "NONE"}};

/**
 * This signature is old. It intentionally shadows a method with more extensive returns.
 * @deprecated Use new signature
 * @param current_dtg
 * @param aircraft_state_history
 * @param horizontal_path
 * @param crossing_time
 * @return
 */
bool IMUtils::GetCrossingTime(const Units::Length current_dtg,
                              const std::vector<AircraftState> &aircraft_state_history,
                              const std::vector<HorizontalPath> &horizontal_path,
                              Units::Time &crossing_time) {
   Units::Length tmpx, tmpy;
   bool ret_val = GetCrossingTime(current_dtg, aircraft_state_history, horizontal_path, crossing_time, tmpx, tmpy);

   LOG4CPLUS_TRACE(m_logger,
                   Units::MetersLength(tmpx).value() << "," << Units::MetersLength(tmpy).value() << std::endl);
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
void IMUtils::CalculateMergePoint(const Units::Length x1,
                                  const Units::Length y1,
                                  const Units::Length x2,
                                  const Units::Length y2,
                                  const Units::Length x3,
                                  const Units::Length y3,
                                  Units::Length &x_merge,
                                  Units::Length &y_merge,
                                  const Units::Angle theta_merge) {

   LOG4CPLUS_TRACE(m_logger, "Pos of AC on final:  (" <<
         Units::MetersLength(x1) << "," << Units::MetersLength(y1) << ")");
   LOG4CPLUS_TRACE(m_logger, "Pos of intercepting AC:  (" <<
         Units::MetersLength(x3) << "," << Units::MetersLength(y3) << ")");
   LOG4CPLUS_TRACE(m_logger, "Achieve-by point:  (" <<
         Units::MetersLength(x2) << "," << Units::MetersLength(y2) << ")");

   Units::RadiansAngle theta_final(atan2(Units::MetersLength(y2-y1).value(),
         Units::MetersLength(x2-x1).value()));
   LOG4CPLUS_TRACE(m_logger, "thetaMerge = " <<
         Units::DegreesAngle(theta_merge) <<
         ", thetaFinal = " <<
         Units::DegreesAngle(theta_final) <<
         ", turn = " <<
         Units::DegreesAngle(theta_final - theta_merge));

   Units::Length x4(x3 + Units::NauticalMilesLength(50) * cos(theta_merge)),
         y4(y3 + Units::NauticalMilesLength(50) * sin(theta_merge));
   Units::Area denominator(((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4))), factor1((x1 * y2) - (y1 * x2)),
         factor2((x3 * y4) - (y3 * x4));

   LOG4CPLUS_TRACE(m_logger, "denominator = " <<
         Units::MetersArea(denominator) <<
         ", factor1 = " <<
         Units::MetersArea(factor1) <<
         ", factor2 = " <<
         Units::MetersArea(factor2));

   x_merge = (((x3 - x4) * factor1) - ((x1 - x2) * factor2)) / denominator;
   y_merge = (((y3 - y4) * factor1) - ((y1 - y2) * factor2)) / denominator;

   LOG4CPLUS_TRACE(m_logger, "Calculated merge point:  (" <<
         Units::MetersLength(x_merge) << "," << Units::MetersLength(y_merge) << ")");
}

/**
 * An algorithm that calculates an extrapolated time for a provided distance-to-go. Uses constant ground speed as an assumption
 * since only one state is provided.
 *
 * @param ownship_dtg: the distance-to-go on ownship's route
 * @param oldest_target_state: the target state to use for the extrapolation
 * @param ownship_horizontal_traj: ownship's horizontal path
 * @param extrapolated_target_time: returned time
 * @param projected_x, projected_y values on the horizontal path at time and dtg
 */
void IMUtils::CalculateTimeBasedExtrapolate(const Units::Length &ownship_dtg,
                                            const AircraftState &oldest_target_state,
                                            const std::vector<HorizontalPath> &ownship_horizontal_traj,
                                            Units::Time &extrapolated_target_time,
                                            Units::Length &projected_x,
                                            Units::Length &projected_y,
                                            Units::Length &projected_distance_to_go) {
   Units::Length target_dtg = Units::ZERO_LENGTH, targetx, targety;
   bool foundTargPos = ProjectTargetPosition(Units::FeetLength(oldest_target_state.m_x),
                                             Units::FeetLength(oldest_target_state.m_y),
                                             ownship_horizontal_traj,  // project onto _ownship's_ horizontal path
                                             targetx,
                                             targety,
                                             target_dtg);  // target_dtg needed on _ownship's_ horizontal path
   if (!foundTargPos) {
      string msg = "No target position could be found on ownship's horizontal path.";
      LOG4CPLUS_FATAL(m_logger, msg);
      throw logic_error(msg);
   }

   Units::Length delta_distance = ownship_dtg -
                                  target_dtg;  // there are valid scenarios in which target_dtg > ownship_dtg; negative is appropriate
   Units::Speed target_ground_speed = oldest_target_state.GetGroundSpeed();
   Units::Time delta_time = delta_distance / target_ground_speed;

   extrapolated_target_time =
         Units::SecondsTime(oldest_target_state.m_time) - delta_time; // time when target was at dtg_ownship
   AircraftState extrapstate, projstate(oldest_target_state);
   projstate.m_x = Units::FeetLength(targetx).value();
   projstate.m_y = Units::FeetLength(targety).value();
   extrapstate.Extrapolate(projstate, Units::SecondsTime(extrapolated_target_time).value());
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
void IMUtils::GetTimeBasedExtrapolateState(const AircraftState &current_ownship_state,
                                           const AircraftState &oldest_target_state,
                                           const std::vector<HorizontalPath> &ownship_horizontal_traj,
                                           const std::vector<HorizontalPath> &target_horizontal_traj,
                                           Units::Time &measured_spacing_interval,
                                           AircraftState &aircraft_state_to_return) {
   Units::Length ownship_dtg = Units::ZERO_LENGTH;
   Units::UnsignedAngle ownship_track = Units::ZERO_ANGLE;
   AircraftCalculations::LegacyGetPathLengthFromPosition(Units::FeetLength(current_ownship_state.m_x),
                                                         Units::FeetLength(current_ownship_state.m_y),
                                                         ownship_horizontal_traj,
                                                         ownship_dtg,
                                                         ownship_track);
   Units::Time extrapolated_target_time;
   Units::Length projX, projY, projDTG;
   CalculateTimeBasedExtrapolate(ownship_dtg,
                                 oldest_target_state,
                                 ownship_horizontal_traj,
                                 extrapolated_target_time,
                                 projX,
                                 projY,
                                 projDTG);
   Units::Time msi =
         Units::SecondsTime(current_ownship_state.m_time) - extrapolated_target_time; // the spacing interval in time

   // Now calculate an estimated target state that is on _target's_ horizontal path at "extrapolated_target_time"
   Units::Length x_out = Units::ZERO_LENGTH;
   Units::Length y_out = Units::ZERO_LENGTH;
   Units::UnsignedAngle track_out = Units::ZERO_ANGLE;
   int traj_index_out = -1;
   IMUtils::GetPositionFromPathLength(
         ownship_dtg,
         target_horizontal_traj, Units::Angle(),
         x_out,
         y_out,
         track_out,
         traj_index_out); // this will return even if ownship_dtg is off the back of the route
   aircraft_state_to_return.m_time = Units::SecondsTime(extrapolated_target_time).value();
   aircraft_state_to_return.m_id = oldest_target_state.m_id;
   aircraft_state_to_return.m_x = Units::FeetLength(x_out).value();
   aircraft_state_to_return.m_y = Units::FeetLength(y_out).value();
   aircraft_state_to_return.m_z = oldest_target_state.m_z;
   aircraft_state_to_return.m_xd = oldest_target_state.m_xd;
   aircraft_state_to_return.m_yd = oldest_target_state.m_yd;
   aircraft_state_to_return.m_zd = oldest_target_state.m_zd;
   aircraft_state_to_return.m_distance_to_go_meters = Units::MetersLength(ownship_dtg).value();

   measured_spacing_interval = msi;
}

/**
 * This intentionally shadows AircraftCalculations::getPosFromPathLength(). This method will handle the situation in
 * which the dist_in value is longer than the route.
 *
 * NOTE: only use this when you truly need this protection and not the error associated with being off the HorizontalPath.
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
                                        const Units::Angle ownship_true_heading,
                                        Units::Length &x_out,
                                        Units::Length &y_out,
                                        Units::UnsignedAngle &course_out,
                                        int &horizontal_path_index) {
   if (distance_to_go_in > Units::MetersLength(horizontal_path_in.back().m_path_length_cumulative_meters)) {
      course_out = ownship_true_heading;

      horizontal_path_index = 0;
      Units::UnsignedRadiansAngle crs(course_out);
      crs.normalize(); // make sure it's on the correct interval

      const Units::Length diff(distance_to_go_in - Units::MetersLength(horizontal_path_in.back().m_path_length_cumulative_meters));

      x_out = Units::MetersLength(horizontal_path_in.back().GetXPositionMeters()) + diff * cos(crs);
      y_out = Units::MetersLength(horizontal_path_in.back().GetYPositionMeters()) + diff * sin(crs);
   } else {
      AircraftCalculations::LegacyGetPositionFromPathLength(distance_to_go_in, horizontal_path_in, x_out, y_out,
                                                            course_out, horizontal_path_index);
   }
}

bool IMUtils::ProjectTargetPosition(const Units::Length x_target,
                                    const Units::Length y_target,
                                    const std::vector<HorizontalPath> &ownship_horizontal_path,
                                    Units::Length &x_projected,
                                    Units::Length &y_projected,
                                    Units::Length &dtg) {
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
         if (dtg - Units::MetersLength(ownship_horizontal_path.back().m_path_length_cumulative_meters) > startOfRouteTol) {
            LOG4CPLUS_ERROR(m_logger, "dtg - m_path_length_cumulative_meters: " << (Units::MetersLength(dtg).value() - ownship_horizontal_path.back().m_path_length_cumulative_meters));
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

bool IMUtils::GetCrossingTime(const Units::Length current_dtg,
                              const std::vector<AircraftState> &aircraft_state_history,
                              const std::vector<HorizontalPath> &horizontal_path,
                              Units::Time &crossing_time,
                              Units::Length &projected_x,
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
      return valid;   // can't do anything
   }

   // Case D returns immediately
   if (aircraft_state_history.size() == 1) {
      // handle case D:  one aircraftstates element
      Units::Length ownshipdtg = current_dtg;
      CalculateTimeBasedExtrapolate(ownshipdtg,
                                    aircraft_state_history.back(),
                                    horizontal_path,
                                    crossing_time,
                                    projected_x,
                                    projected_y,
                                    projDTG); // this will ensure that the state is projected onto ownship's route
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
   const bool b0 = IMUtils::ProjectTargetPosition(Units::FeetLength(aircraft_state_history[0].m_x),
                                                  Units::FeetLength(aircraft_state_history[0].m_y),
                                                  horizontal_path,
                                                  xProjected,
                                                  yProjected,
                                                  dtgProjected); // if this returns false, do not allow case A to happen
   startDist = dtgProjected;
   if (current_dtg > startDist && b0) {
      // case A:  currentdist is before aircraftstates[0], index will be -1
      IMUtils::CalculateTimeBasedExtrapolate(current_dtg, aircraft_state_history[0], horizontal_path, crossing_time, projected_x, projected_y,
                                             projDTG);
      return true;
   }

   /*
    * NOTE: Early returns are above this! Do not add early return logic below!
    *
    * Case E, D, A have already been handled and returned. Below logic is for B and C.
    */
   Units::Length xProjectedBefore, yProjectedBefore, xProjectedAfter, yProjectedAfter;
   bool b1(false), b2(false);
   for (auto loop = 1; loop < aircraft_state_history.size() && !found; ++loop)   // AAES-459
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
      }
      else {
         // possibly first time through loop -- do a fresh calculation
         b1 = IMUtils::ProjectTargetPosition(Units::FeetLength(aircraft_state_history[loop - 1].m_x),
                                             Units::FeetLength(aircraft_state_history[loop - 1].m_y),
                                             horizontal_path, xProjectedBefore, yProjectedBefore, prevdistance);
      }
      b2 = IMUtils::ProjectTargetPosition(Units::FeetLength(aircraft_state_history[loop].m_x),
                                          Units::FeetLength(aircraft_state_history[loop].m_y),
                                          horizontal_path, xProjectedAfter, yProjectedAfter, nextdistance);

      const bool next_distance_is_valid = b2 || nextdistance < Units::zero();
      if (current_dtg >= nextdistance &&
          current_dtg <= prevdistance &&
          b1 &&
          next_distance_is_valid) {
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
      crossing_time = Units::SecondsTime(
            (aircraft_state_history[index].m_time - aircraft_state_history[index - 1].m_time)
            / (nextdistance - prevdistance) * (current_dtg - prevdistance)
            + aircraft_state_history[index - 1].m_time);

      // Calculate a projected state associated with the crossingtime
      AircraftState crossingstate, beforestate(aircraft_state_history[index - 1]), afterstate(aircraft_state_history[index]);
      beforestate.m_x = Units::FeetLength(xProjectedBefore).value();
      beforestate.m_y = Units::FeetLength(yProjectedBefore).value();
      afterstate.m_x = Units::FeetLength(xProjectedAfter).value();
      afterstate.m_y = Units::FeetLength(yProjectedAfter).value();
      crossingstate.Interpolate(beforestate, afterstate, Units::SecondsTime(crossing_time).value());
      projected_x = Units::FeetLength(crossingstate.m_x); // for return
      projected_y = Units::FeetLength(crossingstate.m_y); // for return

      valid = true;
   } else {
      // Case C: The state vector does not encompass currentdist. Case A has already checked the zeroth state, so here
      // the decision must be to extrapolate from the last state to a crossing time in the future
      const bool b1 = IMUtils::ProjectTargetPosition(Units::FeetLength(aircraft_state_history.back().m_x),
                                                     Units::FeetLength(aircraft_state_history.back().m_y),
                                                     horizontal_path, xProjectedAfter, yProjectedAfter, nextdistance);
      AircraftState extrapstate;
      if (b1) {
         extrapstate = aircraft_state_history.back();
      } else {
         string msg = "Could not determine a state to extrapolate from.";
         LOG4CPLUS_FATAL(m_logger, msg);
         throw logic_error(msg);
      }
      IMUtils::CalculateTimeBasedExtrapolate(current_dtg, extrapstate, horizontal_path, crossing_time, projected_x, projected_y, projDTG);
      valid = true;
   }
   return valid;
}

/**
 * Calculate a state on ownship_horizontal_path at time extraptime. The returned state is the projection of target_state
 * onto ownship_horizontal_traj and then extrapolated to extraptime assuming constant ground speed.
 */
bool IMUtils::CalculateTargetStateAtTime(const AircraftState &target_state,
                                         const std::vector<HorizontalPath> &ownship_horizontal_traj,
                                         const Units::Time extrapolation_time,
                                         const Units::Angle ownship_true_heading,
                                         AircraftState &extrapolation_state) {

   AircraftState projstate(target_state), stateattime;
   Units::FeetLength projx, projy, projdtg;
   const bool b0 = IMUtils::ProjectTargetPosition(
         Units::FeetLength(target_state.m_x),
         Units::FeetLength(target_state.m_y),
         ownship_horizontal_traj,
         projx,
         projy,
         projdtg);
   if (!b0) {
      return false;
   }

   Units::Time dt = Units::SecondsTime(target_state.m_time) - extrapolation_time; // sign is important, pos means going backwards
   Units::Length delta_dist = target_state.GetGroundSpeed() * dt;
   Units::Length dtg = projdtg + delta_dist;

   int index;
   Units::UnsignedAngle crs;
   Units::Length x, y;
   GetPositionFromPathLength(dtg, ownship_horizontal_traj, ownship_true_heading, x, y, crs, index);
   extrapolation_state.m_id = target_state.m_id;
   extrapolation_state.m_time = Units::SecondsTime(extrapolation_time).value();
   extrapolation_state.m_x = Units::FeetLength(x).value();
   extrapolation_state.m_y = Units::FeetLength(y).value();
   extrapolation_state.m_z = target_state.m_z;
   extrapolation_state.m_xd = Units::FeetPerSecondSpeed(target_state.GetGroundSpeed() * Units::cos(crs)).value();
   extrapolation_state.m_yd = Units::FeetPerSecondSpeed(target_state.GetGroundSpeed() * Units::sin(crs)).value();

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
bool IMUtils::GetPathLengthFromPosition(const Units::Length x,
                                        const Units::Length y,
                                        const std::vector<HorizontalPath> &horizontal_path,
                                        Units::Length &distance_to_go,
                                        Units::Angle &track) {

   try {
      /*
       * Note: Do not attempt to replace this call with an instance of AlongPathDistanceCalculator. The caller's algorithm
       * will not support this. To use AlongPathDistanceCalculator, the entire algoirthm in this method's caller also
       * needs to be redesigned.
       */
      AircraftCalculations::LegacyGetPathLengthFromPosition(x, y, horizontal_path, distance_to_go, track);
      return true;
   }
   catch (logic_error e) {
      return false;
   }
}

Units::Speed IMUtils::FindLargestSpeedConstraint(const AircraftIntent &aircraft_intent) {
   Units::Speed max_ias_constraint = Units::ZERO_SPEED;
   const Units::KnotsSpeed max_ias_input(Waypoint::MAX_SPEED_CONSTRAINT);
   const struct AircraftIntent::RouteData route_info = aircraft_intent.GetFms();
   for (auto spdidx = 0; spdidx < MAX_NUM_WAYPOINTS - 1; ++spdidx) {
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

Units::Time IMUtils::InterpolateTimeAtDtg(const Units::Length target_dtg,
                                          const Units::Time time1,
                                          const Units::Time time2,
                                          const Units::Length dtg1,
                                          const Units::Length dtg2) {
   Units::Length mu = dtg2 - dtg1;
   return time1 + ((time2 - time1) * (target_dtg - dtg1)) / mu;
}

AircraftState IMUtils::
GetTargetStateOnOwnshipPathForDtg(const vector<AircraftState> &target_adsb_history,
                                  const std::vector<HorizontalPath> &ownship_horizontal_path,
                                  const Units::Length target_distance,
                                  const Units::Angle ownship_true_heading) {
   AircraftState result;
   AircraftState target_state_to_extrapolate_from;

   if (target_distance >= Units::MetersLength(target_adsb_history.front().m_distance_to_go_meters) &&
       Units::MetersLength(target_adsb_history.front().m_distance_to_go_meters).value() > -1) {
      target_state_to_extrapolate_from = target_adsb_history.front();
      IMUtils::CalculateTargetStateAtDistance(target_state_to_extrapolate_from, ownship_horizontal_path,
                                              target_distance, ownship_true_heading,
                                              result);
      return result;
   } else if (target_distance <= Units::MetersLength(target_adsb_history.back().m_distance_to_go_meters)) {
      target_state_to_extrapolate_from = target_adsb_history.back();
      IMUtils::CalculateTargetStateAtDistance(target_state_to_extrapolate_from, ownship_horizontal_path,
                                              target_distance, ownship_true_heading,
                                              result);
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
      AircraftState before = target_adsb_history[ix - 1];
      AircraftState after = target_adsb_history[ix];

      Units::Length beforex, beforey, beforedtg;
      Units::Length afterx, aftery, afterdtg;

      const bool b0 = IMUtils::ProjectTargetPosition(Units::FeetLength(before.m_x),
                                                     Units::FeetLength(before.m_y),
                                                     ownship_horizontal_path,
                                                     beforex,
                                                     beforey,
                                                     beforedtg);

      const bool b1 = IMUtils::ProjectTargetPosition(Units::FeetLength(after.m_x),
                                                     Units::FeetLength(after.m_y),
                                                     ownship_horizontal_path,
                                                     afterx,
                                                     aftery,
                                                     afterdtg);

      before.m_x = Units::FeetLength(beforex).value();
      before.m_y = Units::FeetLength(beforey).value();
      after.m_x = Units::FeetLength(afterx).value();
      after.m_y = Units::FeetLength(aftery).value();

      if (b0 && b1) {
         const Units::Time target_time_at_dtg = IMUtils::InterpolateTimeAtDtg(target_distance,
                                                                              Units::SecondsTime(before.m_time),
                                                                              Units::SecondsTime(after.m_time),
                                                                              Units::MetersLength(beforedtg),
                                                                              Units::MetersLength(afterdtg));

         result.Interpolate(before, after, Units::SecondsTime(target_time_at_dtg).value());
      } else {
         std::string msg = "Failure to project a state onto ownship's path";
         LOG4CPLUS_FATAL(m_logger, msg);
         throw std::logic_error(msg);
      }
   }
   return result;
}

/**
 * Calculate a state on ownship_horizontal_path at distance extrapolation_distance. The returned state is the projection of target_state
 * onto ownship_horizontal_traj and then extrapolated to extrapolation_distance assuming constant ground speed.
 */
void IMUtils::CalculateTargetStateAtDistance(const AircraftState &target_state,
                                             const std::vector<HorizontalPath> &ownship_horizontal_traj,
                                             const Units::Length extrapolation_distance,
                                             const Units::Angle ownship_true_heading,
                                             AircraftState &extrapstate) {

   AircraftState projstate(target_state);
   AircraftState stateattime;
   Units::FeetLength projx, projy, projdtg;
   const bool b0 = IMUtils::ProjectTargetPosition(Units::FeetLength(target_state.m_x),
                                                  Units::FeetLength(target_state.m_y),
                                                  ownship_horizontal_traj,
                                                  projx,
                                                  projy,
                                                  projdtg);
   if (!b0) {
      LOG4CPLUS_FATAL(m_logger, "Unable to extrapoloate a target state to time "
            << Units::MetersLength(extrapolation_distance).value() << ". Cannot proceed.");
      target_state.DumpParms("target_state to extrapolate from:");

      throw logic_error("Cannot proceed.");
   }

   Units::Length delta_dist = extrapolation_distance - Units::MetersLength(target_state.m_distance_to_go_meters);
   Units::Length dtg = projdtg + delta_dist;

   Units::Time dt = delta_dist / target_state.GetGroundSpeed();
   Units::Time extrapolation_state_time = Units::SecondsTime(target_state.m_time) + dt;

   int index;
   Units::UnsignedAngle crs;
   Units::Length x, y;
   GetPositionFromPathLength(dtg, ownship_horizontal_traj, ownship_true_heading, x, y,
                             crs, index);
   extrapstate.m_id = target_state.m_id;
   extrapstate.m_time = Units::SecondsTime(extrapolation_state_time).value();
   extrapstate.m_x = Units::FeetLength(x).value();
   extrapstate.m_y = Units::FeetLength(y).value();
   extrapstate.m_z = target_state.m_z;
   extrapstate.m_xd = Units::FeetPerSecondSpeed(target_state.GetGroundSpeed() * Units::cos(crs)).value();
   extrapstate.m_yd = Units::FeetPerSecondSpeed(target_state.GetGroundSpeed() * Units::sin(crs)).value();
}

AircraftState IMUtils::GetProjectedTargetState(const std::vector<AircraftState> &target_state_history,
                                               const std::vector<HorizontalPath> &ownship_horizontal_traj,
                                               const Units::Time target_time,
                                               const Units::Angle ownship_true_heading,
                                               bool &target_state_is_valid) {
   AircraftState target_state;
   AircraftState extrapolateFromThis;

   bool found = false;
   target_state_is_valid = false;

   if (target_time < Units::SecondsTime(target_state_history.front().m_time)) {
      found = true;
      target_state_is_valid = true;
      extrapolateFromThis = target_state_history.front();
   } else if (target_time == Units::SecondsTime(target_state_history.front().m_time)) {
      found = true;
      target_state_is_valid = true;
      extrapolateFromThis = target_state_history.front();
   } else if (target_time == Units::SecondsTime(target_state_history.back().m_time)) {
      found = true;
      target_state_is_valid = true;
      extrapolateFromThis = target_state_history.back();
   } else if (target_time > Units::SecondsTime(target_state_history.back().m_time)) {
      found = true;
      extrapolateFromThis = target_state_history.back();
   }

   int ix = 0;
   while (ix < target_state_history.size() && !found) {
      if (target_time < Units::SecondsTime(target_state_history[ix].m_time)) {
         ix--;
      }
      else if (ix < (target_state_history.size() - 1) &&
               target_time > Units::SecondsTime(target_state_history[ix + 1].m_time)) {
         ix++;
      }
      else if (target_time > Units::SecondsTime(target_state_history[ix].m_time)) {
         found = true;
         extrapolateFromThis = target_state_history[ix];
         ix++;
      }
   }

   if (ix != 0 && ix < target_state_history.size()) {
      AircraftState before(target_state_history[ix - 1]), after(target_state_history[ix]);
      Units::Length beforex, beforey, beforedtg;
      Units::Length afterx, aftery, afterdtg;
      const bool b0 = IMUtils::ProjectTargetPosition(
            Units::FeetLength(before.m_x),
            Units::FeetLength(before.m_y),
            ownship_horizontal_traj,
            beforex,
            beforey,
            beforedtg);
      const bool b1 = IMUtils::ProjectTargetPosition(
            Units::FeetLength(after.m_x),
            Units::FeetLength(after.m_y),
            ownship_horizontal_traj,
            afterx,
            aftery,
            afterdtg);
      before.m_x = Units::FeetLength(beforex).value();
      before.m_y = Units::FeetLength(beforey).value();
      after.m_x = Units::FeetLength(afterx).value();
      after.m_y = Units::FeetLength(aftery).value();

      if (b0 && b1) {
         target_state.Interpolate(before, after, Units::SecondsTime(target_time).value());
         target_state_is_valid = true;
      } else {
         // This probably means that the bounding states cannot be projected onto the
         // horizontal paht of ownship. This is mathematically valid. The method must return false.
         target_state_is_valid = false;
      }
   } else {
      // Extrapolate along ownship path. This can fail if no possible state on the horizontal_traj. See AAES-798 & test
      bool is_extrapolation_valid = IMUtils::CalculateTargetStateAtTime(extrapolateFromThis,
                                                                        ownship_horizontal_traj,
                                                                        target_time, ownship_true_heading,
                                                                        target_state);

      target_state_is_valid = target_state_is_valid && is_extrapolation_valid;
   }

   return target_state;
}

Units::SignedAngle IMUtils::CalculateTrackAngle(
      const std::list<Units::Angle> angle_history) {

   // Average the track angles
   // The tricky part is avoiding a wrap effect.
   // Compute total using both signed and unsigned,
   // But discard elements in danger of wrapping
   int signed_warning_count(0), unsigned_warning_count(0);
   Units::RadiansAngle total_signed(0), total_unsigned(0);
   for (auto itr = angle_history.begin(); itr != angle_history.end(); ++itr) {
      Units::SignedDegreesAngle x_signed = *itr;
      Units::UnsignedDegreesAngle x_unsigned = *itr;
      if (x_signed < Units::DegreesAngle(45) &&
            x_signed > Units::DegreesAngle(-45)) {
         unsigned_warning_count++;
      }
      else {
         total_unsigned += x_unsigned;
      }
      if (x_unsigned < Units::DegreesAngle(225) &&
            x_unsigned > Units::DegreesAngle(135)) {
         signed_warning_count++;
      }
      else {
         total_signed += x_signed;
      }
   }

   Units::SignedAngle result;
   if (signed_warning_count < unsigned_warning_count) {
      result = total_signed / (angle_history.size() - signed_warning_count);
   }
   else {
      result = total_unsigned / (angle_history.size() - unsigned_warning_count);
   }

   // if both totals have discards, log an error
   if (signed_warning_count > 0 && unsigned_warning_count > 0) {
      LOG4CPLUS_ERROR(m_logger, "Track angle history has " <<
            signed_warning_count << " angles near 180 and " <<
            unsigned_warning_count << " near 0.  This is not normal.");
   }
   return result;
}

