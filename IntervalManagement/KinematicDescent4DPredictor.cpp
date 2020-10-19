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

#include <public/Waypoint.h>
#include "imalgs/KinematicDescent4DPredictor.h"
#include <cmath>

using namespace std;

log4cplus::Logger KinematicDescent4DPredictor::m_logger = log4cplus::Logger::getInstance(
      LOG4CPLUS_TEXT("KinematicDescent4DPredictor"));

const Units::Length KinematicDescent4DPredictor::m_vertical_tolerance_distance = Units::FeetLength(400);


KinematicDescent4DPredictor::KinematicDescent4DPredictor()
      : m_kinematic_descent_type(CONSTRAINED),
        m_altitude_at_end_of_route(Units::zero()),
        m_deceleration_mps(0.5 * KNOTS_TO_METERS_PER_SECOND),
        m_deceleration_level_mps(0.75 * KNOTS_TO_METERS_PER_SECOND),
        m_deceleration_fpa_mps(0.3 * KNOTS_TO_METERS_PER_SECOND),
        m_const_gamma_cas_term_rad(2.9 * DEGREES_TO_RADIAN),
        m_const_gamma_cas_er_rad(3.1 * DEGREES_TO_RADIAN),
        m_const_gamma_mach_rad(4.0 * DEGREES_TO_RADIAN),
        m_prediction_too_low(false),
        m_prediction_too_high(false) {
}

KinematicDescent4DPredictor::~KinematicDescent4DPredictor() = default;

void KinematicDescent4DPredictor::SetMembers(const KineticDescent4DPredictor &kinetic_descent_4d_predictor) {
   VerticalPredictor::SetMembers(kinetic_descent_4d_predictor);

   m_kinematic_descent_type = KinematicDescent4DPredictor::KinematicDescentType::CONSTRAINED;
   LOG4CPLUS_TRACE(m_logger, "kinematic descent type to set to constrained" << std::endl);
}

void KinematicDescent4DPredictor::SetMembers(const double &mach_descent,
                                             const Units::Speed ias_descent,
                                             const Units::Length cruise_altitude,
                                             const Units::Length transition_altitude) {
   m_transition_mach = mach_descent;
   m_transition_ias = ias_descent;
   m_cruise_altitude_msl = cruise_altitude;

   if (IsTransitionMachValid()) {
      m_transition_altitude_msl = transition_altitude;
   } else {
      m_transition_altitude_msl = Units::Infinity();
   }
}

void KinematicDescent4DPredictor::BuildVerticalPrediction(vector<HorizontalPath> &horizontal_path,
                                                          vector<PrecalcWaypoint> &precalc_waypoints,
                                                          const WeatherPrediction &weather_prediction,
                                                          const Units::Length &start_altitude,
                                                          const Units::Length &aircraft_distance_to_go) {
   m_start_altitude_msl = start_altitude;
   m_prediction_too_low = false;
   m_prediction_too_high = false;
   HorizontalPath start_pos(horizontal_path.back());
   LOG4CPLUS_DEBUG(m_logger, "Building vertical prediction from ("
         <<
         start_pos.GetXPositionMeters()
         << ","
         << start_pos.GetYPositionMeters()
         <<
         "), alt="
         << m_start_altitude_msl
         << " dtg: "
         << Units::MetersLength(aircraft_distance_to_go).value());
   m_course_calculator = DirectionOfFlightCourseCalculator(horizontal_path,
                                                           TrajectoryIndexProgressionDirection::UNDEFINED);

   ConstrainedVerticalPath(horizontal_path, precalc_waypoints, m_deceleration_mps,
                           m_const_gamma_cas_term_rad, m_const_gamma_cas_er_rad, m_const_gamma_mach_rad,
                           weather_prediction, aircraft_distance_to_go);

   TrimDuplicatesFromVerticalPath();

   m_current_trajectory_index = m_vertical_path.along_path_distance_m.size() - 1;
}

void KinematicDescent4DPredictor::ConstrainedVerticalPath(vector<HorizontalPath> &horizontal_path,
                                                          vector<PrecalcWaypoint> &precalc_waypoints,
                                                          double deceleration,
                                                          double const_gamma_cas_term,
                                                          double const_gamma_cas_er,
                                                          double const_gamma_mach,
                                                          const WeatherPrediction &weather_prediction,
                                                          const Units::Length &aircraft_distance_to_go) {
   VerticalPath trajTemp;
   trajTemp.mass_kg.push_back(-1.0);
   trajTemp.time_to_go_sec.push_back(Units::SecondsTime(m_descent_start_time).value());
   trajTemp.along_path_distance_m.push_back(0);
   trajTemp.altitude_m.push_back(Units::MetersLength(m_altitude_at_end_of_route).value());
   trajTemp.cas_mps.push_back(Units::MetersPerSecondSpeed(m_ias_at_end_of_route).value());
   trajTemp.altitude_rate_mps.push_back(0);
   trajTemp.true_airspeed.push_back(Units::ZERO_SPEED);
   trajTemp.tas_rate_mps.push_back(0);
   trajTemp.theta_radians.push_back(0);

   Units::Speed vwpara;
   Units::Speed vwperp;
   Units::Speed Vwx, Vwy;
   Units::UnsignedAngle course = m_course_calculator.GetCourseAtPathEnd();
   ComputeWindCoefficients(m_altitude_at_end_of_route, Units::RadiansAngle(course), weather_prediction, vwpara, vwperp,
                           Vwx, Vwy);

   Units::Speed initialgs =
         sqrt(Units::sqr(
               weather_prediction.getAtmosphere()->CAS2TAS(m_ias_at_end_of_route, m_altitude_at_end_of_route)) -
              Units::sqr(vwperp)) + vwpara;

   trajTemp.gs_mps.push_back(Units::MetersPerSecondSpeed(initialgs).value());
   trajTemp.wind_velocity_east.push_back(Vwx);
   trajTemp.wind_velocity_north.push_back(Vwy);

   trajTemp.algorithm_type.push_back(VerticalPath::PredictionAlgorithmType::UNDETERMINED);

   m_vertical_path = trajTemp;

   VerticalPath last_state = m_vertical_path;
   VerticalPath last_waypoint_state = m_vertical_path;
   VerticalPath low_tolerance_state; // earliesr waypoint in prediction that can do FPA to current aircraft position
   bool low_tolerance_state_found = false;

   double FPA;
   double alt1 = min(Units::MetersLength(m_transition_altitude_msl).value(),
                     Units::MetersLength(m_cruise_altitude_msl).value());

   if (aircraft_distance_to_go < Units::NauticalMilesLength(1)) {
      LOG4CPLUS_WARN(m_logger,
                     "Attempting to perform constrained vertical path with less than 1 nautical mile to go.  Calculating level path, instead.");
      m_vertical_path = LevelVerticalPath(m_vertical_path,
                                          precalc_waypoints[precalc_waypoints.size() -
                                                            1].m_precalc_constraints.constraint_dist,
                                          horizontal_path, weather_prediction, aircraft_distance_to_go);
      return;
   }

   LOG4CPLUS_TRACE(m_logger, "begin constrainedVerticalPath, start/cruise/transition altitude: "
         << m_start_altitude_msl.value() << "/" << Units::MetersLength(m_cruise_altitude_msl).value()
         << "/" << Units::MetersLength(m_transition_altitude_msl).value());
   // CAS segment
   // Need to plan to end of segment, at least.  Cannot stop when reaching start altitude.
   // Start altitude can cause a switch in type of descent part way along a segment.
   // The test for start altitude is placed after the segment is complete: see AAES-756.
   while (m_vertical_path.altitude_m.back() < alt1) {
      LOG4CPLUS_TRACE(m_logger, " before constantCASVerticalPath, x/h/v is: "
              << m_vertical_path.along_path_distance_m.back() << "/" << m_vertical_path.altitude_m.back() << "/"
              << m_vertical_path.cas_mps.back());
      if (m_vertical_path.along_path_distance_m.back() > horizontal_path.back().m_path_length_cumulative_meters) {
         LOG4CPLUS_WARN(m_logger,
                        "KinematicDescent4DPredictor cannot create a trajectory that reaches "
                              << alt1 << "m in the distance required.");
         break;
      }

      if (m_vertical_path.altitude_m.back() < 10000 * FEET_TO_METERS) {
         m_vertical_path = ConstantCasVerticalPath(m_vertical_path, alt1, horizontal_path, precalc_waypoints,
                                                   const_gamma_cas_term, weather_prediction, aircraft_distance_to_go);
      } else {
         m_vertical_path = ConstantCasVerticalPath(m_vertical_path, alt1, horizontal_path, precalc_waypoints,
                                                   const_gamma_cas_er, weather_prediction, aircraft_distance_to_go);
      }
      LOG4CPLUS_TRACE(m_logger, " after constantCASVerticalPath, x/h/v is: "
              << m_vertical_path.along_path_distance_m.back() << "/" << m_vertical_path.altitude_m.back() << "/"
              << m_vertical_path.cas_mps.back() << " active_flag is: "
            << static_cast<int>(m_precalculated_constraints.active_flag));
      // special case if on first segment
      if (Units::MetersLength(aircraft_distance_to_go).value() <=
          precalc_waypoints[0].m_precalc_constraints.constraint_dist) {
         low_tolerance_state = last_waypoint_state;
         low_tolerance_state_found = true;
      }

      if (m_prediction_too_low || m_prediction_too_high) {
         break;
      }

      if (m_precalculated_constraints.violation_flag) {
         if (m_precalculated_constraints.active_flag == ActiveFlagType::SEG_END_LOW_ALT) {
            FPA = atan2((m_precalculated_constraints.constraint_altLow - last_waypoint_state.altitude_m.back()),
                        (m_precalculated_constraints.constraint_dist - last_waypoint_state.along_path_distance_m.back()));
            m_vertical_path = ConstantFpaDecelerationVerticalPath(last_waypoint_state,
                                                                  m_precalculated_constraints.constraint_altLow,
                                                                  m_precalculated_constraints.constraint_speedHi, FPA,
                                                                  horizontal_path,
                                                                  precalc_waypoints,
                                                                  weather_prediction,
                                                                  aircraft_distance_to_go);

            LOG4CPLUS_TRACE(m_logger, "   after constantFPADecelVerticalPath, x/h/v is: "
                    << m_vertical_path.along_path_distance_m.back() << "/" << m_vertical_path.altitude_m.back() << "/"
                  << m_vertical_path.cas_mps.back());

            if (m_prediction_too_low || m_prediction_too_high) {
               break;
            }

            m_vertical_path = ConstantGeometricFpaVerticalPath(m_vertical_path,
                                                               m_precalculated_constraints.constraint_altLow,
                                                               FPA, horizontal_path, precalc_waypoints,
                                                               weather_prediction,
                                                               aircraft_distance_to_go);

            LOG4CPLUS_TRACE(m_logger, "   after constantGeoFPAVerticalPath, x/h/v is: "
                    << m_vertical_path.along_path_distance_m.back() << "/" << m_vertical_path.altitude_m.back() << "/"
                  << m_vertical_path.cas_mps.back());


         } else if (m_precalculated_constraints.active_flag == ActiveFlagType::AT_ALT_ON_SPEED) {
            FPA = atan2(m_precalculated_constraints.constraint_altHi - last_state.altitude_m.back(),
                        m_precalculated_constraints.constraint_dist - last_state.along_path_distance_m.back());

            if (FPA > 0.10 * PI / 180.0) {
               m_vertical_path = ConstantGeometricFpaVerticalPath(last_state,
                                                                  m_precalculated_constraints.constraint_altHi, FPA,
                                                                  horizontal_path,
                                                                  precalc_waypoints, weather_prediction,
                                                                  aircraft_distance_to_go);
               LOG4CPLUS_TRACE(m_logger, "   after constantGeoFPAVerticalPath, x/h/v is: "
                       << m_vertical_path.along_path_distance_m.back() << "/" << m_vertical_path.altitude_m.back() << "/"
                     << m_vertical_path.cas_mps.back());

               if (m_prediction_too_low || m_prediction_too_high) {
                  break;
               }
            }

            m_vertical_path = LevelVerticalPath(m_vertical_path,
                                                m_precalculated_constraints.constraint_dist, horizontal_path,
                                                weather_prediction,
                                                aircraft_distance_to_go);
            LOG4CPLUS_TRACE(m_logger, "   after levelVerticalPath, x/h/v is: "
                    << m_vertical_path.along_path_distance_m.back() << "/" << m_vertical_path.altitude_m.back() << "/"
                  << m_vertical_path.cas_mps.back());

         } else if (m_precalculated_constraints.active_flag == ActiveFlagType::BELOW_ALT_SLOW) {

            if (m_precalculated_constraints.index < precalc_waypoints.size()) {
               m_vertical_path = ConstantDecelerationVerticalPath(m_vertical_path,
                                                                  Units::MetersLength(
                                                                        m_precalculated_constraints.constraint_dist),
                                                                  Units::MetersLength(
                                                                        m_precalculated_constraints.constraint_altHi),
                                                                  deceleration,
                                                                  m_precalculated_constraints.constraint_speedHi,
                                                                  horizontal_path, weather_prediction,
                                                                  aircraft_distance_to_go);
               LOG4CPLUS_TRACE(m_logger, "   after constantDecelVerticalPath, x/h/v is: "
                       << m_vertical_path.along_path_distance_m.back() << "/" << m_vertical_path.altitude_m.back() << "/"
                     << m_vertical_path.cas_mps.back());
            }

            if (m_prediction_too_low || m_prediction_too_high) {
               break;
            }

            if ((m_vertical_path.along_path_distance_m.back() > m_precalculated_constraints.constraint_dist) &&
                (m_vertical_path.altitude_m.back() < m_precalculated_constraints.constraint_altLow)) {
               // If idle-descent acceleration is below low altitude constraint-
               // redo with with a constant FPA deceleration trajectory.
               FPA = atan2((m_precalculated_constraints.constraint_altLow - last_state.altitude_m.back()),
                           (m_precalculated_constraints.constraint_dist - last_state.along_path_distance_m.back()));
               Units::DegreesAngle uFPA = Units::RadiansAngle(FPA);
               if (FPA > Units::RadiansAngle(m_descent_angle_max).value())
                  LOG4CPLUS_WARN(m_logger, "prediction FPA is " << uFPA.value() << " which is greater than "
                                                                << m_descent_angle_warning.value());
               if (uFPA < m_descent_angle_max) {
                  m_vertical_path = ConstantFpaDecelerationVerticalPath(last_state,
                                                                        m_precalculated_constraints.constraint_altLow,
                                                                        m_precalculated_constraints.constraint_speedHi,
                                                                        FPA, horizontal_path, precalc_waypoints,
                                                                        weather_prediction,
                                                                        aircraft_distance_to_go);
                  LOG4CPLUS_TRACE(m_logger, "   after constantFPADecelVerticalPath, x/h/v is: "
                          << m_vertical_path.along_path_distance_m.back() << "/" << m_vertical_path.altitude_m.back() << "/"
                        << m_vertical_path.cas_mps.back());
               }
            }
         } else if (m_precalculated_constraints.active_flag == ActiveFlagType::AT_ALT_SLOW) {
            if (m_precalculated_constraints.index < precalc_waypoints.size()) {
               m_vertical_path = LevelDecelerationVerticalPath(m_vertical_path,
                                                               Units::MetersLength(
                                                                     m_precalculated_constraints.constraint_dist),
                                                               m_deceleration_level_mps,
                                                               m_precalculated_constraints.constraint_speedHi,
                                                               horizontal_path,
                                                               weather_prediction,
                                                               aircraft_distance_to_go);
               LOG4CPLUS_TRACE(m_logger, "   after levelDecelVerticalPath, x/h/v is: "
                       << m_vertical_path.along_path_distance_m.back() << "/" << m_vertical_path.altitude_m.back() << "/"
                     << m_vertical_path.cas_mps.back());
            }
         }

         if (m_prediction_too_low || m_prediction_too_high) {
            LOG4CPLUS_TRACE(m_logger, "Constrained Vertical Path is out of tolerance at aircraft position; alt: "
                  << Units::MetersLength(m_start_altitude_msl).value() << " prediction alt: "
                  << m_vertical_path.altitude_m.back());
            break;
         }
      } // end if violation_flag


      last_state = m_vertical_path;
      if ((aircraft_distance_to_go != Units::MetersLength(Units::infinity())) && !low_tolerance_state_found) {
         if ((m_vertical_path.altitude_m.back() > Units::MetersLength(m_start_altitude_msl).value())
             || (m_vertical_path.along_path_distance_m.back() > Units::MetersLength(aircraft_distance_to_go).value())) {
            low_tolerance_state_found = true;
            low_tolerance_state = last_waypoint_state;
         }
      }
      if (m_vertical_path.along_path_distance_m.back() > m_precalculated_constraints.constraint_dist) {
         if (last_waypoint_state == m_vertical_path) {
            LOG4CPLUS_ERROR(m_logger, "Infinite loop detected");
            return;
         }

         if (aircraft_distance_to_go != Units::MetersLength(Units::infinity())) {
            // find if FPA from end of prediction to current position is valid
            double distance_to_current_position =
                  Units::MetersLength(aircraft_distance_to_go).value() - m_vertical_path.along_path_distance_m.back();
            if (!low_tolerance_state_found) {
               //if (distance_to_current_position <= 0) {
               //   low_tolerance_state_found = true;
               //    low_tolerance_state = last_waypoint_state;
               //} else {
               double alt_per_dist = (Units::MetersLength(m_start_altitude_msl).value() - m_vertical_path.altitude_m.back())
                                     / distance_to_current_position;
               for (unsigned int loop = m_precalculated_constraints.index; loop < precalc_waypoints.size(); loop++) {
                  // end of prediction should be on next segment
                  if (m_vertical_path.along_path_distance_m.back() > precalc_waypoints[loop].m_precalc_constraints.constraint_dist) {
                     continue;
                  }
                  if (precalc_waypoints[loop].m_precalc_constraints.constraint_dist >
                      Units::MetersLength(aircraft_distance_to_go).value()) {
                     low_tolerance_state_found = true;
                     low_tolerance_state = m_vertical_path;
                     break;
                  }

                  double x_dist =
                        precalc_waypoints[loop].m_precalc_constraints.constraint_dist - m_vertical_path.along_path_distance_m.back();
                  double y_alt = m_vertical_path.altitude_m.back() + alt_per_dist * x_dist;

                  if (y_alt > (precalc_waypoints[loop].m_precalc_constraints.constraint_altHi + 10)
                      || y_alt < (precalc_waypoints[loop].m_precalc_constraints.constraint_altLow - 10)) {
                     break;
                  }
               }
               //}
            }
         }

         LOG4CPLUS_TRACE(m_logger, "last_waypoint_state reset to " << m_precalculated_constraints.index);
         last_waypoint_state = m_vertical_path;

         if (m_vertical_path.altitude_m.back() > m_start_altitude_msl.value()) {
            break;
         }
      } // END if x < constraint_dist


   } // END while h < alt1

   // Constant Mach segment
   last_state = m_vertical_path;


   if (m_prediction_too_low)
      LOG4CPLUS_DEBUG(m_logger, "Constrained prediction too low.  Replanning with FPA.");
   if (m_prediction_too_high)
      LOG4CPLUS_DEBUG(m_logger, "Constrained prediction too high.  Replanning with FPA.");
   if (m_prediction_too_low || m_prediction_too_high) {
      if (low_tolerance_state_found) {
         m_vertical_path = ConstantFpaToCurrentPositionVerticalPath(low_tolerance_state, horizontal_path,
                                                                    precalc_waypoints, const_gamma_mach,
                                                                    weather_prediction, aircraft_distance_to_go);
         return;
      } else {
         // This should never happen. Wore case is vertical path to waypoint beginning this segment.
         LOG4CPLUS_WARN(m_logger,
                        "low_tolerance_state vertical prediction not found for replan in ConstrainedVerticalPath");
      }
   }

   while ((m_vertical_path.altitude_m.back() < m_start_altitude_msl.value()) &&
          (m_vertical_path.along_path_distance_m.back() <= horizontal_path.back().m_path_length_cumulative_meters)) {
      m_vertical_path = ConstantMachVerticalPath(last_state, m_start_altitude_msl.value(), horizontal_path,
                                                 precalc_waypoints, const_gamma_mach,
                                                 weather_prediction, aircraft_distance_to_go);

      if (m_prediction_too_low || m_prediction_too_high) {
         break;
      }

      if (m_precalculated_constraints.violation_flag) {
         if (m_precalculated_constraints.active_flag == ActiveFlagType::SEG_END_LOW_ALT) {
            FPA = atan2(m_precalculated_constraints.constraint_altLow - last_state.altitude_m.back(),
                        m_precalculated_constraints.constraint_dist - last_state.along_path_distance_m.back());

            // if unable to reach low altitude constraint due to excessive FPA, then continue constantMachVerticalPath
            if (FPA > 10 * PI / 180.0) {
               LOG4CPLUS_WARN(m_logger,
                              "constrainedVerticalPath prediction in mach segment cannot reach low altitude constraint");
            } else if (FPA > 0.10 * PI / 180.0) {
               m_vertical_path = ConstantGeometricFpaVerticalPath(last_state,
                                                                  m_precalculated_constraints.constraint_altLow, FPA,
                                                                  horizontal_path,
                                                                  precalc_waypoints, weather_prediction,
                                                                  aircraft_distance_to_go);
            } else {
               m_vertical_path = LevelVerticalPath(last_state, m_precalculated_constraints.constraint_dist,
                                                   horizontal_path, weather_prediction,
                                                   aircraft_distance_to_go);
            }

            if (m_prediction_too_low || m_prediction_too_high) {
               break;
            }

            m_vertical_path = LevelVerticalPath(m_vertical_path, m_precalculated_constraints.constraint_dist,
                                                horizontal_path, weather_prediction,
                                                aircraft_distance_to_go);
         } else if (m_precalculated_constraints.active_flag == ActiveFlagType::AT_ALT_ON_SPEED) {
            FPA = atan2(m_precalculated_constraints.constraint_altHi - last_state.altitude_m.back(),
                        m_precalculated_constraints.constraint_dist - last_state.along_path_distance_m.back());

            if (FPA > 0.10 * PI / 180.0) {
               m_vertical_path = ConstantGeometricFpaVerticalPath(last_state,
                                                                  m_precalculated_constraints.constraint_altHi, FPA,
                                                                  horizontal_path,
                                                                  precalc_waypoints, weather_prediction,
                                                                  aircraft_distance_to_go);
            } else {
               m_vertical_path = LevelVerticalPath(last_state, m_precalculated_constraints.constraint_dist,
                                                   horizontal_path, weather_prediction,
                                                   aircraft_distance_to_go);
            }
            m_vertical_path = LevelVerticalPath(m_vertical_path, m_precalculated_constraints.constraint_dist,
                                                horizontal_path, weather_prediction,
                                                aircraft_distance_to_go);
         }
      }
      last_state = m_vertical_path;
   }

   if (m_prediction_too_low)
      LOG4CPLUS_DEBUG(m_logger, "Constrained prediction too low.  Replanning with FPA.");
   if (m_prediction_too_high)
      LOG4CPLUS_DEBUG(m_logger, "Constrained prediction too high.  Replanning with FPA.");
   if (m_prediction_too_low || m_prediction_too_high) {
      if (low_tolerance_state_found) {
         m_vertical_path = ConstantFpaToCurrentPositionVerticalPath(low_tolerance_state, horizontal_path,
                                                                    precalc_waypoints, const_gamma_mach,
                                                                    weather_prediction, aircraft_distance_to_go);
         return;
      } else {
         // This should never happen. Wore case is vertical path to waypoint beginning this segment.
         LOG4CPLUS_WARN(m_logger,
                        "low_tolerance_state vertical prediction not found for replan in ConstrainedVerticalPath");
      }
   }

   // Level segment to end of prediction
   if (m_vertical_path.altitude_m.back() < Units::MetersLength(m_transition_altitude_msl).value()) {
      while (m_vertical_path.along_path_distance_m.back() < horizontal_path.back().m_path_length_cumulative_meters) {
         m_precalculated_constraints = FindActiveConstraint(m_vertical_path.along_path_distance_m.back(), precalc_waypoints);
         m_precalculated_constraints = CheckActiveConstraint(m_vertical_path.along_path_distance_m.back(),
                                                             m_vertical_path.altitude_m.back(),
                                                             m_vertical_path.cas_mps.back(),
                                                             m_precalculated_constraints,
                                                             Units::MetersLength(m_transition_altitude_msl).value());
         if (m_precalculated_constraints.violation_flag &&
             (m_precalculated_constraints.active_flag == ActiveFlagType::BELOW_ALT_SLOW ||
              m_precalculated_constraints.active_flag == ActiveFlagType::AT_ALT_SLOW)) {
            m_vertical_path = LevelDecelerationVerticalPath(m_vertical_path,
                                                            m_deceleration_level_mps,
                                                            m_precalculated_constraints.constraint_speedHi,
                                                            horizontal_path,
                                                            weather_prediction,
                                                            aircraft_distance_to_go);
         }
         m_vertical_path = LevelVerticalPath(m_vertical_path, horizontal_path.back().m_path_length_cumulative_meters,
                                             horizontal_path, weather_prediction,
                                             aircraft_distance_to_go);
      }
   }
   m_vertical_path = LevelVerticalPath(m_vertical_path, horizontal_path.back().m_path_length_cumulative_meters,
                                       horizontal_path, weather_prediction,
                                       aircraft_distance_to_go);

   // Do not need to check prediction too high or too low for level (start altitude)

}

VerticalPath KinematicDescent4DPredictor::ConstantCasVerticalPath(VerticalPath vertical_path,
                                                                  double altitude_at_end,
                                                                  vector<HorizontalPath> &horizontal_path,
                                                                  vector<PrecalcWaypoint> &precalc_waypoints,
                                                                  double CAS_gamma,
                                                                  const WeatherPrediction &weather_prediction,
                                                                  const Units::Length &aircraft_distance_to_go) {
   VerticalPath result;

   double delta_t = -0.5;
   double dist = vertical_path.along_path_distance_m[vertical_path.along_path_distance_m.size() - 1];
   double v_cas = vertical_path.cas_mps[vertical_path.cas_mps.size() - 1];
   double h = vertical_path.altitude_m[vertical_path.altitude_m.size() - 1];

   bool bracket_found = false;

   result = vertical_path;

   m_precalculated_constraints = FindActiveConstraint(dist, precalc_waypoints);
   m_precalculated_constraints = CheckActiveConstraint(dist, h, v_cas, m_precalculated_constraints,
                                                       Units::MetersLength(m_transition_altitude_msl).value());

   while (h < altitude_at_end && m_precalculated_constraints.active_flag <= ActiveFlagType::BELOW_ALT_ON_SPEED) {
      double v_tas = Units::MetersPerSecondSpeed(
            weather_prediction.getAtmosphere()->CAS2TAS(Units::MetersPerSecondSpeed(v_cas),
                                                        Units::MetersLength(h))).value();

      double esf;
      double dh_dt;
      double dv_dh;
      double dv_dt;
      double h_new;
      double dist_new;
      double v_tas_new;
      double v_cas_new;
      double curr_time;

      Units::UnsignedRadiansAngle course;
      m_course_calculator.CalculateCourseAtAlongPathDistance(Units::MetersLength(dist), course);

      Units::Speed vwpara;
      Units::Speed vwperp;
      Units::Speed Vwx, Vwy;

      ComputeWindCoefficients(Units::MetersLength(h), Units::RadiansAngle(course), weather_prediction, vwpara, vwperp,
                              Vwx, Vwy);

      esf = CalculateEsfUsingConstantCAS(v_tas, h,
                                         weather_prediction.getAtmosphere()->GetTemperature(Units::MetersLength(h)));

      // climb/descent rate
      dh_dt = -v_tas * sin(CAS_gamma);

      // change in speed with respect to altitude
      dv_dh = (1 / esf - 1) * (GRAVITY_METERS_PER_SECOND / v_tas);

      // acceleration rate
      dv_dt = dv_dh * dh_dt;

      h_new = dh_dt * delta_t + h;
      v_tas_new = dv_dt * delta_t + v_tas;

      v_cas_new = Units::MetersPerSecondSpeed(
            weather_prediction.getAtmosphere()->TAS2CAS(Units::MetersPerSecondSpeed(v_tas_new),
                                                        Units::MetersLength(h_new))).value();

      double gsnew = sqrt(pow(v_tas_new * cos(CAS_gamma), 2) -
                          pow(Units::MetersPerSecondSpeed(vwperp).value(), 2))
                     + Units::MetersPerSecondSpeed(vwpara).value();

      dist_new = dist - delta_t * gsnew;

      result.along_path_distance_m.push_back(dist_new);
      result.cas_mps.push_back(v_cas_new);
      result.altitude_m.push_back(h_new);
      result.altitude_rate_mps.push_back(dh_dt);
      result.true_airspeed.push_back(Units::MetersPerSecondSpeed(v_tas_new));
      result.tas_rate_mps.push_back(dv_dt);
      result.theta_radians.push_back(CAS_gamma);
      result.gs_mps.push_back(gsnew);
      result.wind_velocity_east.push_back(Vwx);
      result.wind_velocity_north.push_back(Vwy);
      result.algorithm_type.push_back(VerticalPath::PredictionAlgorithmType::CONSTANT_CAS);
      result.mass_kg.push_back(-1.0);
      curr_time = result.time_to_go_sec.back();
      result.time_to_go_sec.push_back(curr_time + fabs(delta_t)); // adds last time +0.5 to the end since fabs(delta_t) is 0.5

      if (!bracket_found && dist_new > Units::MetersLength(aircraft_distance_to_go).value()) {
         bracket_found = true;

         if ((h - Units::MetersLength(m_vertical_tolerance_distance).value()) >
             Units::MetersLength(m_start_altitude_msl).value()) {
            LOG4CPLUS_DEBUG(m_logger, "Prediction alt too high. pred: "
                  << h_new
                  << " start_alt: "
                  << Units::MetersLength(m_start_altitude_msl).value());
            m_prediction_too_high = true;
            return result;
         }
         if ((h_new + Units::MetersLength(m_vertical_tolerance_distance).value()) <
             Units::MetersLength(m_start_altitude_msl).value()) {
            LOG4CPLUS_DEBUG(m_logger, "Prediction alt too low. pred: "
                  << h_new
                  << " start_alt: "
                  << Units::MetersLength(m_start_altitude_msl).value());
            m_prediction_too_low = true;
            return result;
         }
      }

      dist = dist_new; // in meters
      h = h_new; // in meters
      v_cas = v_cas_new; // in meters per second

      if (m_precalculated_constraints.active_flag == ActiveFlagType::BELOW_ALT_ON_SPEED) {
         m_precalculated_constraints = CheckActiveConstraint(dist, h, v_cas, m_precalculated_constraints,
                                                             Units::MetersLength(m_transition_altitude_msl).value());
      }
   }

   if ((aircraft_distance_to_go < Units::MetersLength(Units::Infinity())) &&
       (h - Units::MetersLength(m_vertical_tolerance_distance).value()) >
       Units::MetersLength(m_start_altitude_msl).value()) {
      LOG4CPLUS_DEBUG(m_logger, "Prediction alt too high. pred: "
            << h
            << " start_alt: "
            << Units::MetersLength(m_start_altitude_msl).value());
      m_prediction_too_high = true;
   }

   return result;
}

VerticalPath KinematicDescent4DPredictor::ConstantMachVerticalPath(VerticalPath vertical_path,
                                                                   double altitude_at_end,
                                                                   vector<HorizontalPath> &horizontal_path,
                                                                   vector<PrecalcWaypoint> &precalc_waypoints,
                                                                   double gamma,
                                                                   const WeatherPrediction &weather_prediction,
                                                                   const Units::Length &aircraft_distance_to_go) {
   VerticalPath result;

   double delta_t = -0.5;
   double dist = vertical_path.along_path_distance_m[vertical_path.along_path_distance_m.size() - 1];
   double v_cas = vertical_path.cas_mps[vertical_path.cas_mps.size() - 1];
   double h = vertical_path.altitude_m[vertical_path.altitude_m.size() - 1];

   result = vertical_path;

   m_precalculated_constraints = FindActiveConstraint(dist, precalc_waypoints);
   m_precalculated_constraints = CheckActiveConstraint(dist, h, v_cas, m_precalculated_constraints,
                                                       Units::MetersLength(m_transition_altitude_msl).value());

   while ((h < altitude_at_end) && (m_precalculated_constraints.active_flag <= ActiveFlagType::BELOW_ALT_ON_SPEED
                                    || m_precalculated_constraints.active_flag == ActiveFlagType::BELOW_ALT_SLOW
                                    || m_precalculated_constraints.active_flag == ActiveFlagType::AT_ALT_SLOW)) {
      double v_tas = Units::MetersPerSecondSpeed(
            weather_prediction.getAtmosphere()->CAS2TAS(Units::MetersPerSecondSpeed(v_cas),
                                                        Units::MetersLength(h))).value();
      double esf;
      double dh_dt;
      double dv_dh;
      double dv_dt;
      double h_new;
      double dist_new;
      double v_tas_new;
      double v_cas_new;
      double curr_time;

      bool bracket_found = false;

      Units::Speed vwpara;
      Units::Speed vwperp;
      Units::Speed Vwx, Vwy;
      Units::UnsignedRadiansAngle course;
      m_course_calculator.CalculateCourseAtAlongPathDistance(Units::MetersLength(dist), course);
      ComputeWindCoefficients(Units::MetersLength(h), Units::RadiansAngle(course), weather_prediction, vwpara, vwperp,
                              Vwx, Vwy);

      esf = CalculateEsfUsingConstantMach(v_tas, h,
                                          weather_prediction.getAtmosphere()->GetTemperature(Units::MetersLength(h)));

      // climb/descent rate
      dh_dt = -v_tas * sin(gamma);

      // change in speed with respect to altitude
      dv_dh = (1 / esf - 1) * (GRAVITY_METERS_PER_SECOND / v_tas);

      // acceleration rate
      dv_dt = dv_dh * dh_dt;

      h_new = dh_dt * delta_t + h;

      v_tas_new = dv_dt * delta_t + v_tas;

      v_cas_new = Units::MetersPerSecondSpeed(
            weather_prediction.getAtmosphere()->TAS2CAS(Units::MetersPerSecondSpeed(v_tas_new),
                                                        Units::MetersLength(h_new))).value();

      double gsnew = sqrt(pow(v_tas_new * cos(gamma), 2) -
                          pow(Units::MetersPerSecondSpeed(vwperp).value(), 2))
                     + Units::MetersPerSecondSpeed(vwpara).value();

      dist_new = dist - delta_t * gsnew;

      result.along_path_distance_m.push_back(dist_new);
      result.cas_mps.push_back(v_cas_new);
      result.altitude_m.push_back(h_new);
      result.altitude_rate_mps.push_back(dh_dt);
      result.true_airspeed.push_back(Units::MetersPerSecondSpeed(v_tas_new));
      result.tas_rate_mps.push_back(dv_dt);
      result.theta_radians.push_back(gamma);
      result.gs_mps.push_back(gsnew);
      result.wind_velocity_east.push_back(Vwx);
      result.wind_velocity_north.push_back(Vwy);
      result.algorithm_type.push_back(VerticalPath::PredictionAlgorithmType::CONSTANT_MACH);
      result.mass_kg.push_back(-1.0);

      curr_time = result.time_to_go_sec.back();

      result.time_to_go_sec.push_back(curr_time + fabs(delta_t)); // adds last time +0.5 to the end since fabs(delta_t) is 0.5

      if (!bracket_found && dist_new > Units::MetersLength(aircraft_distance_to_go).value()) {
         bracket_found = true;
         if ((h - Units::MetersLength(m_vertical_tolerance_distance).value()) >
             Units::MetersLength(m_start_altitude_msl).value()) {
            LOG4CPLUS_DEBUG(m_logger, "Prediction alt too high. pred: "
                  << h_new
                  << " start_alt: "
                  << Units::MetersLength(m_start_altitude_msl).value());
            m_prediction_too_high = true;
            return result;
         }
         if ((h_new + Units::MetersLength(m_vertical_tolerance_distance).value()) <
             Units::MetersLength(m_start_altitude_msl).value()) {
            LOG4CPLUS_DEBUG(m_logger, "Prediction alt too low. pred: "
                  << h_new
                  << " start_alt: "
                  << Units::MetersLength(m_start_altitude_msl).value());
            m_prediction_too_low = true;
            return result;
         }
      }

      dist = dist_new; // in meters
      h = h_new; // in meters
      v_cas = v_cas_new; // in meters per second

      if (m_precalculated_constraints.active_flag == ActiveFlagType::BELOW_ALT_ON_SPEED
          || m_precalculated_constraints.active_flag == ActiveFlagType::BELOW_ALT_SLOW
          || m_precalculated_constraints.active_flag == ActiveFlagType::AT_ALT_SLOW) {
         m_precalculated_constraints = CheckActiveConstraint(dist, h, v_cas, m_precalculated_constraints,
                                                             Units::MetersLength(m_transition_altitude_msl).value());
      }
   }

   if ((aircraft_distance_to_go < Units::MetersLength(Units::Infinity())) &&
       (h - Units::MetersLength(m_vertical_tolerance_distance).value()) >
       Units::MetersLength(m_start_altitude_msl).value()) {
      LOG4CPLUS_DEBUG(m_logger, "Prediction alt too high. pred: "
            << h
            << " start_alt: "
            << Units::MetersLength(m_start_altitude_msl).value());
      m_prediction_too_high = true;
   }

   return result;
}

VerticalPath KinematicDescent4DPredictor::ConstantGeometricFpaVerticalPath(VerticalPath vertical_path,
                                                                           double altitude_at_end,
                                                                           double flight_path_angle,
                                                                           vector<HorizontalPath> &horizontal_path,
                                                                           vector<PrecalcWaypoint> &precalc_waypoints,
                                                                           const WeatherPrediction &weather_prediction,
                                                                           const Units::Length &aircraft_distance_to_go) {
   VerticalPath result = vertical_path;

   double delta_t = -0.5;
   double v_cas = vertical_path.cas_mps[vertical_path.cas_mps.size() - 1];
   double dist = vertical_path.along_path_distance_m[vertical_path.along_path_distance_m.size() - 1];
   double h = vertical_path.altitude_m[vertical_path.altitude_m.size() - 1];
   double theta = vertical_path.theta_radians[vertical_path.theta_radians.size() - 1];

   bool bracket_found = false;

   while (h < altitude_at_end) {
      double v_tas;
      double esf;
      double dh_dt;
      double dv_dh;
      double dv_dt;
      double h_new;
      double dist_new;
      double v_tas_new;
      double v_cas_new;
      double curr_time;

      Units::Speed vwpara;
      Units::Speed vwperp;
      Units::Speed Vwx, Vwy;
      Units::UnsignedRadiansAngle course;
      m_course_calculator.CalculateCourseAtAlongPathDistance(Units::MetersLength(dist), course);
      ComputeWindCoefficients(Units::MetersLength(h), Units::RadiansAngle(course), weather_prediction, vwpara, vwperp,
                              Vwx, Vwy);
      v_tas = Units::MetersPerSecondSpeed(
            weather_prediction.getAtmosphere()->CAS2TAS(Units::MetersPerSecondSpeed(v_cas),
                                                        Units::MetersLength(h))).value();

      if (h <= Units::MetersLength(m_transition_altitude_msl).value()) {
         esf = CalculateEsfUsingConstantCAS(v_tas, h,
                                            weather_prediction.getAtmosphere()->GetTemperature(Units::MetersLength(h)));
      } else {
         esf = CalculateEsfUsingConstantMach(v_tas, h,
                                             weather_prediction.getAtmosphere()->GetTemperature(
                                                   Units::MetersLength(h)));
      }

      // climb/descent rate
      dh_dt = -v_tas * sin(theta);

      // change in speed with respect to altitude
      dv_dh = (1 / esf - 1) * (GRAVITY_METERS_PER_SECOND / v_tas);

      // acceleration rate
      dv_dt = dv_dh * dh_dt;

      h_new = dh_dt * delta_t + h;

      v_tas_new = dv_dt * delta_t + v_tas;

      v_cas_new = Units::MetersPerSecondSpeed(
            weather_prediction.getAtmosphere()->TAS2CAS(Units::MetersPerSecondSpeed(v_tas_new),
                                                        Units::MetersLength(h_new))).value();

      double gsnew = sqrt(pow(v_tas_new * cos(theta), 2) -
                          pow(Units::MetersPerSecondSpeed(vwperp).value(), 2))
                     + Units::MetersPerSecondSpeed(vwpara).value();

      dist_new = gsnew * (-delta_t) + dist;

      double dh_dt_update = -gsnew * tan(flight_path_angle);

      if (fabs(dh_dt_update) > v_tas_new) {
         string msg =
                 string("ConstantGeometricVerticalPath is about to take asin() of a number greater than 1.0\n")
                 + string("This will result in theta becoming NaN");
         LOG4CPLUS_FATAL(m_logger, msg);
         throw logic_error(msg);
      }
      double theta_new = asin(-dh_dt_update / v_tas_new);

      result.along_path_distance_m.push_back(dist_new);
      result.cas_mps.push_back(v_cas_new);
      result.altitude_m.push_back(h_new);
      result.altitude_rate_mps.push_back(dh_dt);
      result.true_airspeed.push_back(Units::MetersPerSecondSpeed(v_tas_new));
      result.tas_rate_mps.push_back(dv_dt);
      result.theta_radians.push_back(theta_new);
      result.gs_mps.push_back(gsnew);
      result.wind_velocity_east.push_back(Vwx);
      result.wind_velocity_north.push_back(Vwy);
      result.algorithm_type.push_back(VerticalPath::PredictionAlgorithmType::FPA);
      result.mass_kg.push_back(-1.0);

      curr_time = result.time_to_go_sec.back();
      result.time_to_go_sec.push_back(curr_time + fabs(delta_t)); // adds last time +0.5 to the end since fabs(delta_t) is 0.5

      if (!bracket_found && dist_new > Units::MetersLength(aircraft_distance_to_go).value()) {
         bracket_found = true;
         if (h - Units::MetersLength(m_vertical_tolerance_distance).value() >
             Units::MetersLength(m_start_altitude_msl).value()) {
            LOG4CPLUS_DEBUG(m_logger, "Prediction alt too high. pred: "
                  << h_new
                  << " start_alt: "
                  << Units::MetersLength(m_start_altitude_msl).value());
            m_prediction_too_high = true;
            return result;
         }
         if ((h_new + Units::MetersLength(m_vertical_tolerance_distance).value()) <
             Units::MetersLength(m_start_altitude_msl).value()) {
            LOG4CPLUS_DEBUG(m_logger, "Prediction alt too low. pred: "
                  << h_new
                  << " start_alt: "
                  << Units::MetersLength(m_start_altitude_msl).value());
            m_prediction_too_low = true;
            return result;
         }
      }

      dist = dist_new; // in meters
      h = h_new; // in meters
      v_cas = v_cas_new; // in meters per second
      theta = theta_new;
   }

   if ((aircraft_distance_to_go < Units::MetersLength(Units::Infinity())) &&
       (h - Units::MetersLength(m_vertical_tolerance_distance).value()) >
       Units::MetersLength(m_start_altitude_msl).value()) {
      LOG4CPLUS_DEBUG(m_logger, "Prediction alt too high. pred: "
            << h
            << " start_alt: "
            << Units::MetersLength(m_start_altitude_msl).value());
      m_prediction_too_high = true;
   }

   return result;
}

VerticalPath KinematicDescent4DPredictor::ConstantFpaDecelerationVerticalPath(VerticalPath vertical_path,
                                                                              double altitude_at_end,
                                                                              double velocity_cas_end,
                                                                              double flight_path_angle,
                                                                              vector<HorizontalPath> &horizontal_path,
                                                                              vector<PrecalcWaypoint> &precalc_waypoints,
                                                                              const WeatherPrediction &weather_prediction,
                                                                              const Units::Length &aircraft_distance_to_go) {
   VerticalPath result = vertical_path;

   double delta_t = -0.5;
   double v_cas = vertical_path.cas_mps[vertical_path.cas_mps.size() - 1];
   double dist = vertical_path.along_path_distance_m[vertical_path.along_path_distance_m.size() - 1];
   double h = vertical_path.altitude_m[vertical_path.altitude_m.size() - 1];
   double theta = vertical_path.theta_radians[vertical_path.theta_radians.size() - 1];

   while ((h < altitude_at_end) && (v_cas < velocity_cas_end)) {
      double v_tas = Units::MetersPerSecondSpeed(
            weather_prediction.getAtmosphere()->CAS2TAS(Units::MetersPerSecondSpeed(v_cas),
                                                        Units::MetersLength(h))).value();
      Units::KilogramsMeterDensity rho;
      Units::Pressure p;
      double dh_dt;
      double dv_dt;
      double h_new;
      double dist_new;
      double v_tas_new;
      double v_cas_new;
      double curr_time;

      bool bracket_found = false;

      Units::MetersPerSecondSpeed Vwx, Vwy;
      Units::HertzFrequency dVwx_dh, dVwy_dh;
      Units::UnsignedRadiansAngle course;
      m_course_calculator.CalculateCourseAtAlongPathDistance(Units::MetersLength(dist), course);
      m_wind_calculator.ComputeWindGradients(Units::MetersLength(h), weather_prediction, Vwx, Vwy, dVwx_dh, dVwy_dh);

      double Vw_para = Vwx.value() * cos(course) + Vwy.value() * sin(course);
      double Vw_perp = -Vwx.value() * sin(course) + Vwy.value() * cos(course);

      weather_prediction.getAtmosphere()->AirDensity(Units::MetersLength(h), rho, p);

      double gs_new = sqrt(pow(v_tas * cos(theta), 2) - pow(Vw_perp, 2)) + Vw_para;

      // climb/descent rate
      dh_dt = -gs_new * tan(flight_path_angle);

      // TODO:turn the 0.3 into a constant, (maybe decel?).

      // acceleration rate
      dv_dt = -0.3 * KNOTS_TO_METERS_PER_SECOND;

      h_new = dh_dt * delta_t + h;
      v_tas_new = dv_dt * delta_t + v_tas;
      v_cas_new = Units::MetersPerSecondSpeed(
            weather_prediction.getAtmosphere()->TAS2CAS(Units::MetersPerSecondSpeed(v_tas_new),
                                                        Units::MetersLength(h_new))).value();
      dist_new = gs_new * (-delta_t) + dist;
      if (fabs(dh_dt) > v_tas_new) {
         string msg =
                 string("ConstantFpaDecelerationVerticalPath is about to take asin() of a number greater than 1.0\n")
                 + string("This will result in theta becoming NaN");
         LOG4CPLUS_FATAL(m_logger, msg);
         throw logic_error(msg);
      }
      theta = asin((-dh_dt) / v_tas_new);

      result.along_path_distance_m.push_back(dist_new);
      result.cas_mps.push_back(v_cas_new);
      result.altitude_m.push_back(h_new);
      result.altitude_rate_mps.push_back(dh_dt);
      result.true_airspeed.push_back(Units::MetersPerSecondSpeed(v_tas_new));
      result.tas_rate_mps.push_back(dv_dt);
      result.theta_radians.push_back(theta);
      result.gs_mps.push_back(gs_new);
      result.mass_kg.push_back(-1.0);
      result.wind_velocity_east.push_back(Vwx);
      result.wind_velocity_north.push_back(Vwy);
      result.algorithm_type.push_back(VerticalPath::PredictionAlgorithmType::FPA_DECEL);
      curr_time = result.time_to_go_sec.back();
      result.time_to_go_sec.push_back(curr_time + fabs(delta_t)); // adds last time +0.5 to the end since fabs(delta_t) is 0.5

      if (!bracket_found && dist_new > Units::MetersLength(aircraft_distance_to_go).value()) {
         bracket_found = true;
         if ((h - Units::MetersLength(m_vertical_tolerance_distance).value()) >
             Units::MetersLength(m_start_altitude_msl).value()) {
            LOG4CPLUS_DEBUG(m_logger, "Prediction alt too high. pred: "
                  << h_new
                  << " start_alt: "
                  << Units::MetersLength(m_start_altitude_msl).value());
            m_prediction_too_high = true;
            return result;
         }
         if ((h_new + Units::MetersLength(m_vertical_tolerance_distance).value()) <
             Units::MetersLength(m_start_altitude_msl).value()) {
            LOG4CPLUS_DEBUG(m_logger, "Prediction alt too low. pred: "
                  << h_new
                  << " start_alt: "
                  << Units::MetersLength(m_start_altitude_msl).value());
            m_prediction_too_low = true;
            return result;
         }
      }

      dist = dist_new; // in meters
      h = h_new; // in meters
      v_cas = v_cas_new; // in meters per second
   }

   if ((aircraft_distance_to_go < Units::MetersLength(Units::Infinity())) &&
       (h - Units::MetersLength(m_vertical_tolerance_distance).value()) >
       Units::MetersLength(m_start_altitude_msl).value()) {
      LOG4CPLUS_DEBUG(m_logger, "Prediction alt too high. pred: "
            << h
            << " start_alt: "
            << Units::MetersLength(m_start_altitude_msl).value());
      m_prediction_too_high = true;
   }

   return result;
}

VerticalPath KinematicDescent4DPredictor::LevelDecelerationVerticalPath(
      VerticalPath vertical_path,
      double deceleration,
      double velocity_cas_end,
      vector<HorizontalPath> &horizontal_path,
      const WeatherPrediction &weather_prediction,
      const Units::Length &aircraft_distance_to_go) {

   VerticalPath result = vertical_path;

   double delta_t = -0.5;
   double v_cas = vertical_path.cas_mps[vertical_path.cas_mps.size() - 1];
   double dist = vertical_path.along_path_distance_m[vertical_path.along_path_distance_m.size() - 1];
   double h = vertical_path.altitude_m[vertical_path.altitude_m.size() - 1];

   while (v_cas < velocity_cas_end) {
      double v_tas = Units::MetersPerSecondSpeed(
            weather_prediction.getAtmosphere()->CAS2TAS(Units::MetersPerSecondSpeed(v_cas),
                                                        Units::MetersLength(h))).value();
      double dh_dt;
      double dv_dt;
      double h_new;
      double dist_new;
      double v_tas_new;
      double v_cas_new;
      double curr_time;

      Units::Speed vwpara;
      Units::Speed vwperp;
      Units::Speed Vwx, Vwy;
      Units::UnsignedRadiansAngle course;
      m_course_calculator.CalculateCourseAtAlongPathDistance(Units::MetersLength(dist), course);
      ComputeWindCoefficients(Units::MetersLength(h), Units::RadiansAngle(course), weather_prediction, vwpara, vwperp,
                              Vwx, Vwy);


      // climb/descent rate
      dh_dt = 0.0;
      double theta_new = 0.0;

      // acceleration rate
      dv_dt = -deceleration;

      h_new = dh_dt * delta_t + h;
      v_tas_new = dv_dt * delta_t + v_tas;
      v_cas_new = Units::MetersPerSecondSpeed(
            weather_prediction.getAtmosphere()->TAS2CAS(Units::MetersPerSecondSpeed(v_tas_new),
                                                        Units::MetersLength(h_new))).value();

      double gsnew = sqrt(pow(v_tas_new * cos(theta_new), 2) -
                          pow(Units::MetersPerSecondSpeed(vwperp).value(), 2)) +
                     Units::MetersPerSecondSpeed(vwpara).value();

      dist_new = dist - delta_t * gsnew;

      result.along_path_distance_m.push_back(dist_new);
      result.cas_mps.push_back(v_cas_new);
      result.altitude_m.push_back(h_new);
      result.altitude_rate_mps.push_back(dh_dt);
      result.true_airspeed.push_back(Units::MetersPerSecondSpeed(v_tas_new));
      result.tas_rate_mps.push_back(dv_dt);
      result.theta_radians.push_back(theta_new);
      result.gs_mps.push_back(gsnew);
      result.mass_kg.push_back(-1.0);
      result.wind_velocity_east.push_back(Vwx);
      result.wind_velocity_north.push_back(Vwy);
      result.algorithm_type.push_back(VerticalPath::PredictionAlgorithmType::LEVEL_DECEL1);

      curr_time = result.time_to_go_sec.back();
      result.time_to_go_sec.push_back(curr_time + fabs(delta_t)); // adds last time +0.5 to the end since fabs(delta_t) is 0.5

      dist = dist_new; // in meters
      h = h_new; // in meters
      v_cas = v_cas_new; // in meters per second
   }

   if ((result.along_path_distance_m.back() > Units::MetersLength(aircraft_distance_to_go).value()) &&
       ((h + Units::MetersLength(m_vertical_tolerance_distance).value()) <
        Units::MetersLength(m_start_altitude_msl).value())) {
      LOG4CPLUS_DEBUG(m_logger, "Prediction alt too low. pred: "
            << h
            << " start_alt: "
            << Units::MetersLength(m_start_altitude_msl).value());
      m_prediction_too_low = true;
   }

   return result;
}

VerticalPath KinematicDescent4DPredictor::LevelDecelerationVerticalPath(
      VerticalPath vertical_path,
      Units::Length distance_to_go,
      double deceleration,
      double velocity_cas_end,
      vector<HorizontalPath> &horizontal_path,
      const WeatherPrediction &weather_prediction,
      const Units::Length &aircraft_distance_to_go) {

   VerticalPath result = vertical_path;

   double delta_t = -0.5;
   double v_cas = vertical_path.cas_mps[vertical_path.cas_mps.size() - 1];
   double dist = vertical_path.along_path_distance_m[vertical_path.along_path_distance_m.size() - 1];
   double h = vertical_path.altitude_m[vertical_path.altitude_m.size() - 1];

   double distEnd = Units::MetersLength(distance_to_go).value();

   while (v_cas < velocity_cas_end && dist <= distEnd) {
      double v_tas = Units::MetersPerSecondSpeed(
            weather_prediction.getAtmosphere()->CAS2TAS(Units::MetersPerSecondSpeed(v_cas),
                                                        Units::MetersLength(h))).value();
      double dh_dt;
      double dv_dt;
      double h_new;
      double dist_new;
      double v_tas_new;
      double v_cas_new;
      double curr_time;

      Units::Speed vwpara;
      Units::Speed vwperp;
      Units::Speed Vwx, Vwy;
      Units::UnsignedRadiansAngle course;
      m_course_calculator.CalculateCourseAtAlongPathDistance(Units::MetersLength(dist), course);
      ComputeWindCoefficients(Units::MetersLength(h), Units::RadiansAngle(course), weather_prediction, vwpara, vwperp,
                              Vwx, Vwy);


      // climb/descent rate
      dh_dt = 0.0;
      double theta_new = 0.0;

      // acceleration rate
      dv_dt = -deceleration;

      h_new = dh_dt * delta_t + h;
      v_tas_new = dv_dt * delta_t + v_tas;
      v_cas_new = Units::MetersPerSecondSpeed(
            weather_prediction.getAtmosphere()->TAS2CAS(Units::MetersPerSecondSpeed(v_tas_new),
                                                        Units::MetersLength(h_new))).value();

      double gsnew = sqrt(pow(v_tas_new * cos(theta_new), 2) -
                          pow(Units::MetersPerSecondSpeed(vwperp).value(), 2)) +
                     Units::MetersPerSecondSpeed(vwpara).value();

      dist_new = dist - delta_t * gsnew;

      result.along_path_distance_m.push_back(dist_new);
      result.cas_mps.push_back(v_cas_new);
      result.altitude_m.push_back(h_new);
      result.altitude_rate_mps.push_back(dh_dt);
      result.true_airspeed.push_back(Units::MetersPerSecondSpeed(v_tas_new));
      result.tas_rate_mps.push_back(dv_dt);
      result.theta_radians.push_back(theta_new);
      result.gs_mps.push_back(gsnew);
      result.mass_kg.push_back(-1.0);
      result.wind_velocity_east.push_back(Vwx);
      result.wind_velocity_north.push_back(Vwy);
      result.algorithm_type.push_back(VerticalPath::PredictionAlgorithmType::LEVEL_DECEL2);

      curr_time = result.time_to_go_sec.back();
      result.time_to_go_sec.push_back(curr_time + fabs(delta_t)); // adds last time +0.5 to the end since fabs(delta_t) is 0.5

      dist = dist_new; // in meters
      h = h_new; // in meters
      v_cas = v_cas_new; // in meters per second
   }

   if ((result.along_path_distance_m.back() > Units::MetersLength(aircraft_distance_to_go).value()) &&
       ((h + Units::MetersLength(m_vertical_tolerance_distance).value()) <
        Units::MetersLength(m_start_altitude_msl).value())) {
      LOG4CPLUS_DEBUG(m_logger, "Prediction alt too low. pred: "
            << h
            << " start_alt: "
            << Units::MetersLength(m_start_altitude_msl).value());
      m_prediction_too_low = true;
   }

   return result;
}

VerticalPath KinematicDescent4DPredictor::LevelVerticalPath(VerticalPath vertical_path,
                                                            double x_end,
                                                            vector<HorizontalPath> &horizontal_path,
                                                            const WeatherPrediction &weather_prediction,
                                                            const Units::Length &aircraft_distance_to_go) {
   VerticalPath result;

   double delta_t = -0.5;
   double v_cas = vertical_path.cas_mps[vertical_path.cas_mps.size() - 1];
   double h = vertical_path.altitude_m[vertical_path.altitude_m.size() - 1];
   double dist = vertical_path.along_path_distance_m[vertical_path.along_path_distance_m.size() - 1];

   result = vertical_path;

   while (fabs(dist) < fabs(x_end)) {
      double v_tas = Units::MetersPerSecondSpeed(
            weather_prediction.getAtmosphere()->CAS2TAS(Units::MetersPerSecondSpeed(v_cas),
                                                        Units::MetersLength(h))).value();
      double dh_dt;
      double dv_dt;
      double h_new;
      double dist_new;
      double v_tas_new;
      double v_cas_new;
      double theta_new;
      double curr_time;
      double gsnew;

      Units::Speed vwpara;
      Units::Speed vwperp;
      Units::Speed Vwx, Vwy;
      Units::UnsignedRadiansAngle course;
      m_course_calculator.CalculateCourseAtAlongPathDistance(Units::MetersLength(dist), course);
      ComputeWindCoefficients(Units::MetersLength(h), Units::RadiansAngle(course), weather_prediction, vwpara, vwperp,
                              Vwx, Vwy);

      // climb/descent rate
      dh_dt = 0.0;

      // acceleration rate
      dv_dt = 0.0;

      theta_new = 0.0;

      h_new = dh_dt * delta_t + h;

      v_tas_new = dv_dt * delta_t + v_tas;

      v_cas_new = Units::MetersPerSecondSpeed(
            weather_prediction.getAtmosphere()->TAS2CAS(Units::MetersPerSecondSpeed(v_tas_new),
                                                        Units::MetersLength(h_new))).value();

      gsnew = sqrt(pow(v_tas_new * cos(theta_new), 2) -
                   pow(Units::MetersPerSecondSpeed(vwperp).value(), 2)) +
              Units::MetersPerSecondSpeed(vwpara).value();

      dist_new = gsnew * (-delta_t) + dist;

      result.along_path_distance_m.push_back(dist_new);
      result.cas_mps.push_back(v_cas_new);
      result.altitude_m.push_back(h_new);
      result.altitude_rate_mps.push_back(dh_dt);
      result.true_airspeed.push_back(Units::MetersPerSecondSpeed(v_tas_new));
      result.tas_rate_mps.push_back(dv_dt);
      result.theta_radians.push_back(theta_new);
      result.gs_mps.push_back(gsnew);
      result.wind_velocity_east.push_back(Vwx);
      result.wind_velocity_north.push_back(Vwy);
      result.algorithm_type.push_back(VerticalPath::PredictionAlgorithmType::LEVEL);

      result.mass_kg.push_back(-1.0);

      curr_time = result.time_to_go_sec.back();
      result.time_to_go_sec.push_back(curr_time + fabs(delta_t)); // adds last time +0.5 to the end since fabs(delta_t) is 0.5


      // set values for next iteration of the loop
      dist = dist_new;
      h = h_new;
      v_cas = v_cas_new;
   }

   if ((result.along_path_distance_m.back() > Units::MetersLength(aircraft_distance_to_go).value()) &&
       ((h + Units::MetersLength(m_vertical_tolerance_distance).value()) <
        Units::MetersLength(m_start_altitude_msl).value())) {
      LOG4CPLUS_DEBUG(m_logger, "Prediction alt too low. pred: "
            << h
            << " start_alt: "
            << Units::MetersLength(m_start_altitude_msl).value());
      m_prediction_too_low = true;
   }

   return result;
}

VerticalPath KinematicDescent4DPredictor::ConstantDecelerationVerticalPath(VerticalPath vertical_path,
                                                                           Units::Length distance_to_go,
                                                                           Units::Length altitude_high,
                                                                           double deceleration,
                                                                           double velocity_cas_end,
                                                                           vector<HorizontalPath> &horizontal_path,
                                                                           const WeatherPrediction &weather_prediction,
                                                                           const Units::Length &aircraft_distance_to_go) {
   VerticalPath result = vertical_path;

   double delta_t = -0.5;
   double dist = vertical_path.along_path_distance_m[vertical_path.along_path_distance_m.size() - 1];
   double v_cas = vertical_path.cas_mps[vertical_path.cas_mps.size() - 1];
   double h = vertical_path.altitude_m[vertical_path.altitude_m.size() - 1];

   double distEnd = Units::MetersLength(distance_to_go).value();
   double altEnd = Units::MetersLength(altitude_high).value();

   bool bracket_found = false;

   while (v_cas < velocity_cas_end && h < altEnd && dist < distEnd) {
      double v_tas = Units::MetersPerSecondSpeed(
            weather_prediction.getAtmosphere()->CAS2TAS(Units::MetersPerSecondSpeed(v_cas),
                                                        Units::MetersLength(h))).value();
      double dh_dt;
      double dv_dt;
      double h_new;
      double dist_new;
      double v_tas_new;
      double v_cas_new;
      double curr_time;

      Units::Speed vwpara;
      Units::Speed vwperp;
      Units::Speed Vwx, Vwy;
      Units::UnsignedRadiansAngle course;
      m_course_calculator.CalculateCourseAtAlongPathDistance(Units::MetersLength(dist), course);
      ComputeWindCoefficients(Units::MetersLength(h), Units::RadiansAngle(course), weather_prediction, vwpara, vwperp,
                              Vwx, Vwy);

      // One degree descent
      double theta_new = PI / 180.0;


      // climb/descent rate
      dh_dt = -v_tas * sin(theta_new);

      // acceleration rate
      dv_dt = -deceleration;

      h_new = dh_dt * delta_t + h;

      v_tas_new = dv_dt * delta_t + v_tas;

      v_cas_new = Units::MetersPerSecondSpeed(
            weather_prediction.getAtmosphere()->TAS2CAS(Units::MetersPerSecondSpeed(v_tas_new),
                                                        Units::MetersLength(h_new))).value();

      double gsnew = sqrt(pow(v_tas_new * cos(theta_new), 2) - pow(Units::MetersPerSecondSpeed(vwperp).value(), 2)) +
                     Units::MetersPerSecondSpeed(vwpara).value();

      dist_new = dist - delta_t * gsnew;

      result.along_path_distance_m.push_back(dist_new);
      result.cas_mps.push_back(v_cas_new);
      result.altitude_m.push_back(h_new);
      result.altitude_rate_mps.push_back(dh_dt);
      result.true_airspeed.push_back(Units::MetersPerSecondSpeed(v_tas_new));
      result.tas_rate_mps.push_back(dv_dt);
      result.theta_radians.push_back(theta_new);
      result.gs_mps.push_back(gsnew);
      result.wind_velocity_east.push_back(Vwx);
      result.wind_velocity_north.push_back(Vwy);
      result.algorithm_type.push_back(VerticalPath::PredictionAlgorithmType::CONSTANT_DECEL);

      result.mass_kg.push_back(-1.0);

      curr_time = result.time_to_go_sec.back();

      result.time_to_go_sec.push_back(curr_time + fabs(delta_t)); // adds last time +0.5 to the end since fabs(delta_t) is 0.5

      if (!bracket_found && dist_new > Units::MetersLength(aircraft_distance_to_go).value()) {
         bracket_found = true;
         if ((h - Units::MetersLength(m_vertical_tolerance_distance).value()) >
             Units::MetersLength(m_start_altitude_msl).value()) {
            LOG4CPLUS_DEBUG(m_logger, "Prediction alt too high. pred: "
                  << h_new
                  << " start_alt: "
                  << Units::MetersLength(m_start_altitude_msl).value());
            m_prediction_too_high = true;
            return result;
         }
         if ((h_new + Units::MetersLength(m_vertical_tolerance_distance).value()) <
             Units::MetersLength(m_start_altitude_msl).value()) {
            LOG4CPLUS_DEBUG(m_logger, "Prediction alt too low. pred: "
                  << h_new
                  << " start_alt: "
                  << Units::MetersLength(m_start_altitude_msl).value());
            m_prediction_too_low = true;
            return result;
         }
      }

      // set values for next iteration of the loop
      dist = dist_new; // in meters
      h = h_new; // in meters
      v_cas = v_cas_new; // in meters per second

   }

   if ((aircraft_distance_to_go < Units::MetersLength(Units::Infinity())) &&
       (h - Units::MetersLength(m_vertical_tolerance_distance).value()) >
       Units::MetersLength(m_start_altitude_msl).value()) {
      LOG4CPLUS_DEBUG(m_logger, "Prediction alt too high. pred: "
            << h
            << " start_alt: "
            << Units::MetersLength(m_start_altitude_msl).value());
      m_prediction_too_high = true;
   }

   return result;
}

VerticalPath KinematicDescent4DPredictor::ConstantFpaToCurrentPositionVerticalPath(VerticalPath vertical_path,
                                                                                   std::vector<HorizontalPath> &horizontal_path,
                                                                                   std::vector<PrecalcWaypoint> &precalc_waypoints,
                                                                                   double const_gamma_mach,
                                                                                   const WeatherPrediction &weather_prediction,
                                                                                   const Units::Length &aircraft_distance_to_go) {

   Units::Length distance_to_plan = aircraft_distance_to_go;

   // Should probably throw exception if trying to re-predict with aircraft distance to go set to infinite
   if (aircraft_distance_to_go == Units::MetersLength(Units::infinity())) {
      LOG4CPLUS_ERROR(m_logger,
                      "Attempting to re-predict constrained vertical path with infinite aircraft distance to go.");
      distance_to_plan = Units::MetersLength(horizontal_path.back().m_path_length_cumulative_meters);
   }

   VerticalPath result = vertical_path;
   result.algorithm_type.back() = VerticalPath::FPA_TO_CURRENT_POS;
   PrecalcConstraint constraints;

   double dist = vertical_path.along_path_distance_m[vertical_path.along_path_distance_m.size() - 1];
   double v_cas = vertical_path.cas_mps[vertical_path.cas_mps.size() - 1];
   double h = vertical_path.altitude_m[vertical_path.altitude_m.size() - 1];

   double descent_ratio =
         (m_start_altitude_msl.value() - h) / (Units::MetersLength(aircraft_distance_to_go).value() - dist);
   double fpa = atan2(m_start_altitude_msl.value() - h, Units::MetersLength(aircraft_distance_to_go).value() - dist);

   while (dist < Units::MetersLength(aircraft_distance_to_go).value()
          && h < Units::MetersLength(m_transition_altitude_msl).value()) {
      constraints = FindActiveConstraint(dist, precalc_waypoints);
      double distance_left = constraints.constraint_dist;
      if (distance_left > Units::MetersLength(aircraft_distance_to_go).value()) {
         distance_left = Units::MetersLength(aircraft_distance_to_go).value();
      }
      double altitude_at_end = h + (distance_left - dist) * descent_ratio;
      // The first step in ConstantFPADecelerationVerticalPath() and ConstantGeometricVerticalPath() uses the
      // previous value of theta.  A new theta is not calculated until the second step.  If the aircraft
      // position is close to the waypoint calculation of theta can result in NaN or an FPA angle too high.
      // See Issue AAES-1025.  This should rarely occur.
      if ( descent_ratio > 0.2 ) {
         LOG4CPLUS_DEBUG(m_logger, "Aircraft position too close to waypoint for adequate FPA.  See Issue AAES-1025");
         result = LevelVerticalPath(result, Units::MetersLength(aircraft_distance_to_go).value(), horizontal_path,
                 weather_prediction, aircraft_distance_to_go);
         result.altitude_m.back() = altitude_at_end;
         if (result.altitude_m.size() > 2)
            result.altitude_m[result.altitude_m.size() - 2] = altitude_at_end;
         h = result.altitude_m.back();
         dist = result.along_path_distance_m.back();
      }

      while (dist < constraints.constraint_dist && h < altitude_at_end) {
         if (v_cas < constraints.constraint_speedHi) {
            result = ConstantFpaDecelerationVerticalPath(result, altitude_at_end,
                                                         constraints.constraint_speedHi, fpa, horizontal_path,
                                                         precalc_waypoints, weather_prediction,
                                                         Units::Length(Units::infinity()));
         }
         result = ConstantGeometricFpaVerticalPath(result, altitude_at_end, fpa, horizontal_path, precalc_waypoints,
                                                   weather_prediction, Units::Length(Units::infinity()));

         dist = result.along_path_distance_m.back();
         v_cas = result.cas_mps.back();
         h = result.altitude_m.back();
         if (dist > Units::MetersLength(aircraft_distance_to_go).value()) {
            break;
         }
      }
      if (h > altitude_at_end && dist < constraints.constraint_dist) {
         result = LevelVerticalPath(result, constraints.constraint_dist, horizontal_path, weather_prediction,
                                    Units::Length(Units::infinity()));
      }
   }

   // either at transition altitude or prediction goes past aircraft distance to go
   if (result.altitude_m.back() < Units::MetersLength(m_start_altitude_msl).value()) {
      result = ConstantMachVerticalPath(result, m_start_altitude_msl.value(), horizontal_path,
                                        precalc_waypoints, const_gamma_mach, weather_prediction,
                                        Units::Length(Units::infinity()));
   }

   double prediction_dist = precalc_waypoints[precalc_waypoints.size() - 1].m_precalc_constraints.constraint_dist;
   result = LevelVerticalPath(result, prediction_dist,
                              horizontal_path, weather_prediction, Units::Length(Units::infinity()));
   return result;
};

void KinematicDescent4DPredictor::ComputeWindCoefficients(Units::Length altitude,
                                                          Units::Angle course,
                                                          const WeatherPrediction &weather_prediction,
                                                          Units::Speed &parallel_wind_velocity,
                                                          Units::Speed &perpendicular_wind_velocity,
                                                          Units::Speed &wind_velocity_x,
                                                          Units::Speed &wind_velocity_y) {
   Units::HertzFrequency dVwx_dh, dVwy_dh;

   m_wind_calculator.ComputeWindGradients(altitude, weather_prediction, wind_velocity_x, wind_velocity_y, dVwx_dh,
                                          dVwy_dh);

   parallel_wind_velocity = wind_velocity_x * cos(course) + wind_velocity_y * sin(course);
   perpendicular_wind_velocity = -wind_velocity_x * sin(course) + wind_velocity_y * cos(course);
};

