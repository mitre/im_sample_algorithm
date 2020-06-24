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

#include "imalgs/AchievePointCalcs.h"
#include "utility/CustomUnits.h"
#include "math/CustomMath.h"

#include <string>
#include <iomanip>

using namespace std;

#define SQR(x) ({    \
    typeof(x) y = (x);  \
    y*y;                \
})

log4cplus::Logger AchievePointCalcs::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("AchievePointCalcs"));

AchievePointCalcs::AchievePointCalcs() {
   Clear();
   m_waypoint_is_set = false;
}

AchievePointCalcs::AchievePointCalcs(const string &waypoint,
                                     const AircraftIntent &intent,
                                     const VerticalPath &vpath,
                                     const vector<HorizontalPath> &htraj) {
   Clear();

   m_waypoint_name = waypoint;
   m_waypoint_is_set = !m_waypoint_name.empty();
   m_horizontal_path = htraj;
   m_distance_calculator = AlongPathDistanceCalculator(htraj, TrajectoryIndexProgressionDirection::UNDEFINED);

   if (m_waypoint_is_set) {
      if (m_waypoint_name == "CALCULATED_TRP") {
         throw logic_error("TRP cannot be calculated from this constructor.");
      }
      ComputePositions(intent);
      ComputeEndValues(vpath);
   }
}

AchievePointCalcs::AchievePointCalcs(const string &waypoint,
                                     const AircraftIntent &intent,
                                     const VerticalPath &vpath,
                                     const vector<HorizontalPath> &htraj,
                                     const AchievePointCalcs &ownship_calcs,
                                     const AircraftIntent &ownship_intent) {
   Clear();

   m_waypoint_name = waypoint;
   m_waypoint_is_set = !m_waypoint_name.empty();
   m_horizontal_path = htraj;
   m_distance_calculator = AlongPathDistanceCalculator(htraj, TrajectoryIndexProgressionDirection::UNDEFINED);

   if (m_waypoint_name == "CALCULATED_TRP") {
      ComputeDefaultTRP(ownship_calcs, ownship_intent);
      LOG4CPLUS_DEBUG(m_logger, "Calculated TRP = (" << m_waypoint_x << "," << m_waypoint_y << ")");
      // log the geographic coordinates
      EarthModel::LocalPositionEnu xy;
      xy.x = m_waypoint_x;
      xy.y = m_waypoint_y;
      xy.z = Units::zero();
      EarthModel::GeodeticPosition geo;
      intent.GetTangentPlaneSequence()->convertLocalToGeodetic(xy, geo);
      LOG4CPLUS_DEBUG(m_logger, "(lat,lon) = (" <<
                                                std::setprecision(10) << Units::DegreesAngle(geo.latitude) <<
                                                "," << Units::DegreesAngle(geo.longitude) << ")");
   } else {
      ComputePositions(intent);
   }
   ComputeEndValues(vpath);
}

AchievePointCalcs::~AchievePointCalcs() = default;


void AchievePointCalcs::Clear() {

   m_waypoint_name = "";
   m_waypoint_is_set = false;

   m_waypoint_x = Units::ZERO_LENGTH;
   m_waypoint_y = Units::ZERO_LENGTH;

   m_distance_from_waypoint = Units::ZERO_LENGTH;
   m_time_to_go_to_waypoint = Units::ZERO_TIME;

   m_crossing_time = Units::SecondsTime(-999);
   m_horizontal_path.clear();
   m_distance_calculator = AlongPathDistanceCalculator();
}


void AchievePointCalcs::ComputeDefaultTRP(
      const AchievePointCalcs &ownship_calcs,
      const AircraftIntent &ownship_intent) {

   // "this" is the AchievePointCalcs object for the target trajectory.

   // Get ABP index from ownship
   int ix0 = ownship_intent.GetWaypointIndexByName(ownship_calcs.GetWaypointName());
   if (ix0 < 0) {
      string emsg = "Illegal ownship achieve-by point " + ownship_calcs.GetWaypointName() +
                    " encountered while computing achieve point calculations positions";
      LOG4CPLUS_FATAL(AchievePointCalcs::m_logger, emsg);
      throw logic_error(emsg);
   }

   int ix1 = ix0 + 1;
   if (ix1 >= ownship_intent.GetNumberOfWaypoints()) {
      ix1 = ix0 - 1;  // look upstream instead
      if (ix1 < 0) {
         string emsg = "Ownship achieve point " + ownship_calcs.GetWaypointName() +
                       " is the only point in the intent.  Unable to determine direction.";
         LOG4CPLUS_FATAL(AchievePointCalcs::m_logger, emsg);
         throw logic_error(emsg);
      }
   }

   double a, b, c;
   {
      const struct AircraftIntent::RouteData &ownship_fms(ownship_intent.GetFms());

      double x0 = ownship_fms.m_x[ix0].value();
      double y0 = ownship_fms.m_y[ix0].value();
      double x_diff = x0 - ownship_fms.m_x[ix1].value();
      double y_diff = y0 - ownship_fms.m_y[ix1].value();
      // we need the line perpendicular to this which goes through (x0,y0)
      // construct an equation of the form a*x + b*y = c
      double normalize = hypot(x_diff, y_diff);
      a = x_diff / normalize;
      b = y_diff / normalize;
      c = a * x0 + b * y0;

      LOG4CPLUS_TRACE(m_logger, "ABP = (" << x0 << "," << y0 << ")");
      LOG4CPLUS_TRACE(m_logger, "diff to next = (" << x_diff << "," << y_diff << ")");
      LOG4CPLUS_TRACE(m_logger, "a = " << a << ", b = " << b << ", c = " << c);
   }

   // Search horizontal path for an intersecting segment
   bool first(true), crossed(false), end_is_usable(false);
   double x0, y0, d0;
   vector<HorizontalPath>::iterator hpi;
   for (hpi = m_horizontal_path.begin(); hpi != m_horizontal_path.end(); ++hpi) {
      double x1 = hpi->GetXPositionMeters();
      double y1 = hpi->GetYPositionMeters();
      double d1 = a * x1 + b * y1 - c;
      LOG4CPLUS_TRACE(m_logger, "P1 = (" << x1 << "," << y1 << "), d1 = " << d1);
      if (first) {
         first = false;
         if (abs(d1) <= 50) {
            end_is_usable = true;
            // record "first" point (chronologically last), but keep searching
            m_waypoint_x = Units::MetersLength(x1);
            m_waypoint_y = Units::MetersLength(y1);
            LOG4CPLUS_DEBUG(m_logger, "Target trajectory ends " << d1 <<
                                                                " m from ABP projection line.  End point will be used if no crossing is found.");
         }
      } else {
         // Is (x1,y1) exactly on the line?
         if (d1 == 0) {
            m_waypoint_x = Units::MetersLength(x1);
            m_waypoint_y = Units::MetersLength(y1);
            crossed = true;
            break;
         }
         if ((hpi - 1)->m_segment_type != HorizontalPath::TURN) {
            // test if p0 and p1 are on opposite sides of the line
            if (d0 * d1 < 0) {
               // For now, interpolate to get d=0
               double dist_change = d1 - d0;
               // suppose x0 = 99, x1 = 107, d0 = -.25, d1 = 1.75
               // dist_change = 2
               // x = (99 * 1.75 + 107 * -.25) / 2 = 100
               double x = (x0 * d1 - x1 * d0) / dist_change;
               double y = (y0 * d1 - y1 * d0) / dist_change;
               m_waypoint_x = Units::MetersLength(x);
               m_waypoint_y = Units::MetersLength(y);
               crossed = true;
               break;
            }
         } else {
            // turn segment, get info from "previous" segment
            HorizontalTurnPath &turn((hpi - 1)->m_turn_info);
            double turn_center_distance = a * turn.x_position_meters + b * turn.y_position_meters - c;
            if (abs(turn_center_distance) <= turn.radius.value()) {
               // turn's circle intersects line, probably twice
               // But is it a crossing?
               /*
                * a*x + b*y = c
                * y = (c-a*x) / b
                * (x-xc)^2 + (y-yc)^2 = r^2
                */
               // coordinates of intersection points
               double xi1, yi1, xi2, yi2;
               if (abs(a) > abs(b)) {
                  // substitute for x
                  // x = c/a - (b/a)*y
                  // (y-yc)^2 + (xc - c/a + (b/a)*y)^2 - r^2 = 0
                  // y^2 - 2*y*yc + yc^2 + (xc-c/a)^2 + 2*(xc-c/a)*(b/a)*y + (b/a)^2 * y^2 - r^2 = 0
                  // y^2 + (b/a)^2 * y^2 - 2*y*yc + 2*(xc-c/a)*(b/a)*y + yc^2 + (xc-c/a)^2 - r^2 = 0
                  // quadratic formula for y
                  double qa = 1 + SQR(b / a);
                  double qb = -2 * turn.y_position_meters + 2 * (turn.x_position_meters - c / a) * (b / a);
                  double qc =
                        SQR(turn.y_position_meters) + SQR(turn.x_position_meters - c / a) - SQR(turn.radius.value());
                  double discriminantRoot = sqrt(qb * qb - 4 * qa * qc);
                  yi1 = (-qb + discriminantRoot) / (2 * qa);
                  yi2 = (-qb - discriminantRoot) / (2 * qa);
                  xi1 = c / a - (b / a) * yi1;
                  xi2 = c / a - (b / a) * yi2;
               } else {
                  // substitute for y
                  // y = c/b - (a/b)*x
                  // (x-xc)^2 + (yc - c/b + (a/b)*x)^2 - r^2 = 0
                  // x^2 - 2*x*xc + xc^2 + (yc-c/b)^2 + 2*(yc-c/b)*(a/b)*x + (a/b)^2 * x^2 - r^2 = 0
                  // x^2 + (a/b)^2 * x^2 - 2*x*xc + 2*(yc-c/b)*(a/b)*x + xc^2 + (yc-c/b)^2 - r^2 = 0
                  // quadratic formula for x
                  double qa = 1 + SQR(a / b);
                  double qb = -2 * turn.x_position_meters + 2 * (turn.y_position_meters - c / b) * (a / b);
                  double qc =
                        SQR(turn.x_position_meters) + SQR(turn.y_position_meters - c / b) - SQR(turn.radius.value());
                  double discriminantRoot = sqrt(qb * qb - 4 * qa * qc);
                  xi1 = (-qb + discriminantRoot) / (2 * qa);
                  xi2 = (-qb - discriminantRoot) / (2 * qa);
                  yi1 = c / b - (a / b) * xi1;
                  yi2 = c / b - (a / b) * xi2;
               }
               // Now we have (xi1,yi1) and (xi2,yi2) which are on the circle and on the line.
               // But are they within the arc, and if so, which one is last, chronologically?
               HorizontalTurnPath::TURN_DIRECTION turn_direction = turn.GetTurnDirection(*(hpi + 1), *hpi);
               Units::UnsignedRadiansAngle q1(atan2(yi1 - turn.y_position_meters, xi1 - turn.x_position_meters));
               Units::UnsignedRadiansAngle q2(atan2(yi2 - turn.y_position_meters, xi2 - turn.x_position_meters));
               bool p1_on_path(false), p2_on_path(false), p1_is_last(false);
               switch (turn_direction) {
                  case HorizontalTurnPath::TURN_DIRECTION::LEFT_TURN:
                     if (turn.q_end > turn.q_start) {
                        // no wrap
                        p1_on_path = (turn.q_start < q1) && (turn.q_end > q1);
                        p2_on_path = (turn.q_start < q2) && (turn.q_end > q2);
                     } else {
                        // wrap
                        // check end first
                        p1_on_path = (q1 < turn.q_end);
                        p2_on_path = (q2 < turn.q_end);

                        // check start if we don't have a winner yet
                        if (!(p1_on_path || p2_on_path)) {
                           p1_on_path = (q1 > turn.q_start);
                           p2_on_path = (q2 > turn.q_start);
                        }
                     }
                     if (p1_on_path && p2_on_path) {
                        p1_is_last = (q1 > q2);
                     }
                     break;
                  case HorizontalTurnPath::TURN_DIRECTION::RIGHT_TURN:
                     if (turn.q_end < turn.q_start) {
                        // no wrap
                        p1_on_path = (turn.q_start > q1) && (turn.q_end < q1);
                        p2_on_path = (turn.q_start > q2) && (turn.q_end < q2);
                     } else {
                        // wrap
                        // check end first
                        p1_on_path = (q1 > turn.q_end);
                        p2_on_path = (q2 > turn.q_end);

                        // check start if we don't have a winner yet
                        if (!(p1_on_path || p2_on_path)) {
                           p1_on_path = (q1 < turn.q_start);
                           p2_on_path = (q2 < turn.q_start);
                        }
                     }

                     if (p1_on_path && p2_on_path) {
                        p1_is_last = (q1 < q2);
                     }
                     break;
                  default:
                     throw new logic_error("This should be a turn.");
               }

               if (p1_on_path && (p1_is_last || !p2_on_path)) {
                  m_waypoint_x = Units::MetersLength(xi1);
                  m_waypoint_y = Units::MetersLength(yi1);
                  crossed = true;
                  break;
               } else if (p2_on_path) {
                  m_waypoint_x = Units::MetersLength(xi2);
                  m_waypoint_y = Units::MetersLength(yi2);
                  crossed = true;
                  break;
               }
            }  // end if line crosses turn circle

            // Sanity check:  If straight line would have intersected, turn must too.
            if (d0 * d1 < 0) {
               throw logic_error("Failed to calculate circular arc intersection.");
            }
         }  // end if turn
      }  // end if not first
      x0 = x1;
      y0 = y1;
      d0 = d1;
   }  // end HorizontalPath iteration

   if (!crossed) {
      if (end_is_usable) {
         // trajectory end is within 50m of the line, put the TRP there.
         LOG4CPLUS_WARN(m_logger,
                        "Target trajectory ends near ABP projection line, using end-of-trajectory as TRP ("
                              <<
                              m_waypoint_x
                              << ","
                              << m_waypoint_y
                              << ").");
      } else {
         string emsg = "Target trajectory never crosses line " +
                       std::to_string(a) + " * x + " + std::to_string(b) + " * y = " + std::to_string(c);
         LOG4CPLUS_FATAL(m_logger, emsg);
         throw logic_error(emsg);
      }
   }
}

void AchievePointCalcs::ComputePositions(const AircraftIntent &intent) {

   if (this->HasWaypoint()) {

      int ix = intent.GetWaypointIndexByName(m_waypoint_name);

      if (ix > -1) {
         m_waypoint_x = Units::MetersLength(intent.GetFms().m_x[ix]);
         m_waypoint_y = Units::MetersLength(intent.GetFms().m_y[ix]);
      } else {
         string emsg = "Illegal achieve point " + m_waypoint_name +
                       " encountered while computing achieve point calculations positions";
         LOG4CPLUS_FATAL(AchievePointCalcs::m_logger, emsg);
         throw logic_error(emsg);
      }
      LOG4CPLUS_DEBUG(m_logger, "Waypoint[" << ix << "] = " << m_waypoint_name <<
                                            "(" << m_waypoint_x << "," << m_waypoint_y << ")");
   }
}


void AchievePointCalcs::ComputeAlongPathDistanceFromWaypointToEnd() {

   if (this->HasWaypoint()) {
      Units::Length distance_from_waypoint_to_end;
      m_distance_calculator.CalculateAlongPathDistanceFromPosition(this->m_waypoint_x,
                                                                   this->m_waypoint_y,
                                                                   distance_from_waypoint_to_end);

      m_distance_from_waypoint = distance_from_waypoint_to_end;
      m_distance_calculator.InitializeStartingIndex(); // because we just messed up the index
   }

}


void AchievePointCalcs::ComputeEndValues(const VerticalPath &vertical_path) {

   if (this->HasWaypoint()) {
      // compute achieve by distance
      ComputeAlongPathDistanceFromWaypointToEnd();


      // compute time to go.
      if (m_distance_from_waypoint == Units::ZERO_LENGTH) {
         // achieve by at end of route-set to end of route.

         m_time_to_go_to_waypoint = Units::ZERO_TIME;
      } else {

         // compute time to go from achieve point to the end.

         // get index at achieve by distance from end of route (beginning of our search).

         int firsttimeix = vertical_path.time.size() - 1;

         int ix = firsttimeix;

         while ((m_distance_from_waypoint < Units::MetersLength(vertical_path.x[ix])) && ix > 0) {
            ix--;
         }


         // Compute the time to go from the achieve by point to the end.

         if (ix < firsttimeix) {

            // ttg:time to go at points 1 and 0.
            // dtg:distance to go at points 1 and 0.
            const double ttg0 = vertical_path.time[ix];
            const double ttg1 = vertical_path.time[ix + 1];
            const double dtg0 = vertical_path.x[ix];
            const double dtg1 = vertical_path.x[ix + 1];

            if (m_distance_from_waypoint == Units::MetersLength(dtg0)) {
               // Don't interpolate, take time at pt 0.
               m_time_to_go_to_waypoint = Units::SecondsTime(ttg0);
            } else if ((ix == 0) && (m_distance_from_waypoint < Units::MetersLength(dtg0))) {
               // Unlikely case: dtg of first descent point closer to final point
               // than waypoint.  Take time at point 0.
               m_time_to_go_to_waypoint = Units::SecondsTime(ttg0);
            } else {
               // Points valid for interpolation.
               m_time_to_go_to_waypoint = Units::SecondsTime((ttg1 - ttg0) /
                                                             (dtg1 - dtg0) *
                                                             (Units::MetersLength(m_distance_from_waypoint).value() -
                                                              dtg0) + ttg0);
            }

         } else {
            // Take time at last point (beginning of our search).
            m_time_to_go_to_waypoint = Units::SecondsTime(vertical_path.time[firsttimeix]);
         }
      }
   }
}

const bool AchievePointCalcs::IsWaypointPassed(const AircraftState &acstate) {
   Units::Length distance_to_end;
   m_distance_calculator.CalculateAlongPathDistanceFromPosition(Units::FeetLength(acstate.m_x),
                                                                Units::FeetLength(acstate.m_y),
                                                                distance_to_end);

   return (distance_to_end < m_distance_from_waypoint);
}

const Units::Length AchievePointCalcs::ComputeDistanceToWaypoint(const AircraftState &acstate) {
   Units::Length distance_to_end;
   m_distance_calculator.CalculateAlongPathDistanceFromPosition(Units::FeetLength(acstate.m_x),
                                                                Units::FeetLength(acstate.m_y),
                                                                distance_to_end);
   return distance_to_end - m_distance_from_waypoint;
}


void AchievePointCalcs::ComputeCrossingTime(const AircraftState &acstate) {
   /*
    * This implementation does not ensure crossing! It assumes the caller has already
    * calculated a state just after crossing the Achieve waypoint and simply
    * needs a time of crossing.
    */

   // NOTE:Only working with x,y position throughout; not setting z.
   AircraftState prevstate = acstate;
   prevstate.m_x -= acstate.m_xd;
   prevstate.m_y -= acstate.m_yd;


   // get achieve-by waypoint coordinates
   AircraftState endstate;
   endstate.m_x = Units::FeetLength(m_waypoint_x).value();
   endstate.m_y = Units::FeetLength(m_waypoint_y).value();

   // translate origin point to the previous aircraft state
   // translate end point
   AircraftState current_state = acstate;
   current_state.m_x -= prevstate.m_x;
   current_state.m_y -= prevstate.m_y;

   // translate last waypoint
   endstate.m_x -= prevstate.m_x;
   endstate.m_y -= prevstate.m_y;

   // translate previous position to origin
   prevstate.m_x -= prevstate.m_x;
   prevstate.m_y -= prevstate.m_y;

   // calculate the angle from the previous position to the final position
   double aircraftangle = atan2(current_state.m_y, current_state.m_x);

   // calculate the angle from the previous position to the end waypoint
   double waypointangle = atan2(endstate.m_y, endstate.m_x);

   // calculate the angle of the nearest angle
   double nearestangle = subtract_headings(aircraftangle, waypointangle);

   // calculate the distance between the previous aircraft point and end waypoint
   double distwaypoint = sqrt(pow(endstate.m_x, 2) + pow(endstate.m_y, 2));

   // calculate the distance between the previous aircraft point and the aircraft end point
   double distaircraft = sqrt(pow(current_state.m_x, 2) + pow(current_state.m_y, 2));

   // calculate distance to closest point between the previous and end point
   double distclosest = distwaypoint * cos(nearestangle);

   // calculates the ratio of distance to the closest point compared to the distance to the aircraft end point
   double ratio = distclosest / distaircraft;

   m_crossing_time = Units::SecondsTime(current_state.m_time - 1.0 + ratio);

}
