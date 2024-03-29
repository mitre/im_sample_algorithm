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

/*
 * AchievePointCalcs
 */

#pragma once

#include <vector>
#include <string>
#include <scalar/Length.h>
#include "public/AircraftIntent.h"
#include "imalgs/AircraftState.h"
#include "public/VerticalPath.h"
#include "public/HorizontalPath.h"
#include "public/Logging.h"
#include "public/AlongPathDistanceCalculator.h"

namespace interval_management {
namespace open_source {
class AchievePointCalcs {

  public:
   AchievePointCalcs(void);

   AchievePointCalcs(const std::string &waypoint, const AircraftIntent &intent, const VerticalPath &vpath,
                     const std::vector<HorizontalPath> &htraj);

   /** Constructor for target, with enough info to calculate TRP */
   AchievePointCalcs(const std::string &waypoint, const AircraftIntent &intent, const VerticalPath &vpath,
                     const std::vector<HorizontalPath> &htraj, const AchievePointCalcs &ownship_calcs,
                     const AircraftIntent &ownship_intent);

   virtual ~AchievePointCalcs(void);

   /**
    * Compute waypoint distance from end of horizontal path (if there is an achieve waypoint).
    */
   void ComputeAlongPathDistanceFromWaypointToEnd();

   /**
    * Compute distance and time to go between waypoint and end using
    * vertical and horizontal path data.  Calculation is only made if a waypoint is set.
    *
    * @param vertical_path
    */
   void ComputeEndValues(const VerticalPath &vertical_path);

   /**
    * Interpolates to compute a time at which the waypoint was crossed assuming the incoming state is close to the
    * waypoint. The caller must provide a valid state.
    *
    * @param acstate
    */
   void ComputeCrossingTime(const interval_management::open_source::AircraftState &acstate);

   const Units::Length ComputeDistanceToWaypoint(const interval_management::open_source::AircraftState &acstate);

   const bool HasWaypoint() const;

   const bool IsWaypointPassed(const interval_management::open_source::AircraftState &acstate);

   const std::string &GetWaypointName() const;

   const std::pair<Units::Length, Units::Length> GetWaypointLocation() const;

   const Units::Length GetDistanceFromWaypoint() const;

   const Units::Time GetTimeToGoToWaypoint() const;

   const Units::Time GetCrossingTime() const;

   const bool IsWaypointSet() const;

   static void ComputeDefaultTRP(const AchievePointCalcs &ownship_calcs, const AircraftIntent &ownship_intent,
                                 const AircraftIntent &target_intent,
                                 const std::vector<HorizontalPath> &target_horizontal_path,
                                 Waypoint &traffic_reference_point, Units::Length &waypoint_x,
                                 Units::Length &waypoint_y, size_t &waypoint_index_in_target_intent);

  protected:
   void Clear();

   /**
    * Computes achieve waypoint position from intent waypoints if an achieve waypoint is set.
    *
    * @param intent
    */
   void ComputePositions(const AircraftIntent &intent);

   std::string m_waypoint_name;
   Units::MetersLength m_waypoint_x;
   Units::MetersLength m_waypoint_y;
   Units::Length m_distance_from_waypoint;
   Units::Time m_time_to_go_to_waypoint;
   Units::Time m_crossing_time;
   bool m_waypoint_is_set;
   std::vector<HorizontalPath> m_horizontal_path;

  private:
   static log4cplus::Logger m_logger;

   AlongPathDistanceCalculator m_distance_calculator;
};

inline const bool AchievePointCalcs::HasWaypoint() const { return m_waypoint_is_set; }

inline const std::string &AchievePointCalcs::GetWaypointName() const { return m_waypoint_name; }

inline const std::pair<Units::Length, Units::Length> AchievePointCalcs::GetWaypointLocation() const {
   return std::pair<Units::Length, Units::Length>(m_waypoint_x, m_waypoint_y);
}

inline const Units::Length AchievePointCalcs::GetDistanceFromWaypoint() const { return m_distance_from_waypoint; }

inline const Units::Time AchievePointCalcs::GetTimeToGoToWaypoint() const { return m_time_to_go_to_waypoint; }

inline const bool AchievePointCalcs::IsWaypointSet() const { return m_waypoint_is_set; }

inline const Units::Time AchievePointCalcs::GetCrossingTime() const { return m_crossing_time; }

}  // namespace open_source
}  // namespace interval_management