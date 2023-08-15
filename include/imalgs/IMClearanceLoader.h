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

#include "loader/Loadable.h"
#include "imalgs/IMClearance.h"
#include "public/Logging.h"
#include <map>

namespace interval_management {

class IMClearanceLoader : public Loadable {
  public:
   IMClearanceLoader();

   virtual ~IMClearanceLoader();

   const interval_management::open_source::IMClearance BuildClearance();

   static std::string GetClearanceString(const interval_management::open_source::IMClearance::ClearanceType type);

   static std::string GetAssignedSpacingTypeString(
         const interval_management::open_source::IMClearance::SpacingGoalType type);

   bool load(DecodedStream *strm);

  private:
   static std::map<std::string, interval_management::open_source::IMClearance::ClearanceType>
         m_clearance_type_dictionary;
   static std::map<std::string, interval_management::open_source::IMClearance::SpacingGoalType>
         m_assigned_spacing_goal_type_dictionary;
   static interval_management::open_source::IMClearance m_empty_clearance;

   interval_management::open_source::IMClearance::ClearanceType m_clearance_type;
   interval_management::open_source::IMClearance::SpacingGoalType m_assigned_spacing_goal_type;

   std::string m_traffic_reference_point;
   std::string m_achieve_by_point;
   std::string m_planned_termination_point;

   Units::DegreesAngle m_final_approach_spacing_merge_angle_mean;
   Units::DegreesAngle m_final_approach_spacing_merge_angle_std;

   double m_assigned_spacing_goal;
   int m_target_id;
   AircraftIntent m_target_aircraft_intent;
   AircraftIntent m_ownship_aircraft_intent;

   bool m_loaded;
   static log4cplus::Logger m_logger;

   bool m_is_vector_aircraft;
};
}  // namespace interval_management