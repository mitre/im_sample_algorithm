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

#include "imalgs/IMClearanceLoader.h"
#include "utility/CustomUnits.h"
#include "loader/LoadError.h"

using namespace interval_management;
using namespace interval_management::open_source;

std::map<std::string, IMClearance::ClearanceType> IMClearanceLoader::m_clearance_type_dictionary = {
      {"CUSTOM", IMClearance::ClearanceType::CUSTOM},   {"CAPTURE", IMClearance::ClearanceType::CAPTURE},
      {"ACHIEVE", IMClearance::ClearanceType::ACHIEVE}, {"MAINTAIN", IMClearance::ClearanceType::MAINTAIN},
      {"FAS", IMClearance::ClearanceType::FAS},         {"NONE", IMClearance::ClearanceType::NONE}};

std::map<std::string, IMClearance::SpacingGoalType> IMClearanceLoader::m_assigned_spacing_goal_type_dictionary = {
      {"TIME", IMClearance::SpacingGoalType::TIME}, {"DIST", IMClearance::SpacingGoalType::DIST}};

log4cplus::Logger IMClearanceLoader::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("IMClearanceLoader"));

IMClearance IMClearanceLoader::m_empty_clearance;

IMClearanceLoader::IMClearanceLoader(void) {
   this->m_target_id = m_empty_clearance.GetTargetId();
   this->m_traffic_reference_point = m_empty_clearance.GetTrafficReferencePoint();
   this->m_achieve_by_point = m_empty_clearance.GetAchieveByPoint();
   this->m_planned_termination_point = m_empty_clearance.GetPlannedTerminationPoint();
   this->m_assigned_spacing_goal_type = m_empty_clearance.GetAssignedSpacingGoalType();
   this->m_assigned_spacing_goal = Units::SecondsTime(m_empty_clearance.GetAssignedTimeSpacingGoal()).value();
   this->m_clearance_type = m_empty_clearance.GetClearanceType();
   this->m_final_approach_spacing_merge_angle_mean = m_empty_clearance.GetFinalApproachSpacingMergeAngleMean();
   this->m_final_approach_spacing_merge_angle_std = m_empty_clearance.GetFinalApproachSpacingMergeAngleStd();
   this->m_is_vector_aircraft = m_empty_clearance.IsVectorAircraft();
   this->m_loaded = false;
}

IMClearanceLoader::~IMClearanceLoader(void) {}

bool IMClearanceLoader::load(DecodedStream *strm) {

   // Loads clearance data from input file and returns status.
   //
   // strm:input data stream.
   // returns true if load succeeds.
   //         false if load fails.

   set_stream(strm);

   // These parameters are not required, so default
   // to something which doesn't throw an error
   std::string cleartype(IMClearanceLoader::GetClearanceString(m_clearance_type));
   std::string goaltype(IMClearanceLoader::GetAssignedSpacingTypeString(m_assigned_spacing_goal_type));

   register_var("IM_Clearance_Type", &cleartype);
   register_var("Assigned_Spacing_Goal_Type", &goaltype, false);
   register_var("Assigned_Spacing_Goal", &this->m_assigned_spacing_goal, false);
   register_var("Target_Aircraft", &this->m_target_id, false);
   register_var("Traffic_Reference_Point", &this->m_traffic_reference_point);
   register_var("Achieve_by_Point", &this->m_achieve_by_point, false);
   register_var("Planned_Termination_Point", &this->m_planned_termination_point, false);
   register_var("FAS_merge_angle", &m_final_approach_spacing_merge_angle_mean, false);
   register_var("FAS_merge_angle_std", &m_final_approach_spacing_merge_angle_std, false);
   register_var("FAS_is_vectored", &m_is_vector_aircraft, false);
   register_loadable_with_brackets("target_intent", &m_target_aircraft_intent, false);
   register_loadable_with_brackets("ownship_intent", &m_ownship_aircraft_intent, false);

   m_loaded = complete();

   // Check for disallowed TRP value.
   if (m_traffic_reference_point == "CALCULATED_TRP") {
      std::string msg = "Invalid Traffic_Reference_Point:  " + m_traffic_reference_point;
      LOG4CPLUS_FATAL(m_logger, msg);
      throw LoadError(msg);
   }

   std::map<std::string, IMClearance::ClearanceType>::const_iterator it_clear =
         IMClearanceLoader::m_clearance_type_dictionary.find(cleartype);

   if (it_clear == IMClearanceLoader::m_clearance_type_dictionary.end()) {
      std::string msg = "Clearance type parameter not found: " + cleartype;
      LOG4CPLUS_FATAL(m_logger, msg);
      throw LoadError(msg);
   } else {
      this->m_clearance_type = it_clear->second;
   }

   std::map<std::string, IMClearance::SpacingGoalType>::const_iterator it_space =
         IMClearanceLoader::m_assigned_spacing_goal_type_dictionary.find(goaltype);

   if (it_space == IMClearanceLoader::m_assigned_spacing_goal_type_dictionary.end()) {
      std::string msg = "Assigned spacing goal type parameter not found: " + goaltype;
      LOG4CPLUS_FATAL(m_logger, msg);
      throw LoadError(msg);
   } else {
      this->m_assigned_spacing_goal_type = it_space->second;
   }

   return m_loaded;
}

const IMClearance IMClearanceLoader::BuildClearance(void) {

   // Builds clearance object.
   if (!m_loaded) return m_empty_clearance;

   if (m_target_aircraft_intent.IsLoaded()) {
      m_target_aircraft_intent.SetId(m_target_id);
   }

   IMClearance::Builder builder(m_clearance_type, m_target_id, m_target_aircraft_intent, m_achieve_by_point,
                                m_planned_termination_point, m_assigned_spacing_goal_type, m_assigned_spacing_goal);
   builder.TrafficReferencePoint(m_traffic_reference_point);
   if (m_ownship_aircraft_intent.IsLoaded()) builder.OwnshipIntent(m_ownship_aircraft_intent);
   if (m_clearance_type == IMClearance::ClearanceType::FAS) {
      builder.FinalApproachParameters(m_final_approach_spacing_merge_angle_mean,
                                      m_final_approach_spacing_merge_angle_std, m_is_vector_aircraft);
   }
   return builder.Build();
}

std::string IMClearanceLoader::GetClearanceString(const IMClearance::ClearanceType type) {
   // Gets clearance string associated with input clearance type.
   //
   // type:Input clearance type
   // returns clearance string.

   std::string str = "";

   for (std::map<std::string, IMClearance::ClearanceType>::iterator it =
              IMClearanceLoader::m_clearance_type_dictionary.begin();
        ((it != IMClearanceLoader::m_clearance_type_dictionary.end()) && (str == "")); ++it) {
      if (it->second == type) {
         str = it->first;
      }
   }

   return str;
}

std::string IMClearanceLoader::GetAssignedSpacingTypeString(const IMClearance::SpacingGoalType type) {
   // Gets assigned spacing goal type string associated with
   // assigned spacing goal type.
   //
   // type:Input assigned spacing goal type.
   // returns assigned spacing goal type string.
   std::string str = "";

   for (std::map<std::string, IMClearance::SpacingGoalType>::iterator it =
              IMClearanceLoader::m_assigned_spacing_goal_type_dictionary.begin();
        ((it != IMClearanceLoader::m_assigned_spacing_goal_type_dictionary.end()) && (str == "")); ++it) {
      if (it->second == type) {
         str = it->first;
      }
   }

   return str;
}
