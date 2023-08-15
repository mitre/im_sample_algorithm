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

#include "imalgs/IMClearance.h"
#include "imalgs/IMClearanceLoader.h"
#include "utility/CustomUnits.h"
#include "loader/LoadError.h"
#include "imalgs/IMUtils.h"
#include <stdexcept>

using namespace std;
using namespace interval_management::open_source;

log4cplus::Logger IMClearance::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("IMClearance"));

IMClearance::IMClearance()
   : m_clearance_type(CUSTOM),
     m_assigned_spacing_goal_type(TIME),
     m_target_aircraft_intent(),
     m_ownship_intent(),
     m_final_approach_spacing_merge_angle_mean(0),
     m_final_approach_spacing_merge_angle_std(0),
     m_achieve_by_point(),
     m_planned_termination_point(),
     m_traffic_reference_point(),
     m_assigned_spacing_goal(-INFINITY),
     m_target_id(IMUtils::UNINITIALIZED_AIRCRAFT_ID),
     m_is_vector_aircraft(false),
     m_is_coincident_route_pairing(false),
     m_valid(false) {}

IMClearance::IMClearance(const IMClearance::Builder *builder) {
   m_clearance_type = builder->GetClearanceType();
   m_assigned_spacing_goal_type = builder->GetSpacingGoalType();
   m_target_aircraft_intent = builder->GetTargetAircraftIntent();
   m_ownship_intent = builder->GetOwnshipAircraftIntent();
   m_final_approach_spacing_merge_angle_mean = builder->GetMergeAngleMean();
   m_final_approach_spacing_merge_angle_std = builder->GetMergeAngleStd();
   m_achieve_by_point = builder->GetAchieveByPoint();
   m_planned_termination_point = builder->GetPlannedTerminationPoint();
   m_traffic_reference_point = builder->GetTrafficReferencePoint();
   m_assigned_spacing_goal = builder->GetAssignedSpacingGoal();
   m_target_id = builder->GetTargetId();
   m_is_vector_aircraft = builder->IsVectorAircraft();
   m_is_coincident_route_pairing = false;
}

IMClearance::IMClearance(const IMClearance &obj) { *this = obj; }

bool IMClearance::operator==(const IMClearance &obj) const {
   return ((m_valid == obj.m_valid) && (m_clearance_type == obj.m_clearance_type) && (m_target_id == obj.m_target_id) &&
           (m_traffic_reference_point == obj.m_traffic_reference_point) &&
           (m_achieve_by_point == obj.m_achieve_by_point) &&
           (m_assigned_spacing_goal_type == obj.m_assigned_spacing_goal_type) &&
           (m_assigned_spacing_goal == obj.m_assigned_spacing_goal) &&
           (m_planned_termination_point == obj.m_planned_termination_point) &&
           (m_final_approach_spacing_merge_angle_mean == obj.m_final_approach_spacing_merge_angle_mean) &&
           (m_final_approach_spacing_merge_angle_std == obj.m_final_approach_spacing_merge_angle_std) &&
           (m_is_vector_aircraft == obj.m_is_vector_aircraft) &&
           (m_is_coincident_route_pairing == obj.m_is_coincident_route_pairing) &&
           (m_target_aircraft_intent == obj.m_target_aircraft_intent) && (m_ownship_intent == obj.m_ownship_intent));
}

bool IMClearance::operator!=(const IMClearance &obj) const { return (!(operator==(obj))); }

bool IMClearance::Validate(const AircraftIntent &ownship_aircraft_intent,
                           const IMUtils::IMAlgorithmTypes im_algorithm_type) {
   if (im_algorithm_type == IMUtils::IMAlgorithmTypes::TESTSPEEDCONTROL ||
       im_algorithm_type == IMUtils::IMAlgorithmTypes::RTA ||
       im_algorithm_type == IMUtils::IMAlgorithmTypes::RTA_TOAC_NOT_IMALGORITHM ||
       im_algorithm_type == IMUtils::IMAlgorithmTypes::NONE) {
      m_valid = true;
      return m_valid;
   }

   static const IMClearance empty;
   if (*this == empty) {
      m_valid = false;
      std::string msg = "The clearance object is null. Something in the input is wrong.";
      LOG4CPLUS_FATAL(m_logger, msg);
      throw LoadError(msg);
   }

   if (im_algorithm_type == IMUtils::IMAlgorithmTypes::NONE && m_clearance_type == ClearanceType::NONE) {
      m_valid = true;
      return m_valid;
   }

   m_valid = ValidateBasicInputs(ownship_aircraft_intent, im_algorithm_type);

   if (m_valid) {
      // Determine if coincident. Must be done _after_ verifying that intent objects exist and _before_ the rest of the
      // checks
      const pair<int, int> firstCommonIndex =
            ownship_aircraft_intent.FindCommonWaypoint(m_target_aircraft_intent);  // returns -1 if no common waypoints
      m_is_coincident_route_pairing = firstCommonIndex.first >= 0;

      if (m_clearance_type == ClearanceType::ACHIEVE || m_clearance_type == ClearanceType::CUSTOM ||
          m_clearance_type == ClearanceType::FAS) {
         m_valid = ValidateTrafficReferencePoint(ownship_aircraft_intent, im_algorithm_type);
      }

      if (m_valid) {
         if (m_planned_termination_point.empty()) {
            m_valid = true;
            const string lastWptName =
                  ownship_aircraft_intent.GetWaypointName(ownship_aircraft_intent.GetNumberOfWaypoints() - 1);
            string infomsg = "The planned termination point is empty. It is forced to: " + lastWptName;
            LOG4CPLUS_INFO(m_logger, infomsg);
            m_planned_termination_point = lastWptName;
         } else {
            m_valid = ValidatePlannedTerminationPoint(ownship_aircraft_intent, im_algorithm_type);
         }
      }

      if (m_valid && m_clearance_type == ClearanceType::FAS) {
         m_valid = ValidateFinalApproachSpacingClearance(ownship_aircraft_intent, im_algorithm_type);
      }
   }

   return m_valid;
}

bool IMClearance::IsValid() const { return m_valid; }

IMClearance::ClearanceType IMClearance::GetClearanceType() const { return m_clearance_type; }

int IMClearance::GetTargetId() const { return m_target_id; }

const string &IMClearance::GetAchieveByPoint() const { return m_achieve_by_point; }

IMClearance::SpacingGoalType IMClearance::GetAssignedSpacingGoalType() const { return m_assigned_spacing_goal_type; }

Units::Time IMClearance::GetAssignedTimeSpacingGoal() const {
   if (m_assigned_spacing_goal_type != SpacingGoalType::TIME) {
      char msg[100];
      sprintf(msg, "Spacing goal type is %d", m_assigned_spacing_goal_type);
      LOG4CPLUS_FATAL(m_logger, msg);
      throw logic_error(msg);
   }
   return Units::SecondsTime(m_assigned_spacing_goal);
}

const string &IMClearance::GetPlannedTerminationPoint() const { return m_planned_termination_point; }

const string &IMClearance::GetTrafficReferencePoint() const { return m_traffic_reference_point; }

Units::Length IMClearance::GetAssignedDistanceSpacingGoal() const {
   if (m_assigned_spacing_goal_type != SpacingGoalType::DIST) {
      char msg[100];
      sprintf(msg, "Spacing goal type is %d", m_assigned_spacing_goal_type);
      LOG4CPLUS_FATAL(m_logger, msg);
      throw logic_error(msg);
   }
   return Units::NauticalMilesLength(m_assigned_spacing_goal);
}

void IMClearance::dump(string hdr) const {
   string clearstr = IMClearanceLoader::GetClearanceString(m_clearance_type);
   string spacingstr = IMClearanceLoader::GetAssignedSpacingTypeString(m_assigned_spacing_goal_type);

   LOG4CPLUS_DEBUG(IMClearance::m_logger, hdr);
   LOG4CPLUS_DEBUG(IMClearance::m_logger, "Clearance type           " << clearstr.c_str());
   LOG4CPLUS_DEBUG(IMClearance::m_logger, "Target id                " << m_target_id);
   LOG4CPLUS_DEBUG(IMClearance::m_logger, "Target reference pt      " << m_traffic_reference_point.c_str());
   LOG4CPLUS_DEBUG(IMClearance::m_logger, "Achieve Pt               " << m_achieve_by_point.c_str());
   LOG4CPLUS_DEBUG(IMClearance::m_logger, "Planned Termination Pt   " << m_planned_termination_point.c_str());
   LOG4CPLUS_DEBUG(IMClearance::m_logger, "Spacing goal type        " << spacingstr.c_str());
   LOG4CPLUS_DEBUG(IMClearance::m_logger, "Spacing goal             " << m_assigned_spacing_goal);
   LOG4CPLUS_DEBUG(
         IMClearance::m_logger,
         "Final Approach merge angle" << Units::DegreesAngle(m_final_approach_spacing_merge_angle_mean).value());
}

const bool IMClearance::IsCoincidentRoutePairing() const { return m_is_coincident_route_pairing; }

const Units::Angle IMClearance::GetFinalApproachSpacingMergeAngleMean() const {
   return m_final_approach_spacing_merge_angle_mean;
}

Units::Angle IMClearance::GetFinalApproachSpacingMergeAngleStd() const {
   return m_final_approach_spacing_merge_angle_std;
}

bool IMClearance::IsVectorAircraft() const { return m_is_vector_aircraft; }

bool IMClearance::ValidateFinalApproachSpacingClearance(const AircraftIntent &ownship_aircraft_intent,
                                                        const IMUtils::IMAlgorithmTypes im_algorithm_type) const {
   // AAES-632 changed the way FAS is handled.
   //   PTP does not have to be the same as TRP (parallel runway approaches)
   //   merge point only added to aircraft on-vector
   const bool isLastWptNameOwnship =
         ownship_aircraft_intent.GetWaypointName(ownship_aircraft_intent.GetNumberOfWaypoints() - 1) ==
         m_planned_termination_point;

   if (!isLastWptNameOwnship) {
      const string msg = "For FAS, the Planned Termination Point (" + m_planned_termination_point +
                         ") must equal Ownship final waypoint. Input file is malformed.";
      LOG4CPLUS_ERROR(m_logger, msg);
      throw LoadError(msg);
   }

   if (Units::RadiansAngle(m_final_approach_spacing_merge_angle_mean).value() == INFINITY) {
      const string msg = "For FAS, The merge angle was not specified in the clearance. Clearance is malformed.";
      LOG4CPLUS_FATAL(m_logger, msg);
      throw LoadError(msg);
   }

   if (m_is_vector_aircraft) {
      if (ownship_aircraft_intent.GetNumberOfWaypoints() < 2) {
         const string msg = "For FAS, The on-vector ownship intent is too small. It must be malformed.";
         ownship_aircraft_intent.DumpParms("ownship intent:");
         LOG4CPLUS_FATAL(m_logger, msg);
         throw LoadError(msg);
      }

      if (m_target_aircraft_intent.GetNumberOfWaypoints() != 2) {
         const string msg = "For FAS, The on-final target intent is not 2. It must be malformed.";
         m_target_aircraft_intent.DumpParms("target intent:");
         LOG4CPLUS_FATAL(m_logger, msg);
         throw LoadError(msg);
      }
   } else {
      if (ownship_aircraft_intent.GetNumberOfWaypoints() != 2) {
         const string msg = "For FAS, The on-final ownship intent is not 2. It must be malformed.";
         ownship_aircraft_intent.DumpParms("ownship intent:");
         LOG4CPLUS_FATAL(m_logger, msg);
         throw LoadError(msg);
      }

      if (m_target_aircraft_intent.GetNumberOfWaypoints() < 2) {
         const string msg = "For FAS, The on-vector target intent is too small. It must be malformed.";
         m_target_aircraft_intent.DumpParms("target intent:");
         LOG4CPLUS_FATAL(m_logger, msg);
         throw LoadError(msg);
      }
   }

   if (ownship_aircraft_intent.GetWaypointName(ownship_aircraft_intent.GetNumberOfWaypoints() - 1) !=
       m_achieve_by_point) {
      const string msg = "For FAS, The ABP must be the final waypoint and it is not. This intent is malformed.";
      LOG4CPLUS_FATAL(m_logger, msg);
      throw LoadError(msg);
   }

   if (im_algorithm_type != IMUtils::IMAlgorithmTypes::TIMEBASEDACHIEVE &&
       im_algorithm_type != IMUtils::IMAlgorithmTypes::DISTANCEBASEDACHIEVE) {
      const string msg = "For FAS, The application_type is incorrect. Fix your input file.";
      LOG4CPLUS_FATAL(m_logger, msg);
      throw LoadError(msg);
   }

   return true;
}

bool IMClearance::ValidateBasicInputs(const AircraftIntent &ownship_aircraft_intent,
                                      const IMUtils::IMAlgorithmTypes im_algorithm_type) const {
   /*
    * NOTE: All statements in here should throw when a check fails.
    */
   if (im_algorithm_type == IMUtils::IMAlgorithmTypes::RTA ||
       im_algorithm_type == IMUtils::IMAlgorithmTypes::TESTSPEEDCONTROL ||
       im_algorithm_type == IMUtils::IMAlgorithmTypes::NONE)
      return true;

   if (ownship_aircraft_intent.GetNumberOfWaypoints() <= 0) {
      const std::string msg = "The ownship intent contains no waypoints.";
      LOG4CPLUS_FATAL(m_logger, msg);
      throw LoadError(msg);
   }

   const bool reqsintent = (im_algorithm_type == IMUtils::IMAlgorithmTypes::DISTANCEBASEDACHIEVE) ||
                           (im_algorithm_type == IMUtils::IMAlgorithmTypes::TIMEBASEDACHIEVE) ||
                           (im_algorithm_type == IMUtils::IMAlgorithmTypes::TIMEBASEDACHIEVEMUTABLEASG) ||
                           (im_algorithm_type == IMUtils::IMAlgorithmTypes::KINETICTARGETACHIEVE);
   if (reqsintent && m_target_aircraft_intent.GetNumberOfWaypoints() <= 0) {
      std::string msg = "The target intent contains no waypoints.";
      LOG4CPLUS_FATAL(m_logger, msg);
      throw LoadError(msg);
   }

   if (im_algorithm_type == IMUtils::TIMEBASEDACHIEVEMUTABLEASG && m_clearance_type != IMClearance::CUSTOM) {
      std::string msg = "This application type cannot be combined with a non-CUSTOM clearance type.";
      LOG4CPLUS_FATAL(m_logger, msg);
      throw LoadError(msg);
   }

   if (m_assigned_spacing_goal < 0) {
      const std::string msg = "The ASG value is invalid.";
      LOG4CPLUS_FATAL(m_logger, msg);
      throw LoadError(msg);
   }

   bool invalidCombo1 =
         m_assigned_spacing_goal_type == TIME && im_algorithm_type == IMUtils::IMAlgorithmTypes::DISTANCEBASEDACHIEVE;
   bool invalidCombo2 =
         m_assigned_spacing_goal_type == DIST && im_algorithm_type != IMUtils::IMAlgorithmTypes::DISTANCEBASEDACHIEVE;
   if (invalidCombo1 || invalidCombo2) {
      const std::string msg = "The application_type does not match the ASG Type. Check your input file.";
      LOG4CPLUS_FATAL(m_logger, msg);
      throw LoadError(msg);
   }

   return true;
}

void IMClearance::SetFinalApproachSpacingMergeAngleStd(Units::Angle final_approach_spacing_merge_angle_std) {
   m_final_approach_spacing_merge_angle_std = final_approach_spacing_merge_angle_std;
}

bool IMClearance::ValidatePlannedTerminationPoint(const AircraftIntent &ownship_aircraft_intent,
                                                  const IMUtils::IMAlgorithmTypes im_algorithm_type) const {

   if (ownship_aircraft_intent.GetWaypointIndexByName(m_planned_termination_point) < 0) {
      const string msg =
            "The planned termination point, " + m_planned_termination_point + ", was not found in the ownship intent.";
      LOG4CPLUS_FATAL(m_logger, msg);
      ownship_aircraft_intent.DumpParms("Own Intent follows:");
      throw LoadError(msg);
   }

   return true;
}

bool IMClearance::ValidateTrafficReferencePoint(const AircraftIntent &ownship_aircraft_intent,
                                                const IMUtils::IMAlgorithmTypes im_algorithm_type) {
   if (m_is_coincident_route_pairing) {
      if (!m_achieve_by_point.empty()) {
         m_traffic_reference_point = m_achieve_by_point;
      } else {
         std::string msg =
               "No Achieve_By_Point has been provided and there is no rule for a default. The scenario must provide "
               "the ABP.";
         LOG4CPLUS_FATAL(m_logger, msg);
         throw LoadError(msg);
      }
   } else {
      if (m_traffic_reference_point.empty()) {
         std::string msg =
               "This is a non-coincident scenario and no Traffic-Reference-Point has been provided.  TRP will be "
               "calculated.";
         LOG4CPLUS_INFO(m_logger, msg);
         // throw LoadError(msg);
         m_traffic_reference_point = "CALCULATED_TRP";
      } else {
         const int idx = m_target_aircraft_intent.GetWaypointIndexByName(m_traffic_reference_point);
         if (idx < 0) {
            std::string msg = "The Traffic-Reference-Point has not been found in the target intent. Looked for " +
                              m_traffic_reference_point + ".";
            LOG4CPLUS_FATAL(m_logger, msg);
            m_target_aircraft_intent.DumpParms("Target Intent");
            throw LoadError(msg);
         }
      }
   }
   return true;
}
