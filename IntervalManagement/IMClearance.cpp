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

#include "imalgs/IMClearance.h"
#include "imalgs/IMClearanceLoader.h"
#include "utility/CustomUnits.h"
#include "loader/LoadError.h"
#include "imalgs/IMUtils.h"
#include <stdexcept>

using namespace std;

log4cplus::Logger IMClearance::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("IMClearance"));


IMClearance::IMClearance()
      : m_final_approach_spacing_merge_angle_mean(0.0),
        m_achieve_by_point(""),
        m_planned_termination_point(""),
        m_traffic_reference_point(""),
        m_is_vector_aircraft(false) {
   m_valid = false;
   m_clearance_type = CUSTOM;
   m_target_id = IMUtils::UNINITIALIZED_AIRCRAFT_ID;
   m_assigned_spacing_goal_type = TIME;
   m_assigned_spacing_goal = -INFINITY;
   m_planned_final_approach_speed = Units::ZERO_SPEED;
   m_is_coincident_route_pairing = false;
}

IMClearance::~IMClearance() {
}

IMClearance::IMClearance(const ClearanceType &clearance_type,
                         const int target_id,
                         const string &traffic_reference_point,
                         const string &achieve_by_point,
                         const string &planned_termination_point,
                         const SpacingGoalType &assigned_spacing_goal_type,
                         const double assigned_spacing_goal,
                         const Units::Speed planned_final_approach_speed)
      : m_final_approach_spacing_merge_angle_mean(0.0),
        m_final_approach_spacing_merge_angle_std(0.0),
        m_is_vector_aircraft(false) {
   m_valid = false;

   m_clearance_type = clearance_type;
   m_target_id = target_id;
   m_traffic_reference_point = traffic_reference_point;
   m_achieve_by_point = achieve_by_point;
   m_planned_termination_point = planned_termination_point;
   m_assigned_spacing_goal_type = assigned_spacing_goal_type;
   m_assigned_spacing_goal = assigned_spacing_goal;
   m_planned_final_approach_speed = planned_final_approach_speed;
}

IMClearance::IMClearance(const IMClearance &obj) {
   *this = obj;
}

bool IMClearance::operator==(const IMClearance &obj) {
   return ((m_valid == obj.m_valid) &&
           (m_clearance_type == obj.m_clearance_type) &&
           (m_target_id == obj.m_target_id) &&
           (m_traffic_reference_point == obj.m_traffic_reference_point) &&
           (m_achieve_by_point == obj.m_achieve_by_point) &&
           (m_assigned_spacing_goal_type == obj.m_assigned_spacing_goal_type) &&
           (m_assigned_spacing_goal == obj.m_assigned_spacing_goal) &&
           (m_planned_termination_point == obj.m_planned_termination_point) &&
           (m_planned_final_approach_speed == obj.m_planned_final_approach_speed) &&
           (m_final_approach_spacing_merge_angle_mean == obj.m_final_approach_spacing_merge_angle_mean) &&
           (m_final_approach_spacing_merge_angle_std == obj.m_final_approach_spacing_merge_angle_std) &&
           (m_is_vector_aircraft == obj.m_is_vector_aircraft));

}

bool IMClearance::operator!=(const IMClearance &obj) {
   return (!(operator==(obj)));
}

bool IMClearance::Validate(const AircraftIntent &ownship_aircraft_intent,
                           const AircraftIntent &target_aircraft_intent,
                           const IMUtils::IMAlgorithmTypes im_algorithm_type) {
   {
      if (im_algorithm_type == IMUtils::IMAlgorithmTypes::TESTSPEEDCONTROL) {
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
   }

   m_valid = ValidateBasicInputs(ownship_aircraft_intent, target_aircraft_intent, im_algorithm_type);

   if (m_valid) {
      // Determine if coincident. Must be done _after_ verifying that intent objects exist and _before_ the rest of the checks
      const pair<int, int> firstCommonIndex = ownship_aircraft_intent.FindCommonWaypoint(
            target_aircraft_intent); // returns -1 if no common waypoints
      m_is_coincident_route_pairing = firstCommonIndex.first >= 0;

      if (m_clearance_type == ClearanceType::ACHIEVE ||
          m_clearance_type == ClearanceType::CUSTOM ||
          m_clearance_type == ClearanceType::FAS) {
         m_valid = ValidateTrafficReferencePoint(ownship_aircraft_intent, target_aircraft_intent, im_algorithm_type);
      }

      if (m_valid) {
         if (m_planned_termination_point.empty()) {
            m_valid = true;
            const string lastWptName = ownship_aircraft_intent.GetWaypointName(
                  ownship_aircraft_intent.GetNumberOfWaypoints() - 1);
            string infomsg = "The planned termination point is empty. It is forced to: " + lastWptName;
            LOG4CPLUS_INFO(m_logger, infomsg);
            m_planned_termination_point = lastWptName;
         } else {
            m_valid = ValidatePlannedTerminationPoint(ownship_aircraft_intent, target_aircraft_intent,
                                                      im_algorithm_type);
         }
      }

      if (m_valid && m_clearance_type == ClearanceType::FAS) {
         m_valid = ValidateFinalApproachSpacingClearance(ownship_aircraft_intent, target_aircraft_intent,
                                                         im_algorithm_type);
      }
   }

   return m_valid;
}

bool IMClearance::IsValid() const {
   return m_valid;
}

IMClearance::ClearanceType IMClearance::GetClearanceType() const {
   return m_clearance_type;
}

int IMClearance::GetTargetId() const {
   return m_target_id;
}

const string &IMClearance::GetAchieveByPoint() const {
   return m_achieve_by_point;
}

Units::Speed IMClearance::GetPlannedFinalApproachSpeed() const {
   return m_planned_final_approach_speed;
}

IMClearance::SpacingGoalType IMClearance::GetAssignedSpacingGoalType() const {
   return m_assigned_spacing_goal_type;
}

Units::Time IMClearance::GetAssignedTimeSpacingGoal() const {
   if (m_assigned_spacing_goal_type != SpacingGoalType::TIME) {
      char msg[100];
      sprintf(msg, "Spacing goal type is %d", m_assigned_spacing_goal_type);
      LOG4CPLUS_FATAL(m_logger, msg);
      throw logic_error(msg);
   }
   return Units::SecondsTime(m_assigned_spacing_goal);
}

const string &IMClearance::GetPlannedTerminationPoint() const {
   return m_planned_termination_point;
}

const string &IMClearance::GetTrafficReferencePoint() const {
   return m_traffic_reference_point;
}

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
   LOG4CPLUS_DEBUG(IMClearance::m_logger,
                   "Final Approach speed     " << Units::KnotsSpeed(m_planned_final_approach_speed).value());
   LOG4CPLUS_DEBUG(IMClearance::m_logger, "Final Approach merge angle"
         << Units::DegreesAngle(m_final_approach_spacing_merge_angle_mean).value());
}

const bool IMClearance::IsCoincidentRoutePairing() const {
   return m_is_coincident_route_pairing;
}

const Units::Angle IMClearance::GetFinalApproachSpacingMergeAngleMean() const {
   return m_final_approach_spacing_merge_angle_mean;
}

Units::Angle IMClearance::GetFinalApproachSpacingMergeAngleStd() const {
   return m_final_approach_spacing_merge_angle_std;
}

IMClearance::IMClearance(const IMClearance::ClearanceType &clearance_type,
                         const int target_id,
                         const std::string &traffic_reference_point,
                         const std::string &achieve_by_point,
                         const std::string &planned_termination_point,
                         const IMClearance::SpacingGoalType &assigned_spacing_goal_type,
                         const double assigned_spacing_goal,
                         const Units::Speed planned_final_approach_speed,
                         const Units::Angle final_approach_spacing_merge_angle_mean,
                         const Units::Angle final_approach_spacing_merge_angle_std,
                         const bool fas_is_vector_aircraft) {
   m_valid = false;

   m_clearance_type = clearance_type;
   m_target_id = target_id;
   m_traffic_reference_point = traffic_reference_point;
   m_achieve_by_point = achieve_by_point;
   m_planned_termination_point = planned_termination_point;
   m_assigned_spacing_goal_type = assigned_spacing_goal_type;
   m_assigned_spacing_goal = assigned_spacing_goal;
   m_planned_final_approach_speed = planned_final_approach_speed;
   m_final_approach_spacing_merge_angle_mean = final_approach_spacing_merge_angle_mean;
   m_final_approach_spacing_merge_angle_std = final_approach_spacing_merge_angle_std;
   m_is_vector_aircraft = fas_is_vector_aircraft;

}

bool IMClearance::IsVectorAircraft() const {
   return m_is_vector_aircraft;
}

bool IMClearance::ValidateFinalApproachSpacingClearance(const AircraftIntent &ownship_aircraft_intent,
                                                        const AircraftIntent &target_aircraft_intent,
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

      if (target_aircraft_intent.GetNumberOfWaypoints() != 2) {
         const string msg = "For FAS, The on-final target intent is not 2. It must be malformed.";
         target_aircraft_intent.DumpParms("target intent:");
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

      if (target_aircraft_intent.GetNumberOfWaypoints() < 2) {
         const string msg = "For FAS, The on-vector target intent is too small. It must be malformed.";
         target_aircraft_intent.DumpParms("target intent:");
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
                                      const AircraftIntent &target_aircraft_intent,
                                      const IMUtils::IMAlgorithmTypes im_algorithm_type) const {
   /*
    * NOTE: All statements in here should throw when a check fails.
    */
   if (ownship_aircraft_intent.GetNumberOfWaypoints() <= 0) {
      const std::string msg = "The ownship intent contains no waypoints.";
      LOG4CPLUS_FATAL(m_logger, msg);
      throw LoadError(msg);
   }

   const bool reqsintent = (im_algorithm_type == IMUtils::IMAlgorithmTypes::DISTANCEBASEDACHIEVE) ||
                           (im_algorithm_type == IMUtils::IMAlgorithmTypes::TIMEBASEDACHIEVE) ||
                           (im_algorithm_type == IMUtils::IMAlgorithmTypes::TIMEBASEDACHIEVEMUTABLEASG) ||
                           (im_algorithm_type == IMUtils::IMAlgorithmTypes::KINETICTARGETACHIEVE);
   if (reqsintent && target_aircraft_intent.GetNumberOfWaypoints() <= 0) {
      std::string msg = "The target intent contains no waypoints.";
      LOG4CPLUS_FATAL(m_logger, msg);
      throw LoadError(msg);
   }

   if (im_algorithm_type == IMUtils::TIMEBASEDACHIEVEMUTABLEASG && m_clearance_type != IMClearance::CUSTOM) {
      std::string msg = "This application type cannot be combined with a non-CUSTOM clearance type.";
      LOG4CPLUS_FATAL(m_logger, msg);
      throw LoadError(msg);
   }

   if (im_algorithm_type != IMUtils::IMAlgorithmTypes::NONE &&
       m_assigned_spacing_goal < 0) {  // default ASG is -inf, so anything greater than zero looks good
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

void IMClearance::SetFinalApproachSpacingMergeAngleStd(
      Units::Angle final_approach_spacing_merge_angle_std) {
   m_final_approach_spacing_merge_angle_std =
         final_approach_spacing_merge_angle_std;
}

bool IMClearance::ValidatePlannedTerminationPoint(const AircraftIntent &ownship_aircraft_intent,
                                                  const AircraftIntent &target_aircraft_intent,
                                                  const IMUtils::IMAlgorithmTypes im_algorithm_type) const {

   if (ownship_aircraft_intent.GetWaypointIndexByName(m_planned_termination_point) < 0) {
      const string msg = "The planned termination point, " + m_planned_termination_point
                         + ", was not found in the ownship intent.";
      LOG4CPLUS_FATAL(m_logger, msg);
      ownship_aircraft_intent.DumpParms("Own Intent follows:");
      throw LoadError(msg);
   }

   return true;
}

bool IMClearance::ValidateTrafficReferencePoint(const AircraftIntent &ownship_aircraft_intent,
                                                const AircraftIntent &target_aircraft_intent,
                                                const IMUtils::IMAlgorithmTypes im_algorithm_type) {
   if (m_is_coincident_route_pairing) {
      if (!m_achieve_by_point.empty()) {
         m_traffic_reference_point = m_achieve_by_point;
      } else {
         std::string msg = "No Achieve_By_Point has been provided and there is no rule for a default. The scenario must provide the ABP.";
         LOG4CPLUS_FATAL(m_logger, msg);
         throw LoadError(msg);
      }
   } else {
      if (m_traffic_reference_point.empty()) {
         std::string msg = "This is a non-coincident scenario and no Traffic-Reference-Point has been provided.  TRP will be calculated.";
         LOG4CPLUS_INFO(m_logger, msg);
         //throw LoadError(msg);
         m_traffic_reference_point = "CALCULATED_TRP";
      } else {
         const int idx = target_aircraft_intent.GetWaypointIndexByName(m_traffic_reference_point);
         if (idx < 0) {
            std::string msg = "The Traffic-Reference-Point has not been found in the target intent. Looked for " +
                              m_traffic_reference_point + ".";
            LOG4CPLUS_FATAL(m_logger, msg);
            target_aircraft_intent.DumpParms("Target Intent");
            throw LoadError(msg);
         }
      }
   }
   return true;
}
