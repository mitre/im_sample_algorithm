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

#pragma once

#include <string>
#include <Time.h>
#include <Length.h>
#include <Speed.h>
#include "utility/Logging.h"
#include "public/AircraftIntent.h"
#include "imalgs/IMUtils.h"

class IMClearance
{
public:

   enum ClearanceType
   {
      NONE = -1,
      CUSTOM = 0,
      CAPTURE,
      MAINTAIN,
      ACHIEVE,
      FAS
   };

   enum SpacingGoalType
   {
      TIME = 0,
      DIST
   };

   IMClearance();

   virtual ~IMClearance();

   IMClearance(const ClearanceType &clearance_type,
               const int target_id,
               const std::string &traffic_reference_point,
               const std::string &achieve_by_point,
               const std::string &planned_termination_point,
               const SpacingGoalType &assigned_spacing_goal_type,
               const double assigned_spacing_goal,
               const Units::Speed planned_final_approach_speed);

   IMClearance(const ClearanceType &clearance_type,
               const int target_id,
               const std::string &traffic_reference_point,
               const std::string &achieve_by_point,
               const std::string &planned_termination_point,
               const SpacingGoalType &assigned_spacing_goal_type,
               const double assigned_spacing_goal,
               const Units::Speed planned_final_approach_speed,
               const Units::Angle final_approach_spacing_merge_angle,
               const Units::Angle final_approach_spacing_merge_angle_std,
               const bool fas_is_vector_aircraft);

   IMClearance(const IMClearance &obj);

   bool operator==(const IMClearance &obj);

   bool operator!=(const IMClearance &obj);

   bool Validate(const AircraftIntent &ownship_aircraft_intent,
                 const AircraftIntent &target_aircraft_intent,
                 const IMUtils::IMAlgorithmTypes im_algorithm_type);

   bool IsValid() const;

   const Units::Angle GetFinalApproachSpacingMergeAngleMean() const;

   ClearanceType GetClearanceType() const;

   int GetTargetId() const;

   const std::string &GetAchieveByPoint() const;

   SpacingGoalType GetAssignedSpacingGoalType() const;

   Units::Time GetAssignedTimeSpacingGoal() const;

   Units::Length GetAssignedDistanceSpacingGoal() const;

   Units::Speed GetPlannedFinalApproachSpeed() const;

   const std::string &GetPlannedTerminationPoint() const;

   const std::string &GetTrafficReferencePoint() const;

   const bool IsCoincidentRoutePairing() const;

   const bool AbpAndPtpAreColocated() const;

   bool IsVectorAircraft() const;

   void dump(std::string hdr) const;
   Units::Angle GetFinalApproachSpacingMergeAngleStd() const;
   void SetFinalApproachSpacingMergeAngleStd(
         Units::Angle final_approach_spacing_merge_angle_std);

protected:

   // Not const on purpose
   virtual bool ValidateTrafficReferencePoint(const AircraftIntent &ownship_aircraft_intent,
                                              const AircraftIntent &target_aircraft_intent,
                                              const IMUtils::IMAlgorithmTypes im_algorithm_type);

   static log4cplus::Logger m_logger;
private:

   // Developers: try to keep these validate methods as const.
   bool ValidateFinalApproachSpacingClearance(const AircraftIntent &ownship_aircraft_intent,
                                              const AircraftIntent &target_aircraft_intent,
                                              const IMUtils::IMAlgorithmTypes im_algorithm_type) const;

   bool ValidateBasicInputs(const AircraftIntent &ownship_aircraft_intent,
                            const AircraftIntent &target_aircraft_intent,
                            const IMUtils::IMAlgorithmTypes im_algorithm_type) const;

   bool ValidatePlannedTerminationPoint(const AircraftIntent &ownship_aircraft_intent,
                                        const AircraftIntent &target_aircraft_intent,
                                        const IMUtils::IMAlgorithmTypes im_algorithm_type) const;

   ClearanceType m_clearance_type;
   SpacingGoalType m_assigned_spacing_goal_type;

   Units::RadiansAngle m_final_approach_spacing_merge_angle_mean;
   Units::RadiansAngle m_final_approach_spacing_merge_angle_std;
   Units::Speed m_planned_final_approach_speed;

   std::string m_achieve_by_point;
   std::string m_planned_termination_point;
   std::string m_traffic_reference_point;

   double m_assigned_spacing_goal;
   int m_target_id;

   bool m_is_coincident_route_pairing;
   bool m_is_vector_aircraft;
   bool m_valid;
};

inline const bool IMClearance::AbpAndPtpAreColocated() const {
   return m_achieve_by_point == m_planned_termination_point;
}
