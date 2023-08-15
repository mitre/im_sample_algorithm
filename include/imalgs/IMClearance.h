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

#include <string>
#include <optional>

#include <scalar/Time.h>
#include <scalar/Length.h>
#include <scalar/Speed.h>
#include "public/Logging.h"
#include "public/AircraftIntent.h"
#include "imalgs/IMUtils.h"

namespace interval_management {
namespace open_source {
class IMClearance {
  public:
   enum ClearanceType { NONE = -1, CUSTOM = 0, CAPTURE, MAINTAIN, ACHIEVE, FAS };

   enum SpacingGoalType { TIME = 0, DIST };

   IMClearance();

   ~IMClearance() = default;

   class Builder {
     private:
      ClearanceType m_builder_clearance_type;
      SpacingGoalType m_builder_assigned_spacing_goal_type;
      AircraftIntent m_builder_target_aircraft_intent;
      AircraftIntent m_builder_ownship_intent;
      Units::RadiansAngle m_builder_final_approach_spacing_merge_angle_mean;
      Units::RadiansAngle m_builder_final_approach_spacing_merge_angle_std;
      std::string m_builder_achieve_by_point;
      std::string m_builder_planned_termination_point;
      std::string m_builder_traffic_reference_point;
      double m_builder_assigned_spacing_goal;
      int m_builder_target_id;
      bool m_builder_is_vector_aircraft;

     public:
      Builder(const ClearanceType &clearance_type, const int target_id, const AircraftIntent &target_intent,
              const std::string &achieve_by_point, const std::string &planned_termination_point,
              const SpacingGoalType &assigned_spacing_goal_type, const double assigned_spacing_goal)
         : m_builder_ownship_intent(),
           m_builder_final_approach_spacing_merge_angle_mean(0),
           m_builder_final_approach_spacing_merge_angle_std(0),
           m_builder_traffic_reference_point(),
           m_builder_is_vector_aircraft(false) {
         m_builder_clearance_type = clearance_type;
         m_builder_target_id = target_id;
         m_builder_target_aircraft_intent = target_intent;
         m_builder_achieve_by_point = achieve_by_point;
         m_builder_planned_termination_point = planned_termination_point;
         m_builder_assigned_spacing_goal_type = assigned_spacing_goal_type;
         m_builder_assigned_spacing_goal = assigned_spacing_goal;
      };
      ~Builder() = default;
      const IMClearance Build() const { return IMClearance(this); }
      Builder *OwnshipIntent(const AircraftIntent &ownship_intent) {
         m_builder_ownship_intent = ownship_intent;
         return this;
      };
      Builder *TrafficReferencePoint(const std::string traffic_reference_point) {
         m_builder_traffic_reference_point = traffic_reference_point;
         return this;
      };
      Builder *FinalApproachParameters(const Units::Angle merge_angle_mean, const Units::Angle merge_angle_std,
                                       const bool is_vector_aircraft) {
         m_builder_final_approach_spacing_merge_angle_mean = merge_angle_mean;
         m_builder_final_approach_spacing_merge_angle_std = merge_angle_std;
         m_builder_is_vector_aircraft = is_vector_aircraft;
         return this;
      };

      ClearanceType GetClearanceType() const { return m_builder_clearance_type; };
      SpacingGoalType GetSpacingGoalType() const { return m_builder_assigned_spacing_goal_type; };
      AircraftIntent GetTargetAircraftIntent() const { return m_builder_target_aircraft_intent; };
      AircraftIntent GetOwnshipAircraftIntent() const { return m_builder_ownship_intent; };
      Units::RadiansAngle GetMergeAngleMean() const { return m_builder_final_approach_spacing_merge_angle_mean; };
      Units::RadiansAngle GetMergeAngleStd() const { return m_builder_final_approach_spacing_merge_angle_std; };
      std::string GetAchieveByPoint() const { return m_builder_achieve_by_point; };
      std::string GetPlannedTerminationPoint() const { return m_builder_planned_termination_point; };
      std::string GetTrafficReferencePoint() const { return m_builder_traffic_reference_point; };
      double GetAssignedSpacingGoal() const { return m_builder_assigned_spacing_goal; };
      int GetTargetId() const { return m_builder_target_id; };
      bool IsVectorAircraft() const { return m_builder_is_vector_aircraft; };
   };

   IMClearance(const IMClearance &obj);

   bool operator==(const IMClearance &obj) const;

   bool operator!=(const IMClearance &obj) const;

   bool Validate(const AircraftIntent &ownship_aircraft_intent, const IMUtils::IMAlgorithmTypes im_algorithm_type);

   bool IsValid() const;

   const Units::Angle GetFinalApproachSpacingMergeAngleMean() const;

   ClearanceType GetClearanceType() const;

   int GetTargetId() const;

   const std::string &GetAchieveByPoint() const;

   SpacingGoalType GetAssignedSpacingGoalType() const;

   Units::Time GetAssignedTimeSpacingGoal() const;

   Units::Length GetAssignedDistanceSpacingGoal() const;

   const std::string &GetPlannedTerminationPoint() const;

   const std::string &GetTrafficReferencePoint() const;

   const bool IsCoincidentRoutePairing() const;

   const bool AbpAndPtpAreColocated() const;

   bool IsVectorAircraft() const;

   void dump(std::string hdr) const;

   Units::Angle GetFinalApproachSpacingMergeAngleStd() const;

   void SetFinalApproachSpacingMergeAngleStd(Units::Angle final_approach_spacing_merge_angle_std);

   const AircraftIntent &GetTargetAircraftIntent() const { return m_target_aircraft_intent; }

   const std::optional<AircraftIntent> GetOwnshipIntent() const {
      if (m_ownship_intent.IsLoaded()) return std::optional{m_ownship_intent};
      return {};
   }

  protected:
   virtual bool ValidateTrafficReferencePoint(const AircraftIntent &ownship_aircraft_intent,
                                              const IMUtils::IMAlgorithmTypes im_algorithm_type);

  private:
   static log4cplus::Logger m_logger;

   IMClearance(const IMClearance::Builder *builder);

   bool ValidateFinalApproachSpacingClearance(const AircraftIntent &ownship_aircraft_intent,
                                              const IMUtils::IMAlgorithmTypes im_algorithm_type) const;

   bool ValidateBasicInputs(const AircraftIntent &ownship_aircraft_intent,
                            const IMUtils::IMAlgorithmTypes im_algorithm_type) const;

   bool ValidatePlannedTerminationPoint(const AircraftIntent &ownship_aircraft_intent,
                                        const IMUtils::IMAlgorithmTypes im_algorithm_type) const;

   ClearanceType m_clearance_type;
   SpacingGoalType m_assigned_spacing_goal_type;
   AircraftIntent m_target_aircraft_intent;
   AircraftIntent m_ownship_intent;
   Units::RadiansAngle m_final_approach_spacing_merge_angle_mean;
   Units::RadiansAngle m_final_approach_spacing_merge_angle_std;
   std::string m_achieve_by_point;
   std::string m_planned_termination_point;
   std::string m_traffic_reference_point;
   double m_assigned_spacing_goal;
   int m_target_id;
   bool m_is_vector_aircraft;
   bool m_is_coincident_route_pairing;
   bool m_valid;
};

inline const bool IMClearance::AbpAndPtpAreColocated() const {
   return m_achieve_by_point == m_planned_termination_point;
}
}  // namespace open_source
}  // namespace interval_management