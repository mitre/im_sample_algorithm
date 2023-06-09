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
// 2022 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include "imalgs/IMKinematicAchieve.h"
#include "imalgs/TrueDistances.h"
#include "imalgs/IMKinematicDistBasedMaintain.h"

namespace interval_management {
namespace open_source {
class IMDistBasedAchieve : public IMKinematicAchieve {
  public:
   static const Units::Length DEFAULT_DISTANCE_BASED_ASSIGNED_SPACING_GOAL;

   IMDistBasedAchieve();

   IMDistBasedAchieve(const IMDistBasedAchieve &obj);

   virtual ~IMDistBasedAchieve();

   virtual void IterationReset();

   virtual void Initialize(const OwnshipPredictionParameters &ownship_prediction_parameters,
                           const AircraftIntent &ownship_aircraft_intent,
                           WeatherPrediction &weather_prediction) override;

   virtual aaesim::open_source::Guidance Update(
         const aaesim::open_source::Guidance &previous_im_guidance,
         const aaesim::open_source::DynamicsState &three_dof_dynamics_state,
         const interval_management::open_source::AircraftState &current_ownship_state,
         const interval_management::open_source::AircraftState &current_target_state,
         const std::vector<interval_management::open_source::AircraftState> &target_adsb_history);

   virtual void DumpParameters(const std::string &parameters_to_print);

   virtual const double GetSpacingError() const;

   virtual void SetAssignedSpacingGoal(const IMClearance &clearance);

   virtual const bool IsImOperationComplete() const;

   virtual const bool InAchieveStage() const;

   virtual int GetSpeedChangeCount() const;

   virtual const Units::Speed GetImSpeedCommandIas() const;

   virtual const double GetSpacingInterval() const;

   virtual const double GetPsi() const;

   virtual const double GetMsi() const;

   virtual const double GetAssignedSpacingGoal() const;

   const Units::Length GetTrueDtgToAchieveByPointUsingReferenceTime(const Units::Time reference_time);

   IMDistBasedAchieve &operator=(const IMDistBasedAchieve &obj);

   const interval_management::open_source::AircraftState GetTargetStateProjectedAsgAdjusted() const override;

   bool load(DecodedStream *input);

  protected:
   virtual void CalculateIas(const Units::Length current_ownship_altitude,
                             const aaesim::open_source::DynamicsState &three_dof_dynamics_state);

   virtual void CalculateMach(const Units::Time reference_ttg, const Units::Length current_ownship_altitude,
                              const Units::Mass current_mass);

   void Copy(const IMDistBasedAchieve &obj);

   void RecordData(aaesim::open_source::Guidance &im_guidance,
                   const interval_management::open_source::AircraftState &current_ownship_state,
                   const interval_management::open_source::AircraftState &current_target_state,
                   const aaesim::open_source::DynamicsState &three_dof_dynamics_state,
                   Units::Speed unmodified_im_speed_command_ias, Units::Speed im_speed_command_tas,
                   Units::Speed ownship_reference_true_airspeed, Units::Length ownship_reference_dtg_to_ptp,
                   Units::Time target_reference_ttg_to_trp);

   std::shared_ptr<interval_management::open_source::IMKinematicDistBasedMaintain> m_im_kinematic_dist_based_maintain;

  private:
   /**
    * Calculate the PSI. Multiple algorithms depending on the situation.
    * @param target_reference_ttg_to_trp
    * @return unitized PSI
    */
   const Units::MetersLength CalculatePredictedSpacingInterval(Units::Time target_reference_ttg_to_trp);

   std::shared_ptr<TrueDistances> m_true_distances;

   interval_management::open_source::AircraftState m_target_state_projected_on_ownships_path_at_adjusted_distance;
   Units::Length m_predicted_spacing_interval;
   Units::Length m_measured_spacing_interval;
   Units::Length m_assigned_spacing_goal;
   AlongPathDistanceCalculator m_distance_calculator_target_on_ownship_hpath;
   PositionCalculator m_position_calculator_target_on_ownship_hpath;

   static log4cplus::Logger m_logger;
};
inline const bool IMDistBasedAchieve::IsImOperationComplete() const { return IsOwnshipPassedPtp(); }

inline const Units::Length IMDistBasedAchieve::GetTrueDtgToAchieveByPointUsingReferenceTime(
      const Units::Time reference_time) {
   return m_true_distances->ComputeTrueDtgToPtpAtTime(reference_time) -
          m_ownship_kinematic_achieve_by_calcs.GetDistanceFromWaypoint();
}

inline const double IMDistBasedAchieve::GetAssignedSpacingGoal() const {
   return Units::NauticalMilesLength(m_assigned_spacing_goal).value();
}

inline const double IMDistBasedAchieve::GetSpacingError() const {
   if (m_stage_of_im_operation == ACHIEVE) {
      return Units::NauticalMilesLength(m_predicted_spacing_interval - m_assigned_spacing_goal).value();
   } else if (m_stage_of_im_operation == MAINTAIN) {
      return Units::NauticalMilesLength(m_measured_spacing_interval - m_assigned_spacing_goal).value();
   } else {
      return -INFINITY;
   }
}

inline void IMDistBasedAchieve::SetAssignedSpacingGoal(const IMClearance &clearance) {
   if (clearance.IsValid()) {
      m_assigned_spacing_goal = clearance.GetAssignedDistanceSpacingGoal();
   } else {
      std::string msg = "Clearance is invalid, could not set assigned spacing goal";
      LOG4CPLUS_FATAL(m_logger, msg);
      throw std::logic_error(msg);
   }
}

inline const Units::Speed IMDistBasedAchieve::GetImSpeedCommandIas() const {
   if (m_stage_of_im_operation == ACHIEVE) {
      return IMAlgorithm::GetImSpeedCommandIas();
   } else {
      return m_im_kinematic_dist_based_maintain->GetImSpeedCommandIas();
   }
}

inline int IMDistBasedAchieve::GetSpeedChangeCount() const {
   return m_im_kinematic_dist_based_maintain->GetSpeedChangeCount() + m_total_number_of_im_speed_changes;
}

inline const double IMDistBasedAchieve::GetSpacingInterval() const {
   if (m_stage_of_im_operation == ACHIEVE) {
      return GetPsi();
   } else if (m_stage_of_im_operation == MAINTAIN) {
      return GetMsi();
   } else {
      return IMAlgorithm::GetSpacingInterval();
   }
}

inline const double IMDistBasedAchieve::GetPsi() const {
   return Units::NauticalMilesLength(m_predicted_spacing_interval).value();
}

inline const double IMDistBasedAchieve::GetMsi() const {
   return Units::NauticalMilesLength(m_measured_spacing_interval).value();
}

inline const bool IMDistBasedAchieve::InAchieveStage() const {
   const bool has_sequenced_abp = m_ownship_kinematic_dtg_to_abp <= Units::zero();
   const bool has_target_sequenced_trp = IsTargetPassedTrp();
   if ((has_sequenced_abp == false) && (has_target_sequenced_trp == false)) {
      return true;
   }
   return !m_has_maintain_stage;  // only return false if a maintain stage exists
}

inline const interval_management::open_source::AircraftState IMDistBasedAchieve::GetTargetStateProjectedAsgAdjusted()
      const {
   return m_target_state_projected_on_ownships_path_at_adjusted_distance;
}
}  // namespace open_source
}  // namespace interval_management