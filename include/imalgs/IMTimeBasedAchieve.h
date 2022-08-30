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
#include "loader/Loadable.h"

class IMTimeBasedAchieve : public IMKinematicAchieve
{
public:

   IMTimeBasedAchieve();

   IMTimeBasedAchieve(const IMTimeBasedAchieve& obj);

   virtual ~IMTimeBasedAchieve();

   IMTimeBasedAchieve& operator=(const IMTimeBasedAchieve& obj);

   virtual void IterationReset();

   virtual aaesim::open_source::Guidance Update(const aaesim::open_source::Guidance& previous_im_guidance,
                           const aaesim::open_source::DynamicsState& three_dof_dynamics_state,
                           const interval_management::AircraftState& current_ownship_state,
                           const interval_management::AircraftState& current_target_state,
                           const vector<interval_management::AircraftState>& target_adsb_history);

   virtual const bool IsImOperationComplete() const;

   virtual const double GetAssignedSpacingGoal() const;

   virtual const Units::Speed GetImSpeedCommandIas() const;

   virtual const double GetSpacingInterval() const;

   virtual const double GetPsi() const;

   virtual const double GetMsi() const;

   virtual const double GetSpacingError() const;

   virtual int GetSpeedChangeCount() const;

   virtual void DumpParameters(const std::string& parameters_to_print);

   const interval_management::AircraftState GetTargetStateProjectedAsgAdjusted() const override;

   bool load(DecodedStream* input);

protected:
   virtual void CalculateIas(const Units::Length current_ownship_altitude,
                             const aaesim::open_source::DynamicsState& three_dof_dynamics_state);

   virtual void CalculateMach(const Units::Time reference_ttg,
                              const Units::Length current_ownship_altitude);

   virtual void RecordInternalObserverMetrics(const interval_management::AircraftState& current_ownship_state,
                                              const interval_management::AircraftState& current_target_state,
                                              const aaesim::open_source::DynamicsState& dynamics_state,
                                              const Units::Speed unmodified_ias,
                                              const Units::Speed tas_command,
                                              const Units::Speed reference_velocity,
                                              const Units::Length reference_distance,
                                              const aaesim::open_source::Guidance& guidance);


   void Copy(const IMTimeBasedAchieve& obj);

   virtual void SetAssignedSpacingGoal(const IMClearance& clearance);

   std::shared_ptr<IMKinematicTimeBasedMaintain> m_im_kinematic_time_based_maintain;

   Units::SecondsTime m_assigned_spacing_goal;

private:
   aaesim::open_source::Guidance HandleAchieveStage(const interval_management::AircraftState& current_ownship_state,
                               const interval_management::AircraftState& current_target_state,
                               const vector<interval_management::AircraftState>& target_adsb_history,
                               const aaesim::open_source::DynamicsState& three_dof_dynamics_state,
                               aaesim::open_source::Guidance& guidance_out);

   void TestForTrafficAlignment(const interval_management::AircraftState& current_ownship_state,
                                const std::vector<interval_management::AircraftState>& target_adsb_history);

   aaesim::open_source::Guidance HandleMaintainStage(const interval_management::AircraftState& current_ownship_state,
                                const interval_management::AircraftState& current_target_state,
                                const vector<interval_management::AircraftState>& target_adsb_history,
                                const aaesim::open_source::DynamicsState& three_dof_dynamics_state,
                                const aaesim::open_source::Guidance& previous_im_guidance,
                                aaesim::open_source::Guidance& guidance_out);

   const Units::Time CalculateMeasuredSpacingInterval(
         const interval_management::AircraftState &current_ownship_state,
         const interval_management::AircraftState &current_target_state);

   const bool HasOwnshipReachedTargetAlongPathPositionAtAlignment() const;

   const bool HasOwnshipReachedTargetAlongPathPositionAtCdtiInitiation() const;

   void SaveTargetStateAtTrafficAlignment(Units::Time ownship_current_time,
                                          const interval_management::AircraftState& target_state_at_traffic_alignment,
                                          const Units::Length target_dtg_at_alignment);

   interval_management::AircraftState m_target_state_at_traffic_alignment;
   interval_management::AircraftState m_target_state_at_cdti_initiate_signal_receipt;

   Units::Length m_target_dtg_at_traffic_alignment;
   Units::Length m_target_dtg_at_cdti_initiate_signal_receipt;

   Units::Time m_time_since_traffic_alignment;
   Units::Time m_time_at_traffic_alignment;
   Units::Time m_predicted_spacing_interval;
   Units::Time m_measured_spacing_interval;
   Units::Time m_cdti_initiate_signal_receipt_time;

   interval_management::AircraftState m_target_state_projected_asg_adjusted;

   static log4cplus::Logger m_logger;

};

inline const bool IMTimeBasedAchieve::IsImOperationComplete() const {
   return m_ownship_kinematic_dtg_to_ptp <= Units::zero();
}

inline int IMTimeBasedAchieve::GetSpeedChangeCount() const {
   return m_im_kinematic_time_based_maintain->GetSpeedChangeCount() + m_total_number_of_im_speed_changes;
}

inline const double IMTimeBasedAchieve::GetSpacingInterval() const {
   if (m_stage_of_im_operation == ACHIEVE) {
      return GetPsi();
   } else if (m_stage_of_im_operation == MAINTAIN) {
      return GetMsi();
   } else {
      return IMAlgorithm::GetSpacingInterval();
   }
}

inline const double IMTimeBasedAchieve::GetPsi() const {
   return Units::SecondsTime(m_predicted_spacing_interval).value();
}

inline const double IMTimeBasedAchieve::GetMsi() const {
   return Units::SecondsTime(m_measured_spacing_interval).value();
}

inline void IMTimeBasedAchieve::SetAssignedSpacingGoal(const IMClearance& clearance) {
   m_assigned_spacing_goal = clearance.GetAssignedTimeSpacingGoal();
}

inline const double IMTimeBasedAchieve::GetAssignedSpacingGoal() const {
   return Units::SecondsTime(m_assigned_spacing_goal).value();
}

inline const double IMTimeBasedAchieve::GetSpacingError() const {
   if (m_stage_of_im_operation == ACHIEVE) {
      return Units::SecondsTime(m_predicted_spacing_interval - m_assigned_spacing_goal).value();
   } else if (m_stage_of_im_operation == MAINTAIN) {
      return Units::SecondsTime(m_measured_spacing_interval - m_assigned_spacing_goal).value();
   } else {
      return -INFINITY;
   }
}

inline const bool IMTimeBasedAchieve::HasOwnshipReachedTargetAlongPathPositionAtAlignment() const {
   if (m_target_dtg_at_traffic_alignment != Units::Infinity()) {
      return m_ownship_kinematic_dtg_to_ptp <= m_target_dtg_at_traffic_alignment;
   }
   return false;
}

inline const bool IMTimeBasedAchieve::HasOwnshipReachedTargetAlongPathPositionAtCdtiInitiation() const {
   if (m_target_dtg_at_cdti_initiate_signal_receipt != Units::Infinity()) {
      return m_ownship_kinematic_dtg_to_ptp <= m_target_dtg_at_cdti_initiate_signal_receipt;
   }
   return false;
}

inline const interval_management::AircraftState IMTimeBasedAchieve::GetTargetStateProjectedAsgAdjusted() const {
   return m_target_state_projected_asg_adjusted;
}
