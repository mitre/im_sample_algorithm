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

#include "imalgs/IMAlgorithm.h"
#include "Frequency.h"
#include "aaesim/PredictedWindEvaluator.h"

class IMAchieve : public IMAlgorithm
{


public:
   static const Units::HertzFrequency ACHIEVE_CONTROL_GAIN_DEFAULT;
   static const Units::HertzFrequency MAINTAIN_CONTROL_GAIN_DEFAULT;
   static const Units::SecondsTime TIME_THRESHOLD_DEFAULT;
   static const Units::Angle TOLERANCE_ANGLE;
   static const Units::Speed TOLERANCE_SPEED;
   static const bool THRESHOLD_FLAG_DEFAULT;


   IMAchieve();

   IMAchieve(const IMAchieve &obj);

   virtual ~IMAchieve();

   virtual void IterationReset();

   virtual Guidance Update(const Guidance &prevguidance,
                           const DynamicsState &dynamicsstate,
                           const AircraftState &owntruthstate,
                           const AircraftState &targettruthstate,
                           const vector<AircraftState> &targethistory);

   virtual void ResetDefaults();

   virtual void DumpParameters(const std::string &parameters_to_print);

   virtual const bool IsTargetPassedTrp() const;

   virtual const bool IsTargetPassedLastWaypoint() const;

   virtual const bool InAchieveStage() const;

   void SetThresholdFlag(bool threshold_flag);

   const bool GetThresholdFlag() const;

   void SetTimeThreshold(Units::Time time_threshold);

   const Units::Length GetTargetKinematicDtgToTrp() const;

   const bool IsWithinErrorThreshold() const;

protected:

   void Copy(const IMAchieve &obj);

   virtual void CalculateIas(const Units::Length current_ownship_altitude,
                             const DynamicsState &three_dof_dynamics_state);

   virtual void CalculateMach(const Units::Time reference_ttg,
                              const Units::Length current_ownship_altitude);

   virtual const bool IsOwnshipBelowTransitionAltitude(Units::Length current_ownship_altitude);

   virtual void RecordInternalObserverMetrics(const AircraftState &current_ownship_state,
                                              const AircraftState &current_target_state,
                                              const DynamicsState &dynamics_state,
                                              const Units::Speed unmodified_ias,
                                              const Units::Speed tas_command,
                                              const Units::Speed reference_velocity,
                                              const Units::Length reference_distance,
                                              const Guidance &guidance);

   const bool WithinErrorThreshold(const Units::Length distance_to_go,
                                   const Units::Time ownship_ttg,
                                   const Units::Time reference_ttg);

   Units::Time GetErrorThreshold(Units::Length distance_to_go);

   static const std::shared_ptr<PredictedWindEvaluator> m_predicted_wind_evaluator;

   Units::Frequency m_achieve_control_gain;
   Units::Frequency m_maintain_control_gain;
   Units::Time m_time_threshold;

   bool m_threshold_flag;
   bool m_transitioned_to_maintain;
   bool m_within_error_threshold;
   bool m_received_one_valid_target_state;

private:
   void IterClearIMAch();
   bool m_is_target_aligned;

   static log4cplus::Logger m_logger;
};

inline const bool IMAchieve::InAchieveStage() const {
   return m_ownship_kinematic_dtg_to_abp > Units::zero();
}

inline const bool IMAchieve::IsTargetPassedTrp() const {
   return m_target_kinematic_dtg_to_trp <= Units::zero();
}

inline const bool IMAchieve::IsTargetPassedLastWaypoint() const {
   return m_target_kinematic_dtg_to_last_waypoint <= Units::zero();
}

inline const Units::Length IMAchieve::GetTargetKinematicDtgToTrp() const {
   return m_target_kinematic_dtg_to_trp;
}

inline const bool IMAchieve::IsWithinErrorThreshold() const {
   return m_within_error_threshold;
}

inline const bool IMAchieve::IsOwnshipBelowTransitionAltitude(Units::Length current_ownship_altitude) {
   return false;
}


inline void IMAchieve::CalculateIas(const Units::Length current_ownship_altitude,
                                    const DynamicsState &three_dof_dynamics_state) {
// Do nothing.
}

inline void IMAchieve::CalculateMach(const Units::Time reference_ttg,
                                     const Units::Length current_ownship_altitude) {
// Do nothing.
}

inline void IMAchieve::RecordInternalObserverMetrics(const AircraftState &current_ownship_state,
                                                     const AircraftState &current_target_state,
                                                     const DynamicsState &dynamics_state,
                                                     const Units::Speed unmodified_ias,
                                                     const Units::Speed tas_command,
                                                     const Units::Speed reference_velocity,
                                                     const Units::Length reference_distance,
                                                     const Guidance &guidance) {
   // Do Nothing
}
