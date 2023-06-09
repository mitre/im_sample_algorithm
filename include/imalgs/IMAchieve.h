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

#include "imalgs/IMAlgorithm.h"
#include <scalar/Frequency.h>
#include "public/PredictedWindEvaluator.h"

namespace interval_management {
namespace open_source {

class IMAchieve : public IMAlgorithm {

  public:
   static const Units::Angle TOLERANCE_ANGLE;

   IMAchieve();

   IMAchieve(const IMAchieve &obj);

   virtual ~IMAchieve() = default;

   void IterationReset() override;

   aaesim::open_source::Guidance Update(
         const aaesim::open_source::Guidance &prevguidance, const aaesim::open_source::DynamicsState &dynamicsstate,
         const interval_management::open_source::AircraftState &owntruthstate,
         const interval_management::open_source::AircraftState &targettruthstate,
         const std::vector<interval_management::open_source::AircraftState> &targethistory) override;

   void Initialize(const OwnshipPredictionParameters &ownship_prediction_parameters,
                   const AircraftIntent &ownship_aircraft_intent, WeatherPrediction &weather_prediction) override;

   void DumpParameters(const std::string &parameters_to_print) override;

   virtual const bool IsTargetPassedTrp() const;

   virtual const bool IsTargetPassedLastWaypoint() const;

   virtual const bool InAchieveStage() const;

   const Units::Length GetTargetKinematicDtgToTrp() const;

   const bool IsWithinErrorThreshold() const;

   void SetBlendWind(bool wind_blending_enabled) override;

  protected:
   void Copy(const IMAchieve &obj);

   virtual void CalculateIas(const Units::Length current_ownship_altitude,
                             const aaesim::open_source::DynamicsState &three_dof_dynamics_state);

   virtual void CalculateMach(const Units::Time reference_ttg, const Units::Length current_ownship_altitude);

   virtual const bool IsOwnshipBelowTransitionAltitude(Units::Length current_ownship_altitude);

   virtual void RecordInternalObserverMetrics(
         const interval_management::open_source::AircraftState &current_ownship_state,
         const interval_management::open_source::AircraftState &current_target_state,
         const aaesim::open_source::DynamicsState &dynamics_state, const Units::Speed unmodified_ias,
         const Units::Speed tas_command, const Units::Speed reference_velocity, const Units::Length reference_distance,
         const aaesim::open_source::Guidance &guidance);

   const bool WithinErrorThreshold(const Units::Length distance_to_go, const Units::Time ownship_ttg,
                                   const Units::Time reference_ttg);

   Units::Time GetErrorThreshold(Units::Length distance_to_go);

   static const std::shared_ptr<aaesim::open_source::PredictedWindEvaluator> m_predicted_wind_evaluator;

   bool m_transitioned_to_maintain;
   bool m_within_error_threshold;
   bool m_received_one_valid_target_state;
   std::string m_achieve_by_point;

  private:
   void IterClearIMAch();

   static log4cplus::Logger m_logger;
};

inline const bool IMAchieve::InAchieveStage() const { return m_ownship_kinematic_dtg_to_abp > Units::zero(); }

inline const bool IMAchieve::IsTargetPassedTrp() const { return m_target_kinematic_dtg_to_trp <= Units::zero(); }

inline const bool IMAchieve::IsTargetPassedLastWaypoint() const {
   return m_target_kinematic_dtg_to_last_waypoint <= Units::zero();
}

inline const Units::Length IMAchieve::GetTargetKinematicDtgToTrp() const { return m_target_kinematic_dtg_to_trp; }

inline const bool IMAchieve::IsWithinErrorThreshold() const { return m_within_error_threshold; }

inline const bool IMAchieve::IsOwnshipBelowTransitionAltitude(Units::Length current_ownship_altitude) { return false; }

inline void IMAchieve::CalculateIas(const Units::Length current_ownship_altitude,
                                    const aaesim::open_source::DynamicsState &three_dof_dynamics_state) {
   // Do nothing.
}

inline void IMAchieve::CalculateMach(const Units::Time reference_ttg, const Units::Length current_ownship_altitude) {
   // Do nothing.
}

inline void IMAchieve::RecordInternalObserverMetrics(
      const interval_management::open_source::AircraftState &current_ownship_state,
      const interval_management::open_source::AircraftState &current_target_state,
      const aaesim::open_source::DynamicsState &dynamics_state, const Units::Speed unmodified_ias,
      const Units::Speed tas_command, const Units::Speed reference_velocity, const Units::Length reference_distance,
      const aaesim::open_source::Guidance &guidance) {
   // Do Nothing
}

inline void IMAchieve::SetBlendWind(bool wind_blending_enabled) { /* required by interface, but not implemented in this
                                                                     class */
}
}  // namespace open_source
}  // namespace interval_management