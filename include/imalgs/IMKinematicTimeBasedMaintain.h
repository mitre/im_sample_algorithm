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

#include <nlohmann/json.hpp>

#include "imalgs/IMMaintain.h"
#include "public/KinematicTrajectoryPredictor.h"
#include "imalgs/TrueDistances.h"
#include "imalgs/AchievePointCalcs.h"
#include "imalgs/AircraftState.h"

namespace interval_management {
namespace open_source {
class IMKinematicTimeBasedMaintain final : public IMMaintain {
  public:
   IMKinematicTimeBasedMaintain();

   virtual ~IMKinematicTimeBasedMaintain();

   virtual void IterationReset();

   aaesim::open_source::Guidance Update(
         const aaesim::open_source::DynamicsState &dynamics_state,
         const interval_management::open_source::AircraftState &ownship_aircraft_state,
         const interval_management::open_source::AircraftState &target_aircraft_state_projected_asg_adjusted,
         const aaesim::open_source::KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor,
         const aaesim::open_source::Guidance &guidance_in,
         const std::vector<interval_management::open_source::AircraftState> &target_aircraft_state_history,
         const interval_management::open_source::AchievePointCalcs &ownship_achieve_point_calcs,
         const interval_management::open_source::AchievePointCalcs &traffic_reference_point_calcs,
         PilotDelay &pilot_delay_model, const Units::Length &target_kinematic_dtg_to_end_of_route);

   virtual const double GetMsi() const;

   virtual void DumpParameters(const std::string &parameters_to_print);

  private:
   static log4cplus::Logger m_logger;

   void DoAlgorithmLogging(
         const interval_management::open_source::AircraftState &ownship_aircraft_state,
         const interval_management::open_source::AircraftState &target_aircraft_state_projected_asg_adjusted,
         Units::Speed targetvelocity, Units::Length ownship_estimated_dtg, Units::Length target_dtg_on_ownship_route,
         Units::Speed gscommand, Units::Speed tascommand, Units::Time target_crossing_time,
         bool target_crossing_time_valid) const;

   void CalculateIas(const Units::Length current_ownship_altitude,
                     const aaesim::open_source::DynamicsState &dynamics_state,
                     const aaesim::open_source::KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor,
                     PilotDelay &pilot_delay);

   void CalculateMach(const Units::Length current_ownship_altitude, const Units::Speed true_airspeed_command,
                      const aaesim::open_source::KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor,
                      PilotDelay &pilot_delay, const Units::Mass current_mass);

   const bool IsOwnshipBelowTransitionAltitude(
         Units::Length current_ownship_altitude,
         const aaesim::open_source::KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor);

   Units::Time m_measured_spacing_interval;
};

inline const bool IMKinematicTimeBasedMaintain::IsOwnshipBelowTransitionAltitude(
      Units::Length current_ownship_altitude,
      const aaesim::open_source::KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor) {
   return current_ownship_altitude <
          ownship_kinematic_trajectory_predictor.GetKinematicDescent4dPredictor()->GetTransitionAltitude();
}

inline const double IMKinematicTimeBasedMaintain::GetMsi() const {
   return Units::SecondsTime(m_measured_spacing_interval).value();
}

inline void IMKinematicTimeBasedMaintain::DoAlgorithmLogging(
      const interval_management::open_source::AircraftState &ownship_aircraft_state,
      const interval_management::open_source::AircraftState &target_aircraft_state_projected_asg_adjusted,
      Units::Speed targetvelocity, Units::Length ownship_estimated_dtg, Units::Length target_dtg_on_ownship_route,
      Units::Speed gscommand, Units::Speed tascommand, Units::Time target_crossing_time,
      bool target_crossing_time_valid) const {
   using json = nlohmann::json;
   if (m_logger.getLogLevel() == log4cplus::TRACE_LOG_LEVEL) {
      json j;
      auto state_to_json = [&j](std::string prefix, const AircraftState &state) {
         j[prefix + ".id"] = state.GetId();
         j[prefix + ".timestamp_sec"] = Units::SecondsTime(state.GetTimeStamp()).value();
         j[prefix + ".groundspeed_mps"] = Units::MetersPerSecondSpeed(state.GetGroundSpeed()).value();
         j[prefix + ".true_airspeed_mps"] = Units::MetersPerSecondSpeed(state.GetTrueAirspeed()).value();
         j[prefix + ".position_derivative_x_mps"] = Units::MetersPerSecondSpeed(state.GetSpeedXd()).value();
         j[prefix + ".position_derivative_y_mps"] = Units::MetersPerSecondSpeed(state.GetSpeedYd()).value();
         j[prefix + ".position_derivative_z_mps"] = Units::MetersPerSecondSpeed(state.GetSpeedZd()).value();
         j[prefix + ".position_x_m"] = Units::MetersLength(state.GetPositionX()).value();
         j[prefix + ".position_y_m"] = Units::MetersLength(state.GetPositionY()).value();
         j[prefix + ".position_z_m"] = Units::MetersLength(state.GetPositionZ()).value();
         j[prefix + ".sensed_temp_c"] = Units::CelsiusTemperature(state.GetSensedTemperature()).value();
         j[prefix + ".sensed_wind_east_mps"] = Units::MetersPerSecondSpeed(state.GetSensedWindEastComponent()).value();
         j[prefix + ".sensed_wind_north_mps"] =
               Units::MetersPerSecondSpeed(state.GetSensedWindNorthComponent()).value();
      };
      state_to_json("ownship_state", ownship_aircraft_state);
      state_to_json("target_adjusted_state", target_aircraft_state_projected_asg_adjusted);
      j["targetvelocity_mps"] = Units::MetersPerSecondSpeed(targetvelocity).value();
      j["ownship_estimated_dtg_m"] = Units::MetersLength(ownship_estimated_dtg).value();
      j["target_dtg_on_ownship_route_m"] = Units::MetersLength(target_dtg_on_ownship_route).value();
      j["maintain_control_gain_hz"] = Units::HertzFrequency(m_maintain_control_gain).value();
      j["gscommand_mps"] = Units::MetersPerSecondSpeed(gscommand).value();
      j["tascommand_mps"] = Units::MetersPerSecondSpeed(tascommand).value();
      j["unmodified_im_speed_command_ias_mps"] = Units::MetersPerSecondSpeed(m_unmodified_im_speed_command_ias).value();
      j["im_speed_command_ias_mps"] = Units::MetersPerSecondSpeed(m_im_speed_command_ias).value();
      j["target_crossing_time_sec"] = Units::SecondsTime(target_crossing_time).value();
      j["target_crossing_time_valid"] = target_crossing_time_valid;
      j["measured_spacing_interval_sec"] = Units::SecondsTime(m_measured_spacing_interval).value();
      LOG4CPLUS_TRACE(m_logger, j.dump());
   }
}
}  // namespace open_source
}  // namespace interval_management