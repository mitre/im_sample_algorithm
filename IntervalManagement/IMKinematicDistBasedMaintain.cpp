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

#include "imalgs/IMKinematicDistBasedMaintain.h"

#include "public/CustomMath.h"
#include "public/AircraftCalculations.h"
#include "public/CoreUtils.h"
#include "imalgs/InternalObserver.h"

using namespace interval_management::open_source;

log4cplus::Logger IMKinematicDistBasedMaintain::m_logger =
      log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("IMKinematicDistBasedMaintain"));

IMKinematicDistBasedMaintain::IMKinematicDistBasedMaintain() : m_measured_spacing_interval(Units::NegInfinity()) {}

IMKinematicDistBasedMaintain::~IMKinematicDistBasedMaintain() = default;

void IMKinematicDistBasedMaintain::IterationReset() {
   IMMaintain::IterationReset();

   m_measured_spacing_interval = Units::NegInfinity();
}

aaesim::open_source::Guidance IMKinematicDistBasedMaintain::Update(
      const aaesim::open_source::DynamicsState &dynamics_state,
      const interval_management::open_source::AircraftState &ownship_aircraft_state,
      const interval_management::open_source::AircraftState
            &target_state_projected_on_ownships_path_at_adjusted_distance,
      const Units::Length target_dtg_along_ownships_path_at_adjusted_distance,
      const Units::Length target_dtg_along_ownships_path,
      const aaesim::open_source::KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor,
      const aaesim::open_source::Guidance &guidance_in,
      const std::vector<interval_management::open_source::AircraftState> &target_aircraft_state_history,
      const interval_management::open_source::AchievePointCalcs &ownship_achieve_point_calcs,
      const interval_management::open_source::AchievePointCalcs &traffic_reference_point_calcs,
      PilotDelay &pilot_delay) {

   /*
    * Developer's note: In this level of the algorithm, all uses of the /target state/
    * must be projected onto ownship's route prior to use. This includes all items
    * in the targethistory vector (they have not already been projected). Some lower
    * level algorithms carry this load automatically
    */
   aaesim::open_source::Guidance guidanceout = guidance_in;

   // target's along-path position on ownship's route (adjusted for ASG)
   Units::Length target_projected_x(target_state_projected_on_ownships_path_at_adjusted_distance.GetPositionX());
   Units::Length target_projected_y(target_state_projected_on_ownships_path_at_adjusted_distance.GetPositionY());
   Units::Length target_projected_dtg(target_dtg_along_ownships_path_at_adjusted_distance);

   Units::Length ownship_estimated_dtg;
   m_ownship_decrementing_distance_calculator.CalculateAlongPathDistanceFromPosition(
         ownship_aircraft_state.GetPositionX(), ownship_aircraft_state.GetPositionY(), ownship_estimated_dtg);

   if (ownship_estimated_dtg <=
       Units::MetersLength(ownship_kinematic_trajectory_predictor.GetVerticalPathDistances().back())) {
      guidanceout.SetValid(true);

      m_measured_spacing_interval = ownship_estimated_dtg - target_dtg_along_ownships_path;

      if (m_previous_reference_im_speed_command_tas == Units::zero()) {
         m_previous_reference_im_speed_command_tas = m_weather_prediction.getAtmosphere()->CAS2TAS(
               Units::MetersPerSecondSpeed(ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities().back()),
               Units::MetersLength(ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes().back()));
         m_previous_im_speed_command_ias =
               Units::MetersPerSecondSpeed(ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities().back());
         m_previous_reference_im_speed_command_mach =
               Units::MetersPerSecondSpeed(m_previous_reference_im_speed_command_tas).value() /
               sqrt(GAMMA * R.value() *
                    m_weather_prediction.getAtmosphere()
                          ->GetTemperature(Units::MetersLength(
                                ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes().back()))
                          .value());
      }

      Units::Speed ground_speed_command =
            target_state_projected_on_ownships_path_at_adjusted_distance.GetGroundSpeed() +
            (ownship_estimated_dtg - target_projected_dtg) * m_maintain_control_gain;
      Units::Speed true_airspeed_command =
            sqrt(Units::sqr(ground_speed_command -
                            Units::MetersPerSecondSpeed(ownship_aircraft_state.GetSensedWindParallelComponent())) +
                 Units::sqr(
                       Units::MetersPerSecondSpeed(ownship_aircraft_state.GetSensedWindPerpendicularComponent()))) /
            cos(ownship_aircraft_state.GetGamma().value());

      if (true_airspeed_command < Units::zero()) {
         true_airspeed_command = Units::zero();
      }

      m_previous_reference_im_speed_command_tas = true_airspeed_command;
      m_im_speed_command_ias = m_weather_prediction.getAtmosphere()->TAS2CAS(
            true_airspeed_command, Units::FeetLength(ownship_aircraft_state.m_z));
      m_unmodified_im_speed_command_ias = m_im_speed_command_ias;

      m_ownship_reference_lookup_index =
            CoreUtils::FindNearestIndex(Units::MetersLength(ownship_estimated_dtg).value(),
                                        ownship_kinematic_trajectory_predictor.GetVerticalPathDistances());

      if (guidanceout.GetSelectedSpeed().GetSpeedType() == INDICATED_AIR_SPEED) {
         CalculateIas(Units::FeetLength(ownship_aircraft_state.m_z), target_dtg_along_ownships_path, dynamics_state,
                      ownship_kinematic_trajectory_predictor, pilot_delay);
      } else {
         CalculateMach(Units::FeetLength(ownship_aircraft_state.m_z), target_dtg_along_ownships_path,
                       true_airspeed_command, ownship_kinematic_trajectory_predictor, pilot_delay,
                       dynamics_state.current_mass);
      }

      m_previous_im_speed_command_ias = m_im_speed_command_ias;

      if (pilot_delay.IsPilotDelayOn()) {
         guidanceout.m_ias_command = m_im_speed_command_with_pilot_delay;
         if (guidanceout.GetSelectedSpeed().GetSpeedType() == MACH_SPEED) {
            const auto true_airspeed_equivalent = m_weather_prediction.CAS2TAS(m_im_speed_command_with_pilot_delay,
                                                                               ownship_aircraft_state.GetPositionZ());
            const auto mach_equivalent =
                  m_weather_prediction.TAS2Mach(true_airspeed_equivalent, ownship_aircraft_state.GetPositionZ());
            guidanceout.SetMachCommand(mach_equivalent);
         }
      } else {
         guidanceout.m_ias_command = m_im_speed_command_ias;
         if (guidanceout.GetSelectedSpeed().GetSpeedType() == MACH_SPEED) {
            const auto true_airspeed_equivalent =
                  m_weather_prediction.CAS2TAS(m_im_speed_command_ias, ownship_aircraft_state.GetPositionZ());
            const auto mach_equivalent =
                  m_weather_prediction.TAS2Mach(true_airspeed_equivalent, ownship_aircraft_state.GetPositionZ());
            guidanceout.SetMachCommand(mach_equivalent);
         }
      }

      RecordInternalObserverData(ownship_aircraft_state, target_state_projected_on_ownships_path_at_adjusted_distance,
                                 dynamics_state, true_airspeed_command, target_projected_dtg, ownship_estimated_dtg,
                                 target_aircraft_state_history, ownship_kinematic_trajectory_predictor);
   } else {
      m_ownship_reference_lookup_index =
            static_cast<int>(ownship_kinematic_trajectory_predictor.GetVerticalPathDistances().size() - 1);

      guidanceout.SetValid(false);
      m_measured_spacing_interval = Units::NegInfinity();
   }

   return guidanceout;
}

void IMKinematicDistBasedMaintain::CalculateIas(
      const Units::Length current_ownship_altitude, const Units::Length target_kinematic_dtg_to_end_of_route,
      const aaesim::open_source::DynamicsState &dynamics_state,
      const aaesim::open_source::KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor,
      PilotDelay &pilot_delay) {
   m_im_speed_command_ias = m_speed_limiter.LimitSpeedCommand(
         m_previous_im_speed_command_ias, m_im_speed_command_ias,
         ownship_kinematic_trajectory_predictor.GetVerticalPathCasByIndex(m_ownship_reference_lookup_index),
         Units::ZERO_LENGTH,
         Units::MetersLength(
               ownship_kinematic_trajectory_predictor.GetVerticalPathDistanceByIndex(m_ownship_reference_lookup_index)),
         current_ownship_altitude, dynamics_state.flap_configuration);

   if (m_im_speed_command_ias != m_previous_im_speed_command_ias) {
      m_total_number_of_im_speed_changes++;
   }

   if (pilot_delay.IsPilotDelayOn()) {
      m_im_speed_command_with_pilot_delay =
            pilot_delay.UpdateIAS(m_previous_im_speed_command_ias, m_im_speed_command_ias, current_ownship_altitude,
                                  ownship_kinematic_trajectory_predictor.GetAltitudeAtFinalWaypoint());
   }
}

void IMKinematicDistBasedMaintain::CalculateMach(
      const Units::Length current_ownship_altitude, const Units::Length target_kinematic_dtg_to_end_of_route,
      const Units::Speed true_airspeed_command,
      const aaesim::open_source::KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor,
      PilotDelay &pilot_delay, const Units::Mass current_mass) {

   Units::Speed nominal_profile_ias = Units::MetersPerSecondSpeed(
         ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(m_ownship_reference_lookup_index));
   Units::Speed nominal_profile_tas =
         m_weather_prediction.getAtmosphere()->CAS2TAS(nominal_profile_ias, current_ownship_altitude);

   BoundedValue<double, 0, 2> nominal_mach(
         Units::MetersPerSecondSpeed(nominal_profile_tas).value() /
         sqrt(GAMMA * R.value() *
              m_weather_prediction.getAtmosphere()->GetTemperature(current_ownship_altitude).value()));

   BoundedValue<double, 0, 2> temp_mach(
         Units::MetersPerSecondSpeed(true_airspeed_command).value() /
         sqrt(GAMMA * R.value() *
              m_weather_prediction.getAtmosphere()->GetTemperature(current_ownship_altitude).value()));

   if (target_kinematic_dtg_to_end_of_route > Units::zero()) {
      temp_mach = m_speed_limiter.LimitMachCommand(
            BoundedValue<double, 0, 2>(m_previous_reference_im_speed_command_mach), temp_mach, nominal_mach,
            current_mass, current_ownship_altitude, m_weather_prediction);
   } else {
      m_measured_spacing_interval = Units::NegInfinity();
      m_ownship_reference_lookup_index =
            CoreUtils::FindNearestIndex(Units::MetersLength(m_ownship_kinematic_dtg_to_ptp).value(),
                                        ownship_kinematic_trajectory_predictor.GetVerticalPathDistances());
      temp_mach = std::min(nominal_mach, BoundedValue<double, 0, 2>(m_previous_reference_im_speed_command_mach));
   }

   if (temp_mach != m_previous_reference_im_speed_command_mach) {
      m_total_number_of_im_speed_changes++;
   }

   if (pilot_delay.IsPilotDelayOn()) {
      m_im_speed_command_with_pilot_delay =
            pilot_delay.UpdateMach(m_previous_reference_im_speed_command_mach, temp_mach, current_ownship_altitude,
                                   ownship_kinematic_trajectory_predictor.GetAltitudeAtFinalWaypoint());
   }

   m_previous_reference_im_speed_command_mach = temp_mach;
   m_im_speed_command_ias = m_weather_prediction.getAtmosphere()->MachToIAS(temp_mach, current_ownship_altitude);
}

void IMKinematicDistBasedMaintain::RecordInternalObserverData(
      const interval_management::open_source::AircraftState &ownship_aircraft_state,
      const interval_management::open_source::AircraftState &target_aircraft_state,
      const aaesim::open_source::DynamicsState &dynamics_state, const Units::Speed true_airspeed_command,
      const Units::Length target_true_dtg, const Units::Length ownship_true_dtg,
      const std::vector<interval_management::open_source::AircraftState> &target_aircraft_state_history,
      const aaesim::open_source::KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor) {

   InternalObserver::getInstance()->updateFinalGS(
         target_aircraft_state.GetId(),
         Units::MetersPerSecondSpeed(target_aircraft_state_history.back().GetGroundSpeed()).value());
   InternalObserver::getInstance()->updateFinalGS(
         ownship_aircraft_state.GetId(), Units::MetersPerSecondSpeed(ownship_aircraft_state.GetGroundSpeed()).value());

   if (InternalObserver::getInstance()->GetRecordMaintainMetrics()) {
      MaintainMetric &maintain_metric =
            InternalObserver::getInstance()->GetMaintainMetric(ownship_aircraft_state.GetId());
      maintain_metric.AddSpacingErrorSec(Units::SecondsTime(Units::zero()).value());

      if (!maintain_metric.TimeAtAbpRecorded()) {
         maintain_metric.SetTimeAtAbp(ownship_aircraft_state.GetTimeStamp().value());
      }
      maintain_metric.ComputeTotalMaintainTime(ownship_aircraft_state.GetTimeStamp().value());
   }

   if (InternalObserver::getInstance()->outputNM()) {
      NMObserver &nm_observer = InternalObserver::getInstance()->GetNMObserver(ownship_aircraft_state.GetId());

      if (nm_observer.curr_NM == -2) {
         nm_observer.curr_NM = static_cast<int>(Units::NauticalMilesLength(ownship_true_dtg).value());
      }

      if (ownship_true_dtg <= Units::NauticalMilesLength(nm_observer.curr_NM)) {
         --nm_observer.curr_NM;

         double lval = m_speed_limiter.LowLimit(
               ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(m_ownship_reference_lookup_index));
         double hval = m_speed_limiter.HighLimit(
               ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(m_ownship_reference_lookup_index));

         double ltas = Units::MetersPerSecondSpeed(
                             m_weather_prediction.getAtmosphere()->CAS2TAS(
                                   Units::MetersPerSecondSpeed(lval), Units::FeetLength(ownship_aircraft_state.m_z)))
                             .value();
         double htas = Units::MetersPerSecondSpeed(
                             m_weather_prediction.getAtmosphere()->CAS2TAS(
                                   Units::MetersPerSecondSpeed(hval), Units::FeetLength(ownship_aircraft_state.m_z)))
                             .value();

         nm_observer.output_NM_values(
               Units::MetersLength(ownship_true_dtg).value(), Units::MetersLength(-ownship_true_dtg).value(),
               ownship_aircraft_state.GetTimeStamp().value(),
               Units::MetersPerSecondSpeed(m_im_speed_command_ias).value(),
               Units::MetersPerSecondSpeed(ownship_aircraft_state.GetGroundSpeed()).value(),
               Units::MetersPerSecondSpeed(target_aircraft_state.GetGroundSpeed()).value(), lval, hval, ltas, htas);
      }
   }
}

void IMKinematicDistBasedMaintain::DumpParameters(const std::string &parameters_to_print) {
   IMMaintain::DumpParameters(parameters_to_print);
}
