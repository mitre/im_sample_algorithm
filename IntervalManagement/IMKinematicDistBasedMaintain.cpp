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

#include <iomanip>
#include "imalgs/IMKinematicDistBasedMaintain.h"

#include "math/CustomMath.h"
#include "public/AircraftCalculations.h"
#include "public/CoreUtils.h"

log4cplus::Logger IMKinematicDistBasedMaintain::logger =
      log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("IMKinematicDistBasedMaintain"));

IMKinematicDistBasedMaintain::IMKinematicDistBasedMaintain()
      : m_measured_spacing_interval(Units::NegInfinity()) {
}

IMKinematicDistBasedMaintain::~IMKinematicDistBasedMaintain() = default;

void IMKinematicDistBasedMaintain::IterationReset() {
   IMMaintain::IterationReset();

   m_measured_spacing_interval = Units::NegInfinity();
}

// Does NOT inherit from IMAlgorithm::Update()
Guidance IMKinematicDistBasedMaintain::Update(const DynamicsState &dynamics_state,
                                              const AircraftState &ownship_aircraft_state,
                                              const AircraftState &target_state_projected_on_ownships_path_at_adjusted_distance,
                                              const Units::Length target_dtg_along_ownships_path_at_adjusted_distance,
                                              const Units::Length target_dtg_along_ownships_path,
                                              const KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor,
                                              const Guidance &guidance_in,
                                              const vector<AircraftState> &target_aircraft_state_history,
                                              const AchievePointCalcs &ownship_achieve_point_calcs,
                                              const AchievePointCalcs &traffic_reference_point_calcs,
                                              PilotDelay &pilot_delay) {
   LOG4CPLUS_DEBUG(logger, "ownship_aircraft_state.m_time," << std::setprecision(15) <<  ownship_aircraft_state.m_time);
   LOG4CPLUS_DEBUG(logger, "ownship_aircraft_state.m_id," << ownship_aircraft_state.m_id);
   LOG4CPLUS_DEBUG(logger, "target_dtg_along_ownships_path," << std::setprecision(12) << Units::MetersLength(target_dtg_along_ownships_path));
   LOG4CPLUS_DEBUG(logger, "target_dtg_along_ownships_path_at_adjusted_distance," << std::setprecision(12) << Units::MetersLength(target_dtg_along_ownships_path_at_adjusted_distance));
   LOG4CPLUS_DEBUG(logger, "target_state_projected_on_ownships_path_at_adjusted_distance.x," << std::setprecision(12) << Units::MetersLength(target_state_projected_on_ownships_path_at_adjusted_distance.GetPositionX()));
   LOG4CPLUS_DEBUG(logger, "target_state_projected_on_ownships_path_at_adjusted_distance.y," << std::setprecision(12) << Units::MetersLength(target_state_projected_on_ownships_path_at_adjusted_distance.GetPositionY()));
   LOG4CPLUS_DEBUG(logger, "target_state_projected_on_ownships_path_at_adjusted_distance.z," << std::setprecision(12) << Units::MetersLength(target_state_projected_on_ownships_path_at_adjusted_distance.GetPositionZ()));
   LOG4CPLUS_DEBUG(logger, "target_state_projected_on_ownships_path_at_adjusted_distance.xd," << std::setprecision(12) << Units::KnotsSpeed(target_state_projected_on_ownships_path_at_adjusted_distance.GetSpeedXd()));
   LOG4CPLUS_DEBUG(logger, "target_state_projected_on_ownships_path_at_adjusted_distance.yd," << std::setprecision(12) << Units::KnotsSpeed(target_state_projected_on_ownships_path_at_adjusted_distance.GetSpeedYd()));
   LOG4CPLUS_DEBUG(logger, "ownship_aircraft_state.x," << std::setprecision(12) << Units::MetersLength(ownship_aircraft_state.GetPositionX()));
   LOG4CPLUS_DEBUG(logger, "ownship_aircraft_state.y," << std::setprecision(12) << Units::MetersLength(ownship_aircraft_state.GetPositionY()));
   LOG4CPLUS_DEBUG(logger, "ownship_aircraft_state.z," << std::setprecision(12) << Units::MetersLength(ownship_aircraft_state.GetPositionZ()));
   LOG4CPLUS_DEBUG(logger, "ownship_aircraft_state.xd," << std::setprecision(12) << Units::KnotsSpeed(ownship_aircraft_state.GetSpeedXd()));
   LOG4CPLUS_DEBUG(logger, "ownship_aircraft_state.yd," << std::setprecision(12) << Units::KnotsSpeed(ownship_aircraft_state.GetSpeedYd()));

   /*
    * Developer's note: In this level of the algorithm, all uses of the /target state/
    * must be projected onto ownship's route prior to use. This includes all items
    * in the targethistory vector (they have not already been projected). Some lower
    * level algorithms carry this load automatically
    */
   Guidance guidanceout = guidance_in;

   // This will give us a state projected onto ownship's h-path
   // NOTE: targetin has already been projected. We only need the distance
   // calculation here. But we must use projectTargetPos() to get protection
   // against being off the back of the horizontal path.
   Units::Length target_projected_x(target_state_projected_on_ownships_path_at_adjusted_distance.GetPositionX());
   Units::Length target_projected_y(target_state_projected_on_ownships_path_at_adjusted_distance.GetPositionY());
   Units::Length target_projected_dtg(target_dtg_along_ownships_path_at_adjusted_distance);

   Units::Length ownship_true_dtg;
   m_ownship_decrementing_distance_calculator.CalculateAlongPathDistanceFromPosition(
         Units::FeetLength(ownship_aircraft_state.m_x),
         Units::FeetLength(ownship_aircraft_state.m_y),
         ownship_true_dtg);

   if (ownship_true_dtg <=
       Units::MetersLength(ownship_kinematic_trajectory_predictor.GetVerticalPathDistances().back())) {
      guidanceout.SetValid(true);

      m_measured_spacing_interval = ownship_true_dtg - target_dtg_along_ownships_path;

      if (m_previous_reference_im_speed_command_tas == Units::zero()) {
         m_previous_reference_im_speed_command_tas = m_weather_prediction.getAtmosphere()->CAS2TAS(
               Units::MetersPerSecondSpeed(ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities().back()),
               Units::MetersLength(ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes().back()));
         m_previous_im_speed_command_ias = Units::MetersPerSecondSpeed(
               ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities().back());
         m_previous_reference_im_speed_command_mach =
               Units::MetersPerSecondSpeed(m_previous_reference_im_speed_command_tas).value() /
               sqrt(GAMMA * R.value() * m_weather_prediction.getAtmosphere()->GetTemperature(
                     Units::MetersLength(
                           ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes().back())).value());
      }

      Units::Speed ground_speed_command =
            target_state_projected_on_ownships_path_at_adjusted_distance.GetGroundSpeed() +
            (ownship_true_dtg - target_projected_dtg) * m_maintain_control_gain;
      Units::Speed true_airspeed_command =
            sqrt(Units::sqr(ground_speed_command - Units::MetersPerSecondSpeed(ownship_aircraft_state.m_Vw_para)) +
                 Units::sqr(Units::MetersPerSecondSpeed(ownship_aircraft_state.m_Vw_perp))) /
            cos(ownship_aircraft_state.m_gamma);
      LOG4CPLUS_DEBUG(logger, "ground_speed_command," << std::setprecision(12) << Units::KnotsSpeed(ground_speed_command));
      LOG4CPLUS_DEBUG(logger, "ownship_true_dtg," << std::setprecision(12) << Units::MetersLength(ownship_true_dtg));
      LOG4CPLUS_DEBUG(logger, "target_projected_dtg," << std::setprecision(12) << Units::MetersLength(target_projected_dtg));
      LOG4CPLUS_DEBUG(logger, "true_airspeed_command," << std::setprecision(12) << Units::KnotsSpeed(true_airspeed_command));

      if (true_airspeed_command < Units::zero()) {
         true_airspeed_command = Units::zero();
      }

      m_previous_reference_im_speed_command_tas = true_airspeed_command;
      m_im_speed_command_ias =
            m_weather_prediction.getAtmosphere()->TAS2CAS(true_airspeed_command,
                                                          Units::FeetLength(ownship_aircraft_state.m_z));
      m_unmodified_im_speed_command_ias = m_im_speed_command_ias;

      m_ownship_reference_lookup_index =
            CoreUtils::FindNearestIndex(Units::MetersLength(ownship_true_dtg).value(),
                                        ownship_kinematic_trajectory_predictor.GetVerticalPathDistances());

      if (guidanceout.GetSelectedSpeed().GetSpeedType() == INDICATED_AIR_SPEED) {
         CalculateIas(Units::FeetLength(ownship_aircraft_state.m_z), target_dtg_along_ownships_path,
                      dynamics_state, ownship_kinematic_trajectory_predictor, pilot_delay);
      } else {
         CalculateMach(Units::FeetLength(ownship_aircraft_state.m_z), target_dtg_along_ownships_path,
                       true_airspeed_command,
                       ownship_kinematic_trajectory_predictor, pilot_delay);
      }

      m_previous_im_speed_command_ias = m_im_speed_command_ias;

      if (pilot_delay.IsPilotDelayOn()) {
         guidanceout.m_ias_command = m_im_speed_command_with_pilot_delay;
         if (guidanceout.GetSelectedSpeed().GetSpeedType() == MACH_SPEED) {
            const auto true_airspeed_equivalent = m_weather_prediction.CAS2TAS(m_im_speed_command_with_pilot_delay, ownship_aircraft_state.GetPositionZ());
            const auto mach_equivalent = m_weather_prediction.TAS2Mach(true_airspeed_equivalent, ownship_aircraft_state.GetPositionZ());
            guidanceout.SetMachCommand(mach_equivalent);
         }
      } else {
         guidanceout.m_ias_command = m_im_speed_command_ias;
         if (guidanceout.GetSelectedSpeed().GetSpeedType() == MACH_SPEED) {
            const auto true_airspeed_equivalent = m_weather_prediction.CAS2TAS(m_im_speed_command_ias, ownship_aircraft_state.GetPositionZ());
            const auto mach_equivalent = m_weather_prediction.TAS2Mach(true_airspeed_equivalent, ownship_aircraft_state.GetPositionZ());
            guidanceout.SetMachCommand(mach_equivalent);
         }
      }

      RecordInternalObserverData(ownship_aircraft_state, target_state_projected_on_ownships_path_at_adjusted_distance,
                                 dynamics_state, true_airspeed_command,
                                 target_projected_dtg, ownship_true_dtg, target_aircraft_state_history,
                                 ownship_kinematic_trajectory_predictor);
   } else {
      m_ownship_reference_lookup_index =
            static_cast<int>(ownship_kinematic_trajectory_predictor.GetVerticalPathDistances().size() - 1);

      guidanceout.SetValid(false);
      m_measured_spacing_interval = Units::NegInfinity();
   }

   return guidanceout;
}

void IMKinematicDistBasedMaintain::CalculateIas(const Units::Length current_ownship_altitude,
                                                const Units::Length target_kinematic_dtg_to_end_of_route,
                                                const DynamicsState &dynamics_state,
                                                const KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor,
                                                PilotDelay &pilot_delay) {
   BadaWithCalc &bada_calculator =
         ownship_kinematic_trajectory_predictor.GetKinematicDescent4dPredictor()->m_bada_calculator;

   Units::Speed rf_leg_limit_speed = Units::zero();
   if (m_has_rf_leg) {
      rf_leg_limit_speed = GetRFLegSpeedLimit(Units::MetersLength(
            ownship_kinematic_trajectory_predictor.GetVerticalPathDistanceByIndex(
                  m_ownship_reference_lookup_index)));
   }

   m_im_speed_command_ias =
         LimitImSpeedCommand(m_im_speed_command_ias,
                             ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex
                                   (m_ownship_reference_lookup_index),
                             Units::zero(), bada_calculator, current_ownship_altitude,
                             dynamics_state.m_flap_configuration, rf_leg_limit_speed);

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
      const Units::Length current_ownship_altitude,
      const Units::Length target_kinematic_dtg_to_end_of_route,
      const Units::Speed true_airspeed_command,
      const KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor,
      PilotDelay &pilot_delay) {

   Units::Speed nominal_profile_ias =
         Units::MetersPerSecondSpeed(
               ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(m_ownship_reference_lookup_index));
   Units::Speed nominal_profile_tas =
         m_weather_prediction.getAtmosphere()->CAS2TAS(nominal_profile_ias, current_ownship_altitude);

   double nominal_mach =
         Units::MetersPerSecondSpeed(nominal_profile_tas).value() /
         sqrt(GAMMA * R.value() *
              m_weather_prediction.getAtmosphere()->GetTemperature(current_ownship_altitude).value());

   double temp_mach =
         Units::MetersPerSecondSpeed(true_airspeed_command).value() /
         sqrt(GAMMA * R.value() *
              m_weather_prediction.getAtmosphere()->GetTemperature(current_ownship_altitude).value());

   if (target_kinematic_dtg_to_end_of_route > Units::zero()) {
      BadaWithCalc &bada_calculator =
            ownship_kinematic_trajectory_predictor.GetKinematicDescent4dPredictor()->m_bada_calculator;

      temp_mach = LimitImMachCommand(temp_mach, nominal_mach, bada_calculator, current_ownship_altitude);

   } else {
      m_measured_spacing_interval = Units::NegInfinity();
      m_ownship_reference_lookup_index =
            CoreUtils::FindNearestIndex(Units::MetersLength(m_ownship_kinematic_dtg_to_ptp).value(),
                                        ownship_kinematic_trajectory_predictor.GetVerticalPathDistances());
      temp_mach = std::min(nominal_mach, m_previous_reference_im_speed_command_mach);
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

void IMKinematicDistBasedMaintain::
RecordInternalObserverData(const AircraftState &ownship_aircraft_state,
                           const AircraftState &target_aircraft_state,
                           const DynamicsState &dynamics_state,
                           const Units::Speed true_airspeed_command,
                           const Units::Length target_true_dtg,
                           const Units::Length ownship_true_dtg,
                           const std::vector<AircraftState> &target_aircraft_state_history,
                           const KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor) {

   if (InternalObserver::getInstance()->GetScenarioIter() >= 0) {
      InternalObserver::getInstance()->IM_command_output(ownship_aircraft_state.m_id,
                                                         ownship_aircraft_state.m_time,
                                                         ownship_aircraft_state.m_z,
                                                         Units::MetersPerSecondSpeed(
                                                               dynamics_state.v_true_airspeed).value(),
                                                         Units::MetersPerSecondSpeed(
                                                               ownship_aircraft_state.GetGroundSpeed()).value(),
                                                         Units::MetersPerSecondSpeed(m_im_speed_command_ias).value(),
                                                         Units::MetersPerSecondSpeed(
                                                               m_unmodified_im_speed_command_ias).value(),
                                                         Units::MetersPerSecondSpeed(true_airspeed_command).value(),
                                                         Units::MetersPerSecondSpeed(
                                                               target_aircraft_state.GetGroundSpeed()).value(),
                                                         Units::MetersLength(-target_true_dtg).value(),
                                                         Units::MetersLength(-ownship_true_dtg).value(),
                                                         Units::MetersLength(ownship_true_dtg).value());
   }

   InternalObserver::getInstance()->updateFinalGS(target_aircraft_state.m_id, Units::MetersPerSecondSpeed(
         AircraftCalculations::GsAtACS(target_aircraft_state_history.back())).value());
   InternalObserver::getInstance()->updateFinalGS(ownship_aircraft_state.m_id, Units::MetersPerSecondSpeed(
         AircraftCalculations::GsAtACS(ownship_aircraft_state)).value());

   if (InternalObserver::getInstance()->GetRecordMaintainMetrics()) {
      MaintainMetric &maintain_metric = InternalObserver::getInstance()->GetMaintainMetric(ownship_aircraft_state.m_id);
      maintain_metric.AddSpacingErrorSec(
            Units::SecondsTime(Units::zero()).value());

      if (!maintain_metric.TimeAtAbpRecorded()) {
         maintain_metric.SetTimeAtAbp(
               ownship_aircraft_state.m_time);
      }
      maintain_metric.ComputeTotalMaintainTime(
            ownship_aircraft_state.m_time);
   }

   if (InternalObserver::getInstance()->outputNM()) {
      NMObserver &nm_observer = InternalObserver::getInstance()->GetNMObserver(ownship_aircraft_state.m_id);

      if (nm_observer.curr_NM == -2) {
         nm_observer.curr_NM =
               static_cast<int>(Units::NauticalMilesLength(ownship_true_dtg).value());
      }

      if (ownship_true_dtg <=
          Units::NauticalMilesLength(
                nm_observer.curr_NM)) {
         --nm_observer.curr_NM;

         double lval =
               LowLimit(ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(
                     m_ownship_reference_lookup_index));
         double hval =
               HighLimit(ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(
                     m_ownship_reference_lookup_index));

         double ltas = Units::MetersPerSecondSpeed(
               m_weather_prediction.getAtmosphere()->CAS2TAS(Units::MetersPerSecondSpeed(lval),
                                                             Units::FeetLength(ownship_aircraft_state.m_z))).value();
         double htas = Units::MetersPerSecondSpeed(
               m_weather_prediction.getAtmosphere()->CAS2TAS(Units::MetersPerSecondSpeed(hval),
                                                             Units::FeetLength(ownship_aircraft_state.m_z))).value();

         nm_observer.output_NM_values(
               Units::MetersLength(ownship_true_dtg).value(),
               Units::MetersLength(-ownship_true_dtg).value(),
               ownship_aircraft_state.m_time,
               Units::MetersPerSecondSpeed(m_im_speed_command_ias).value(),
               Units::MetersPerSecondSpeed(ownship_aircraft_state.GetGroundSpeed()).value(),
               Units::MetersPerSecondSpeed(target_aircraft_state.GetGroundSpeed()).value(),
               lval, hval, ltas, htas);
      }
   }
}

void IMKinematicDistBasedMaintain::DumpParameters(const std::string &parameters_to_print) {
   IMMaintain::DumpParameters(parameters_to_print);
}
