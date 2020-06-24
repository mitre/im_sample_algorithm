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

#include <numeric>
#include <aaesim/Bada.h>
#include "public/SimulationTime.h"
#include "imalgs/IMKinematicTimeBasedMaintain.h"
#include "public/AircraftCalculations.h"
#include "math/CustomMath.h"
#include "public/CoreUtils.h"
#include "imalgs/IMScenario.h"

log4cplus::Logger IMKinematicTimeBasedMaintain::logger = log4cplus::Logger::getInstance(
      LOG4CPLUS_TEXT("IMKinematicTimeBasedMaintain"));

IMKinematicTimeBasedMaintain::IMKinematicTimeBasedMaintain() {
   m_stage_of_im_operation = MAINTAIN;
}

IMKinematicTimeBasedMaintain::~IMKinematicTimeBasedMaintain() = default;

Guidance IMKinematicTimeBasedMaintain::Update(const DynamicsState &dynamics_state,
                                              const AircraftState &ownship_aircraft_state,
                                              const AircraftState &target_aircraft_state_projected_asg_adjusted,
                                              const KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor,
                                              const Guidance &guidance_in,
                                              const vector<AircraftState> &target_aircraft_state_history,
                                              const AchievePointCalcs &ownship_achieve_point_calcs,
                                              const AchievePointCalcs &traffic_reference_point_calcs,
                                              PilotDelay &pilot_delay_model,
                                              const Units::Length &target_kinematic_dtg_to_end_of_route) {
   /*
    * Developer's note: In this level of the algorithm, all uses of the /target state/
    * must be projected onto ownship's route prior to use. This includes all items
    * in the targethistory vector (they have not already been projected). Some lower
    * level algorithms carry this load automatically
    */
   Guidance guidanceout = guidance_in;

   guidanceout.SetValid(true);

   Units::MetersLength tmpdist;
   m_ownship_decrementing_distance_calculator.CalculateAlongPathDistanceFromPosition(
         Units::FeetLength(ownship_aircraft_state.m_x),
         Units::FeetLength(ownship_aircraft_state.m_y),
         tmpdist);
   const Units::Length ownship_true_dtg = tmpdist;


   // This will give us a state projected onto ownship's h-path
   // NOTE: targetin has already been projected. We only need the distance
   // calculation here. But we must use projectTargetPos() to get protection
   // against being off the back of the horizontal path.
   Units::Length targetProjectedX = Units::zero();
   Units::Length targetProjectedY = Units::zero();
   const bool foundProjectedPos = IMUtils::ProjectTargetPosition(
         Units::FeetLength(target_aircraft_state_projected_asg_adjusted.m_x),
         Units::FeetLength(target_aircraft_state_projected_asg_adjusted.m_y),
         ownship_kinematic_trajectory_predictor.GetHorizontalPath(),
         targetProjectedX,
         targetProjectedY,
         tmpdist);
   const Units::Length target_projected_dtg = tmpdist;
   bool storespacingerror = true;
   const Units::Speed targetvelocity = target_aircraft_state_projected_asg_adjusted.GetGroundSpeed();
   Units::Time spacingerrorformaintainstats = Units::zero();
   Units::Time target_crossing_time = Units::zero();

   m_measured_spacing_interval = Units::NegInfinity();
   Units::Length projected_x;
   Units::Length projected_y;

   if (!target_aircraft_state_history.empty()) {
      vector<AircraftState> history = target_aircraft_state_history;
      if (history.back().m_time < target_aircraft_state_projected_asg_adjusted.m_time) {
         // This is an edge case. The incoming state was extrapolated to get to the current
         // time. In this case, the history vector needs to have this extrapolated state also
         // in order for GetCrossingTime to operate consistently with this method.
         history.push_back(target_aircraft_state_projected_asg_adjusted);
      }
      storespacingerror = IMUtils::GetCrossingTime(ownship_true_dtg,
                                                   history,
                                                   m_ownship_distance_calculator,
                                                   target_crossing_time,
                                                   projected_x,
                                                   projected_y);

      if (!storespacingerror) {
         LOG4CPLUS_WARN(IMKinematicTimeBasedMaintain::logger, "update ac " << ownship_aircraft_state.m_id
                                                                           << "  time " << ownship_aircraft_state.m_time
                                                                           << std::endl
                                                                           << "Bad target crossing time computing spacing error for maintain stats"
                                                                           << std::endl);
      } else {
         spacingerrorformaintainstats = Units::SecondsTime(ownship_aircraft_state.m_time) - target_crossing_time -
                                        m_im_clearance.GetAssignedTimeSpacingGoal();
         m_measured_spacing_interval = Units::SecondsTime(ownship_aircraft_state.m_time) - target_crossing_time;
      }
   }

   if (m_ownship_reference_lookup_index == -1) {
      m_ownship_reference_lookup_index = static_cast<int>(
            ownship_kinematic_trajectory_predictor.GetVerticalPathDistances().size() - 1);
   }

   // if target distance is inside the range of the precalculated trajectory, use that to find the precalc index for the +- 10% calculation
   if (target_aircraft_state_projected_asg_adjusted.m_id != IMUtils::UNINITIALIZED_AIRCRAFT_ID && foundProjectedPos) {
      if (Units::abs(ownship_true_dtg) <=
          Units::abs(Units::MetersLength(ownship_kinematic_trajectory_predictor.GetVerticalPathDistances().back()))) {
         // Compute precalculated index from own descent and current distance.
         m_ownship_reference_lookup_index = CoreUtils::FindNearestIndex(Units::MetersLength(ownship_true_dtg).value(),
                                                                        ownship_kinematic_trajectory_predictor.GetVerticalPathDistances());

         if (m_ownship_reference_lookup_index >
             ownship_kinematic_trajectory_predictor.GetVerticalPathDistances().size() - 1) {
            m_ownship_reference_lookup_index = static_cast<int>(
                  ownship_kinematic_trajectory_predictor.GetVerticalPathDistances().size() - 1);
         }
      } else {
         guidanceout.SetValid(false);
      }
   } else {
      guidanceout.SetValid(false);
   }

   if (guidanceout.IsValid()) {
      if (!target_aircraft_state_history.empty()) {
         InternalObserver::getInstance()->updateFinalGS(target_aircraft_state_projected_asg_adjusted.m_id,
                                                        Units::MetersPerSecondSpeed(
                                                              AircraftCalculations::GsAtACS(
                                                                    target_aircraft_state_history.back())).value());
         InternalObserver::getInstance()->updateFinalGS(ownship_aircraft_state.m_id, Units::MetersPerSecondSpeed(
               AircraftCalculations::GsAtACS(ownship_aircraft_state)).value());

         if (m_previous_reference_im_speed_command_tas == Units::zero()) {
            m_previous_reference_im_speed_command_tas = m_weather_prediction.getAtmosphere()->CAS2TAS(
                  Units::MetersPerSecondSpeed(
                        ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities().back()),
                  Units::MetersLength(ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes().back()));
            m_previous_im_speed_command_ias = Units::MetersPerSecondSpeed(
                  ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities().back());
            m_previous_reference_im_speed_command_mach =
                  Units::MetersPerSecondSpeed(m_previous_reference_im_speed_command_tas).value() /
                  sqrt(GAMMA * R.value() * m_weather_prediction.getAtmosphere()->GetTemperature(
                        Units::MetersLength(
                              ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes().back())).value());
         }

         if (InternalObserver::getInstance()->GetRecordMaintainMetrics()) {
            if (Units::abs(ownship_true_dtg) < ownship_achieve_point_calcs.GetDistanceFromWaypoint()) {
               MaintainMetric &maintain_metric = InternalObserver::getInstance()->GetMaintainMetric(
                     ownship_aircraft_state.m_id);
               if (storespacingerror) {
                  maintain_metric.AddSpacingErrorSec(
                        Units::SecondsTime(spacingerrorformaintainstats).value());
               }

               if (!maintain_metric.TimeAtAbpRecorded()) {
                  maintain_metric.SetTimeAtAbp(
                        ownship_aircraft_state.m_time);
               }
               maintain_metric.ComputeTotalMaintainTime(
                     ownship_aircraft_state.m_time);
            }
         }
      }

      Units::Speed gscommand = targetvelocity + (ownship_true_dtg - target_projected_dtg) * m_maintain_control_gain;
      Units::Speed tascommand =
            sqrt(Units::sqr(gscommand - Units::MetersPerSecondSpeed(ownship_aircraft_state.m_Vw_para)) +
                 Units::sqr(Units::MetersPerSecondSpeed(ownship_aircraft_state.m_Vw_perp))) /
            cos(ownship_aircraft_state.m_gamma);
      m_previous_reference_im_speed_command_tas = tascommand;  // remember the speed command

      if (tascommand < Units::zero()) {
         tascommand = Units::zero();
      }

      m_im_speed_command_ias = m_weather_prediction.getAtmosphere()->TAS2CAS(tascommand,
                                                                             Units::FeetLength(
                                                                                   ownship_aircraft_state.m_z));
      m_unmodified_im_speed_command_ias = m_im_speed_command_ias;

      if (guidanceout.GetSelectedSpeed().GetSpeedType() == INDICATED_AIR_SPEED) {
         CalculateIas(Units::FeetLength(ownship_aircraft_state.m_z), dynamics_state,
                      ownship_kinematic_trajectory_predictor,
                      pilot_delay_model);
      } else {
         CalculateMach(Units::FeetLength(ownship_aircraft_state.m_z), tascommand,
                       ownship_kinematic_trajectory_predictor,
                       pilot_delay_model);
      }

      m_previous_im_speed_command_ias = m_im_speed_command_ias;

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
                                                            Units::MetersPerSecondSpeed(tascommand).value(),
                                                            Units::MetersPerSecondSpeed(targetvelocity).value(),
                                                            Units::MetersLength(-target_projected_dtg).value(),
                                                            Units::MetersLength(-ownship_true_dtg).value(),
                                                            Units::MetersLength(ownship_true_dtg).value());
      }

      if (pilot_delay_model.IsPilotDelayOn()) {
         guidanceout.m_ias_command = m_im_speed_command_with_pilot_delay;
      } else {
         guidanceout.m_ias_command = m_im_speed_command_ias;
      }

      if (InternalObserver::getInstance()->outputNM()) {
         NMObserver &nm_observer = InternalObserver::getInstance()->GetNMObserver(ownship_aircraft_state.m_id);

         if (nm_observer.curr_NM == -2 && guidanceout.IsValid()) {
            nm_observer.curr_NM =
                  static_cast<int>(std::fabs(Units::NauticalMilesLength(-ownship_true_dtg).value()));
         }

         if (Units::abs(ownship_true_dtg) <=
             Units::NauticalMilesLength(nm_observer.curr_NM) &&
             guidanceout.IsValid()) {
            --nm_observer.curr_NM;

            double lval = LowLimit(ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(
                  m_ownship_reference_lookup_index));
            double hval = HighLimit(ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(
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
                  Units::MetersPerSecondSpeed(target_aircraft_state_projected_asg_adjusted.GetGroundSpeed()).value(),
                  lval, hval, ltas, htas);
         }
      }
   }
   return guidanceout;
}

void IMKinematicTimeBasedMaintain::CalculateIas(const Units::Length current_ownship_altitude,
                                                const DynamicsState &dynamics_state,
                                                const KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor,
                                                PilotDelay &pilot_delay) {
   BadaWithCalc &bada_calculator =
         ownship_kinematic_trajectory_predictor.GetKinematicDescent4dPredictor()->m_bada_calculator;

   Units::Speed rf_leg_limit_speed = Units::zero();
   if (m_has_rf_leg) {
      rf_leg_limit_speed = GetRFLegSpeedLimit(Units::MetersLength(
            ownship_kinematic_trajectory_predictor.GetVerticalPathDistanceByIndex(m_ownship_reference_lookup_index)));
   }

   m_im_speed_command_ias =
         LimitImSpeedCommand(m_im_speed_command_ias,
                             ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(
                                   m_ownship_reference_lookup_index),
                             Units::zero(), bada_calculator,
                             current_ownship_altitude,
                             dynamics_state.m_flap_configuration,
                             rf_leg_limit_speed);

   if (m_im_speed_command_ias != m_previous_im_speed_command_ias) {
      m_total_number_of_im_speed_changes++;
   }

   if (pilot_delay.IsPilotDelayOn()) {
      m_im_speed_command_with_pilot_delay =
            pilot_delay.UpdateIAS(m_previous_im_speed_command_ias, m_im_speed_command_ias, current_ownship_altitude,
                                  ownship_kinematic_trajectory_predictor.GetAltitudeAtFinalWaypoint());
   }
}

void IMKinematicTimeBasedMaintain::CalculateMach(const Units::Length current_ownship_altitude,
                                                 const Units::Speed true_airspeed_command,
                                                 const KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor,
                                                 PilotDelay &pilot_delay) {
   BadaWithCalc &bada_calculator =
         ownship_kinematic_trajectory_predictor.GetKinematicDescent4dPredictor()->m_bada_calculator;

   // Make sure velocity is within nominal limits (AAES-694)
   Units::Speed nominal_profile_ias = Units::MetersPerSecondSpeed(
         ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(m_ownship_reference_lookup_index));
   Units::Speed nominal_profile_tas =
         m_weather_prediction.getAtmosphere()->CAS2TAS(nominal_profile_ias, current_ownship_altitude);

   double estimated_mach =
         Units::MetersPerSecondSpeed(true_airspeed_command).value() /
         sqrt(GAMMA * R.value() *
              m_weather_prediction.getAtmosphere()->GetTemperature(current_ownship_altitude).value());

   double nominal_mach =
         Units::MetersPerSecondSpeed(nominal_profile_tas).value() /
         sqrt(GAMMA * R.value() *
              m_weather_prediction.getAtmosphere()->GetTemperature(current_ownship_altitude).value());

   estimated_mach = LimitImMachCommand(estimated_mach, nominal_mach, bada_calculator, current_ownship_altitude);

   if (estimated_mach != m_previous_reference_im_speed_command_mach) {
      m_total_number_of_im_speed_changes++;
   }

   if (pilot_delay.IsPilotDelayOn()) {
      m_im_speed_command_with_pilot_delay =
            pilot_delay.UpdateMach(m_previous_reference_im_speed_command_mach, estimated_mach, current_ownship_altitude,
                                   ownship_kinematic_trajectory_predictor.GetAltitudeAtFinalWaypoint());
   }

   m_previous_reference_im_speed_command_mach = estimated_mach;
   m_im_speed_command_ias = m_weather_prediction.getAtmosphere()->MachToIAS(estimated_mach, current_ownship_altitude);
}

void IMKinematicTimeBasedMaintain::IterationReset() {
   IMMaintain::IterationReset();
   m_measured_spacing_interval = Units::NegInfinity();
}

void IMKinematicTimeBasedMaintain::DumpParameters(const std::string &parameters_to_print) {
   IMMaintain::DumpParameters(parameters_to_print);
}
