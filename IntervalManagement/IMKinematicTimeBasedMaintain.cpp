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

#include "imalgs/IMKinematicTimeBasedMaintain.h"

#include <numeric>

#include "public/CustomMath.h"
#include "public/SimulationTime.h"
#include "public/AircraftCalculations.h"
#include "public/CoreUtils.h"
#include "imalgs/InternalObserver.h"

using namespace interval_management::open_source;

log4cplus::Logger IMKinematicTimeBasedMaintain::m_logger =
      log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("IMKinematicTimeBasedMaintain"));

IMKinematicTimeBasedMaintain::IMKinematicTimeBasedMaintain() { m_stage_of_im_operation = MAINTAIN; }

IMKinematicTimeBasedMaintain::~IMKinematicTimeBasedMaintain() = default;

aaesim::open_source::Guidance IMKinematicTimeBasedMaintain::Update(
      const aaesim::open_source::DynamicsState &dynamics_state,
      const interval_management::open_source::AircraftState &ownship_aircraft_state,
      const interval_management::open_source::AircraftState &target_aircraft_state_projected_asg_adjusted,
      const aaesim::open_source::KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor,
      const aaesim::open_source::Guidance &guidance_in,
      const std::vector<interval_management::open_source::AircraftState> &target_aircraft_state_history,
      const interval_management::open_source::AchievePointCalcs &ownship_achieve_point_calcs,
      const interval_management::open_source::AchievePointCalcs &traffic_reference_point_calcs,
      PilotDelay &pilot_delay_model, const Units::Length &target_kinematic_dtg_to_end_of_route) {
   /*
    * Developer's note: In this level of the algorithm, all uses of the /target state/
    * must be projected onto ownship's route prior to use. This includes all items
    * in the target_aircraft_state_history vector (they have not been projected). Some lower
    * level algorithms carry this load automatically
    */
   aaesim::open_source::Guidance guidance_out = guidance_in;
   guidance_out.SetValid(true);

   Units::MetersLength tmpdist;
   m_ownship_decrementing_distance_calculator.CalculateAlongPathDistanceFromPosition(
         ownship_aircraft_state.GetPositionX(), ownship_aircraft_state.GetPositionY(), tmpdist);
   const Units::Length ownship_estimated_dtg = tmpdist;

   // Get target's along-path position on ownship's route (adjusted for ASG)
   Units::Length target_projected_x = Units::zero();
   Units::Length target_projected_y = Units::zero();
   const bool foundProjectedPos = IMUtils::ProjectTargetPosition(
         Units::FeetLength(target_aircraft_state_projected_asg_adjusted.m_x),
         Units::FeetLength(target_aircraft_state_projected_asg_adjusted.m_y),
         ownship_kinematic_trajectory_predictor.GetHorizontalPath(), target_projected_x, target_projected_y, tmpdist);
   const Units::Length target_dtg_on_ownship_route = tmpdist;
   bool target_crossing_time_valid = true;
   const Units::Speed targetvelocity = target_aircraft_state_projected_asg_adjusted.GetGroundSpeed();
   Units::Time spacingerrorformaintainstats = Units::zero();
   Units::Time target_crossing_time = Units::zero();

   m_measured_spacing_interval = Units::NegInfinity();
   Units::Length projected_x;
   Units::Length projected_y;

   if (!target_aircraft_state_history.empty()) {
      std::vector<interval_management::open_source::AircraftState> history = target_aircraft_state_history;
      if (history.back().GetTimeStamp().value() < target_aircraft_state_projected_asg_adjusted.GetTimeStamp().value()) {
         // This is an edge case. The incoming state was extrapolated to get to the current
         // time. In this case, the history vector needs to have this extrapolated state also
         // in order for GetCrossingTime to operate consistently with this method.
         history.push_back(target_aircraft_state_projected_asg_adjusted);
      }
      target_crossing_time_valid =
            IMUtils::GetCrossingTime(ownship_estimated_dtg, history, m_ownship_distance_calculator,
                                     target_crossing_time, projected_x, projected_y);
      if (target_crossing_time_valid) {
         spacingerrorformaintainstats = Units::SecondsTime(ownship_aircraft_state.GetTimeStamp().value()) -
                                        target_crossing_time - m_im_clearance.GetAssignedTimeSpacingGoal();
         m_measured_spacing_interval = ownship_aircraft_state.GetTimeStamp() - target_crossing_time;
      }
   }

   if (m_ownship_reference_lookup_index == -1) {
      m_ownship_reference_lookup_index =
            static_cast<int>(ownship_kinematic_trajectory_predictor.GetVerticalPathDistances().size() - 1);
   }

   // if target distance is inside the range of the precalculated trajectory, use that to find the precalc index for the
   // +- 10% calculation
   if (target_aircraft_state_projected_asg_adjusted.GetId() != IMUtils::UNINITIALIZED_AIRCRAFT_ID &&
       foundProjectedPos) {
      if (Units::abs(ownship_estimated_dtg) <=
          Units::abs(Units::MetersLength(ownship_kinematic_trajectory_predictor.GetVerticalPathDistances().back()))) {
         m_ownship_reference_lookup_index =
               CoreUtils::FindNearestIndex(Units::MetersLength(ownship_estimated_dtg).value(),
                                           ownship_kinematic_trajectory_predictor.GetVerticalPathDistances());

         if (m_ownship_reference_lookup_index >
             ownship_kinematic_trajectory_predictor.GetVerticalPathDistances().size() - 1) {
            m_ownship_reference_lookup_index =
                  static_cast<int>(ownship_kinematic_trajectory_predictor.GetVerticalPathDistances().size() - 1);
         }
      } else {
         guidance_out.SetValid(false);
      }
   } else {
      guidance_out.SetValid(false);
   }

   if (guidance_out.IsValid()) {
      if (!target_aircraft_state_history.empty()) {
         InternalObserver::getInstance()->updateFinalGS(
               target_aircraft_state_projected_asg_adjusted.GetId(),
               Units::MetersPerSecondSpeed(target_aircraft_state_history.back().GetGroundSpeed()).value());
         InternalObserver::getInstance()->updateFinalGS(
               ownship_aircraft_state.GetId(),
               Units::MetersPerSecondSpeed(ownship_aircraft_state.GetGroundSpeed()).value());

         if (m_previous_reference_im_speed_command_tas == Units::zero()) {
            m_previous_reference_im_speed_command_tas = m_weather_prediction.getAtmosphere()->CAS2TAS(
                  Units::MetersPerSecondSpeed(
                        ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities().back()),
                  Units::MetersLength(ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes().back()));
            m_previous_im_speed_command_ias = Units::MetersPerSecondSpeed(
                  ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities().back());
            m_previous_reference_im_speed_command_mach =
                  Units::MetersPerSecondSpeed(m_previous_reference_im_speed_command_tas).value() /
                  sqrt(GAMMA * R.value() *
                       m_weather_prediction.getAtmosphere()
                             ->GetTemperature(Units::MetersLength(
                                   ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes().back()))
                             .value());
         }

         if (InternalObserver::getInstance()->GetRecordMaintainMetrics()) {
            if (Units::abs(ownship_estimated_dtg) < ownship_achieve_point_calcs.GetDistanceFromWaypoint()) {
               MaintainMetric &maintain_metric =
                     InternalObserver::getInstance()->GetMaintainMetric(ownship_aircraft_state.GetId());
               if (target_crossing_time_valid) {
                  maintain_metric.AddSpacingErrorSec(Units::SecondsTime(spacingerrorformaintainstats).value());
               }

               if (!maintain_metric.TimeAtAbpRecorded()) {
                  maintain_metric.SetTimeAtAbp(ownship_aircraft_state.GetTimeStamp().value());
               }
               maintain_metric.ComputeTotalMaintainTime(ownship_aircraft_state.GetTimeStamp().value());
            }
         }
      }

      Units::Speed gscommand =
            targetvelocity + (ownship_estimated_dtg - target_dtg_on_ownship_route) * m_maintain_control_gain;
      Units::Speed tascommand =
            sqrt(Units::sqr(gscommand -
                            Units::MetersPerSecondSpeed(ownship_aircraft_state.GetSensedWindParallelComponent())) +
                 Units::sqr(
                       Units::MetersPerSecondSpeed(ownship_aircraft_state.GetSensedWindPerpendicularComponent()))) /
            cos(ownship_aircraft_state.GetGamma().value());
      m_previous_reference_im_speed_command_tas = tascommand;

      if (tascommand < Units::zero()) {
         tascommand = Units::zero();
      }

      m_im_speed_command_ias =
            m_weather_prediction.getAtmosphere()->TAS2CAS(tascommand, Units::FeetLength(ownship_aircraft_state.m_z));
      m_unmodified_im_speed_command_ias = m_im_speed_command_ias;

      if (guidance_out.GetSelectedSpeed().GetSpeedType() == INDICATED_AIR_SPEED) {
         CalculateIas(Units::FeetLength(ownship_aircraft_state.m_z), dynamics_state,
                      ownship_kinematic_trajectory_predictor, pilot_delay_model);
      } else {
         CalculateMach(ownship_aircraft_state.GetPositionZ(), tascommand, ownship_kinematic_trajectory_predictor,
                       pilot_delay_model, dynamics_state.current_mass);
      }

      DoAlgorithmLogging(ownship_aircraft_state, target_aircraft_state_projected_asg_adjusted, targetvelocity,
                         ownship_estimated_dtg, target_dtg_on_ownship_route, gscommand, tascommand,
                         target_crossing_time, target_crossing_time_valid);

      m_previous_im_speed_command_ias = m_im_speed_command_ias;

      if (pilot_delay_model.IsPilotDelayOn()) {
         guidance_out.m_ias_command = m_im_speed_command_with_pilot_delay;
         if (guidance_out.GetSelectedSpeed().GetSpeedType() == MACH_SPEED) {
            const auto true_airspeed_equivalent =
                  m_weather_prediction.CAS2TAS(m_im_speed_command_ias, ownship_aircraft_state.GetPositionZ());
            const auto mach_equivalent =
                  m_weather_prediction.TAS2Mach(true_airspeed_equivalent, ownship_aircraft_state.GetPositionZ());
            guidance_out.SetMachCommand(mach_equivalent);
         }
      } else {
         guidance_out.m_ias_command = m_im_speed_command_ias;
         if (guidance_out.GetSelectedSpeed().GetSpeedType() == MACH_SPEED) {
            guidance_out.SetMachCommand(m_previous_reference_im_speed_command_mach);
         }
      }

      if (InternalObserver::getInstance()->outputNM()) {
         NMObserver &nm_observer = InternalObserver::getInstance()->GetNMObserver(ownship_aircraft_state.GetId());

         if (nm_observer.curr_NM == -2 && guidance_out.IsValid()) {
            nm_observer.curr_NM =
                  static_cast<int>(std::fabs(Units::NauticalMilesLength(-ownship_estimated_dtg).value()));
         }

         if (Units::abs(ownship_estimated_dtg) <= Units::NauticalMilesLength(nm_observer.curr_NM) &&
             guidance_out.IsValid()) {
            --nm_observer.curr_NM;

            double lval =
                  m_speed_limiter.LowLimit(ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(
                        m_ownship_reference_lookup_index));
            double hval =
                  m_speed_limiter.HighLimit(ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(
                        m_ownship_reference_lookup_index));

            double ltas = Units::MetersPerSecondSpeed(
                                m_weather_prediction.getAtmosphere()->CAS2TAS(
                                      Units::MetersPerSecondSpeed(lval), Units::FeetLength(ownship_aircraft_state.m_z)))
                                .value();
            double htas = Units::MetersPerSecondSpeed(
                                m_weather_prediction.getAtmosphere()->CAS2TAS(
                                      Units::MetersPerSecondSpeed(hval), Units::FeetLength(ownship_aircraft_state.m_z)))
                                .value();

            nm_observer.output_NM_values(
                  Units::MetersLength(ownship_estimated_dtg).value(),
                  Units::MetersLength(-ownship_estimated_dtg).value(), ownship_aircraft_state.GetTimeStamp().value(),
                  Units::MetersPerSecondSpeed(m_im_speed_command_ias).value(),
                  Units::MetersPerSecondSpeed(ownship_aircraft_state.GetGroundSpeed()).value(),
                  Units::MetersPerSecondSpeed(target_aircraft_state_projected_asg_adjusted.GetGroundSpeed()).value(),
                  lval, hval, ltas, htas);
         }
      }
   }
   return guidance_out;
}

void IMKinematicTimeBasedMaintain::CalculateIas(
      const Units::Length current_ownship_altitude, const aaesim::open_source::DynamicsState &dynamics_state,
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

void IMKinematicTimeBasedMaintain::CalculateMach(
      const Units::Length current_ownship_altitude, const Units::Speed true_airspeed_command,
      const aaesim::open_source::KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor,
      PilotDelay &pilot_delay, const Units::Mass current_mass) {

   // Make sure velocity is within nominal limits (AAES-694)
   Units::Speed nominal_profile_ias = Units::MetersPerSecondSpeed(
         ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(m_ownship_reference_lookup_index));
   Units::Speed nominal_profile_tas =
         m_weather_prediction.getAtmosphere()->CAS2TAS(nominal_profile_ias, current_ownship_altitude);

   BoundedValue<double, 0, 2> estimated_mach = BoundedValue<double, 0, 2>(
         Units::MetersPerSecondSpeed(true_airspeed_command).value() /
         sqrt(GAMMA * R.value() *
              m_weather_prediction.getAtmosphere()->GetTemperature(current_ownship_altitude).value()));

   BoundedValue<double, 0, 2> nominal_mach = BoundedValue<double, 0, 2>(
         Units::MetersPerSecondSpeed(nominal_profile_tas).value() /
         sqrt(GAMMA * R.value() *
              m_weather_prediction.getAtmosphere()->GetTemperature(current_ownship_altitude).value()));

   estimated_mach = m_speed_limiter.LimitMachCommand(
         BoundedValue<double, 0, 2>(m_previous_reference_im_speed_command_mach), estimated_mach, nominal_mach,
         current_mass, current_ownship_altitude, m_weather_prediction);

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
