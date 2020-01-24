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

#include <stdexcept>

#include "aaesim/SimpleAircraft.h"
#include "imalgs/IMTimeBasedAchieve.h"
#include "math/CustomMath.h"
#include "public/AircraftCalculations.h"
#include "public/CoreUtils.h"
#include "imalgs/IMAircraft.h"

using namespace std;

log4cplus::Logger IMTimeBasedAchieve::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("IMTimeBasedAchieve"));


IMTimeBasedAchieve::IMTimeBasedAchieve() {
   if (m_im_kinematic_time_based_maintain == nullptr) {
      m_im_kinematic_time_based_maintain = std::make_shared<IMKinematicTimeBasedMaintain>();
   }
}

IMTimeBasedAchieve::IMTimeBasedAchieve(const IMTimeBasedAchieve &obj) {
   if (m_im_kinematic_time_based_maintain == nullptr) {
      m_im_kinematic_time_based_maintain = std::make_shared<IMKinematicTimeBasedMaintain>();
   }
   Copy(obj);
}

IMTimeBasedAchieve::~IMTimeBasedAchieve() = default;

IMTimeBasedAchieve &IMTimeBasedAchieve::operator=(const IMTimeBasedAchieve &obj) {
   if (this != &obj) {
      Copy(obj);
   }

   return *this;
}

void IMTimeBasedAchieve::Copy(const IMTimeBasedAchieve &obj) {
   IMKinematicAchieve::Copy(obj);
   m_im_kinematic_time_based_maintain = obj.m_im_kinematic_time_based_maintain;
   m_assigned_spacing_goal = obj.m_assigned_spacing_goal;
   m_predicted_spacing_interval = obj.m_predicted_spacing_interval;
   m_measured_spacing_interval = obj.m_measured_spacing_interval;
}

void IMTimeBasedAchieve::IterationReset() {
   IMKinematicAchieve::IterationReset();

   m_predicted_spacing_interval = Units::NegInfinity();
   m_measured_spacing_interval = Units::NegInfinity();

   if (m_im_kinematic_time_based_maintain == nullptr) {
      m_im_kinematic_time_based_maintain = std::make_shared<IMKinematicTimeBasedMaintain>();
   }
   m_im_kinematic_time_based_maintain->IterationReset();
}

bool IMTimeBasedAchieve::load(DecodedStream *input) {
   bool loaded = IMKinematicAchieve::load(input);

   m_assigned_spacing_goal = Units::SecondsTime(m_assigned_spacing_goal_from_input_file);

   if (m_im_kinematic_time_based_maintain == nullptr) {
      m_im_kinematic_time_based_maintain = std::make_shared<IMKinematicTimeBasedMaintain>();
   }
   m_im_kinematic_time_based_maintain->InitializeScenario(this, m_maintain_control_gain);

   return loaded;
}

Guidance IMTimeBasedAchieve::Update(const Guidance &previous_im_guidance,
                                    const DynamicsState &three_dof_dynamics_state,
                                    const AircraftState &current_ownship_state,
                                    const AircraftState &current_target_state,
                                    const vector<AircraftState> &target_adsb_history) {
   Guidance guidanceout = IMKinematicAchieve::Update(previous_im_guidance, three_dof_dynamics_state,
                                                     current_ownship_state, current_target_state, target_adsb_history);

   if (!IsImOperationComplete() && guidanceout.IsValid()) {
      Units::Time reference_ttg = Units::zero();
      m_ownship_reference_cas = Units::ZERO_SPEED;
      Units::Time ownrefttgtoend = Units::zero();
      Units::Length reference_distance = Units::zero();

      m_im_speed_command_ias = m_previous_im_speed_command_ias;

      if (m_target_reference_lookup_index == -1) {
         m_target_reference_lookup_index =
               static_cast<int>(m_target_kinematic_trajectory_predictor.GetVerticalPathDistances().size() - 1);
         m_ownship_reference_lookup_index =
               static_cast<int>(m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances().size() - 1);
         m_reference_precalc_index =
               static_cast<int>(m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances().size() - 1);
      }

      if (InAchieveStage()) {
         m_stage_of_im_operation = ACHIEVE;

         InternalObserver::getInstance()->updateFinalGS(current_target_state.m_id, Units::MetersPerSecondSpeed(
               AircraftCalculations::GsAtACS(target_adsb_history.back())).value());
         InternalObserver::getInstance()->updateFinalGS(current_ownship_state.m_id, Units::MetersPerSecondSpeed(
               AircraftCalculations::GsAtACS(current_ownship_state)).value());

         bool is_crossing_time_valid = false;

         if (m_target_aircraft_exists && !IsTargetPassedTrp()) {
            m_target_reference_lookup_index =
                  CoreUtils::FindNearestIndex(Units::MetersLength(m_target_kinematic_dtg_to_last_waypoint).value(),
                                              m_target_kinematic_trajectory_predictor.GetVerticalPathDistances());

            if (m_target_reference_lookup_index == 0) {
               m_target_ttg_to_end_of_route = Units::SecondsTime(
                     m_target_kinematic_trajectory_predictor.GetVerticalPathTimeByIndex(0));
               m_target_reference_altitude = Units::MetersLength(
                     m_target_kinematic_trajectory_predictor.GetVerticalPathAltitudeByIndex(0));
            } else {
               m_target_ttg_to_end_of_route = Units::SecondsTime(
                     CoreUtils::LinearlyInterpolate(m_target_reference_lookup_index,
                                                    Units::MetersLength(
                                                          m_target_kinematic_dtg_to_last_waypoint).value(),
                                                    m_target_kinematic_trajectory_predictor.GetVerticalPathDistances(),
                                                    m_target_kinematic_trajectory_predictor.GetVerticalPathTimes()));
               m_target_reference_altitude = Units::MetersLength(
                     CoreUtils::LinearlyInterpolate(m_target_reference_lookup_index,
                           Units::MetersLength(m_target_kinematic_dtg_to_last_waypoint).value(),
                           m_target_kinematic_trajectory_predictor.GetVerticalPathDistances(),
                           m_target_kinematic_trajectory_predictor.GetVerticalPathAltitudes()));
            }

            m_target_ttg_to_trp = m_target_ttg_to_end_of_route -
                                  m_target_kinematic_traffic_reference_point_calcs.GetTimeToGoToWaypoint();
            reference_ttg = m_target_ttg_to_trp + m_assigned_spacing_goal;

         } else if ((target_adsb_history.size() >= 2) || IsTargetPassedTrp()) {
            is_crossing_time_valid = IMUtils::GetCrossingTime(
                  m_target_kinematic_traffic_reference_point_calcs.GetDistanceFromWaypoint(),
                  target_adsb_history,
                  m_target_kinematic_trajectory_predictor.GetHorizontalPath(),
                  m_target_trp_crossing_time);
            if (!is_crossing_time_valid) {
               LOG4CPLUS_WARN(IMTimeBasedAchieve::m_logger, "IMTimeBasedAchieve::update ac " << current_ownship_state.m_id
                                                                                             << "  time "
                                                                                             << current_ownship_state.m_time
                                                                                             << endl
                                                                                             << "Non calculation of targettrpcrossingtime in GetCrossingTime;"
                                                                                             << endl
                                                                                             << "referencettg calculation suspicious"
                                                                                             << endl);
            }
            reference_ttg =
                  m_target_trp_crossing_time + m_assigned_spacing_goal -
                  Units::SecondsTime(current_ownship_state.m_time);

            m_target_ttg_to_trp = Units::zero();
         }

         if (m_previous_im_speed_command_ias == Units::zero()) {
            m_previous_reference_im_speed_command_tas = m_weather_prediction.getAtmosphere()->CAS2TAS(
                  Units::MetersPerSecondSpeed(
                        m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities().back()),
                  Units::MetersLength(
                        m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes().back()));
            m_previous_im_speed_command_ias = Units::MetersPerSecondSpeed(
                  m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities().back());

            m_previous_reference_im_speed_command_mach =
                  Units::MetersPerSecondSpeed(m_previous_reference_im_speed_command_tas).value() /
                  sqrt(GAMMA * R.value() * m_weather_prediction.getAtmosphere()->GetTemperature(
                        Units::MetersLength(
                              m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes().back())).value());
         }

         if (Units::abs(m_ownship_kinematic_dtg_to_ptp) <= Units::abs(Units::MetersLength(
               m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances().back()))) {
            m_ownship_reference_lookup_index = CoreUtils::FindNearestIndex(
                  Units::MetersLength(m_ownship_kinematic_dtg_to_ptp).value(),
                  m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances());


            if (m_ownship_reference_lookup_index == 0) {
               m_ownship_reference_ttg_to_ptp = Units::SecondsTime(
                     m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimeByIndex(0));
               m_ownship_ttg_to_abp =
                     m_ownship_reference_ttg_to_ptp - m_ownship_kinematic_achieve_by_calcs.GetTimeToGoToWaypoint();
            } else {
               m_ownship_reference_ttg_to_ptp = Units::SecondsTime(
                     CoreUtils::LinearlyInterpolate(m_ownship_reference_lookup_index,
                                                    Units::MetersLength(
                                                          m_ownship_kinematic_dtg_to_ptp).value(),
                                                    m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances(),
                                                    m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes()));
               m_ownship_ttg_to_abp =
                     m_ownship_reference_ttg_to_ptp - m_ownship_kinematic_achieve_by_calcs.GetTimeToGoToWaypoint();
            }
         }

         LOG4CPLUS_TRACE(m_logger, "Ownship TTG to ABP: " << Units::SecondsTime(m_ownship_ttg_to_abp).value());
         if (is_crossing_time_valid) {
            Units::Time tmp1 = Units::SecondsTime(current_ownship_state.m_time) + m_ownship_ttg_to_abp;
            m_predicted_spacing_interval = tmp1 - m_target_trp_crossing_time;
         } else {
            m_predicted_spacing_interval = m_ownship_ttg_to_abp - m_target_ttg_to_trp;
         }
         m_measured_spacing_interval = Units::NegInfinity();

         if (m_threshold_flag) {
            Units::Time error_threshold = GetErrorThreshold(Units::abs(m_ownship_kinematic_dtg_to_ptp) -
                                                            m_ownship_kinematic_achieve_by_calcs.GetDistanceFromWaypoint());

            if (abs(m_ownship_ttg_to_abp - reference_ttg) > error_threshold) {
               reference_ttg = reference_ttg + error_threshold * (m_ownship_ttg_to_abp - reference_ttg) /
                                               abs(m_ownship_ttg_to_abp - reference_ttg);
            } else {
               reference_ttg = m_ownship_ttg_to_abp;
            }
         }

         ownrefttgtoend = reference_ttg + m_ownship_kinematic_achieve_by_calcs.GetTimeToGoToWaypoint();

         m_reference_precalc_index = CoreUtils::FindNearestIndex(Units::SecondsTime(ownrefttgtoend).value(),
                                                                 m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes());

         if (m_reference_precalc_index >=
             static_cast<int>(m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes().size() - 1)) {
            // NEED:Change the calculations here to compute a point before the start.
            // Use gs not v in the calculation.

            // It is possible for FindNearestIndex to return the last index in the tested vector if the tested value is greater than
            // vector.size() - 2. We catch those cases here, but this should be reworked to make the logic more robust and understandable.
            if (Units::SecondsTime(ownrefttgtoend).value() >=
                m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes().back()) {
               m_reference_precalc_index =
                     static_cast<int>(m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes().size() - 1);
               reference_distance = Units::MetersLength(
                     -m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances().back());
               m_ownship_reference_cas =
                     Units::MetersPerSecondSpeed(
                           m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities().back());
               m_ownship_reference_altitude =
                     Units::MetersLength(
                           m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes().back());
            } else {
               reference_distance = Units::MetersLength(
                     -CoreUtils::LinearlyInterpolate(m_reference_precalc_index,
                           Units::SecondsTime(ownrefttgtoend).value(),
                           m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes(),
                           m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances()));

               m_ownship_reference_cas = Units::MetersPerSecondSpeed(
                     CoreUtils::LinearlyInterpolate(m_reference_precalc_index,
                           Units::SecondsTime(ownrefttgtoend).value(),
                           m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes(),
                           m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities()));
               m_ownship_reference_altitude = Units::MetersLength(
                     CoreUtils::LinearlyInterpolate(m_reference_precalc_index,
                           Units::SecondsTime(ownrefttgtoend).value(),
                           m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes(),
                           m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes()));
            }
         } else if (m_reference_precalc_index == 0) {
            reference_distance = Units::MetersLength(-m_ownship_kinematic_trajectory_predictor
                  .GetVerticalPathDistanceByIndex(0));
            m_ownship_reference_cas = Units::MetersPerSecondSpeed(
                  m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(0));
            m_ownship_reference_altitude = Units::MetersLength(
                  m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudeByIndex(0));

         } else {
            reference_distance = Units::MetersLength(
                  -CoreUtils::LinearlyInterpolate(m_reference_precalc_index,
                        Units::SecondsTime(ownrefttgtoend).value(),
                        m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes(),
                        m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances()));

            m_ownship_reference_cas = Units::MetersPerSecondSpeed(
                  CoreUtils::LinearlyInterpolate(m_reference_precalc_index,
                        Units::SecondsTime(ownrefttgtoend).value(),
                        m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes(),
                        m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities()));
            m_ownship_reference_altitude = Units::MetersLength(
                  CoreUtils::LinearlyInterpolate(m_reference_precalc_index,
                        Units::SecondsTime(ownrefttgtoend).value(),
                        m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes(),
                        m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes()));
         }

         const Units::Length temp = reference_distance + m_ownship_kinematic_dtg_to_ptp;

         m_im_speed_command_ias = m_ownship_reference_cas + temp * m_achieve_control_gain;

         if (m_im_speed_command_ias < Units::zero()) {
            m_im_speed_command_ias = Units::zero();
         }

         m_unmodified_im_speed_command_ias = m_im_speed_command_ias;

         if (IsOwnshipBelowTransitionAltitude(Units::FeetLength(current_ownship_state.m_z))) {
            CalculateIas(Units::FeetLength(current_ownship_state.m_z), three_dof_dynamics_state);
         } else {
            CalculateMach(reference_ttg, Units::FeetLength(current_ownship_state.m_z));
         }

         m_previous_im_speed_command_ias = m_im_speed_command_ias;
         Units::Speed tascommand =
               m_weather_prediction.getAtmosphere()->CAS2TAS(m_im_speed_command_ias,
                                                             Units::FeetLength(current_ownship_state.m_z));

         RecordInternalObserverMetrics(current_ownship_state, current_target_state, three_dof_dynamics_state,
                                       m_unmodified_im_speed_command_ias, tascommand, m_ownship_reference_cas,
                                       reference_distance, guidanceout);

         if (m_pilot_delay.IsPilotDelayOn()) {
            guidanceout.m_ias_command = m_im_speed_command_with_pilot_delay;
         } else {
            guidanceout.m_ias_command = m_im_speed_command_ias;
         }
      } else {
         m_stage_of_im_operation = MAINTAIN;

         m_ownship_ttg_to_abp = Units::zero();
         m_ownship_kinematic_dtg_to_abp = Units::zero();
         m_predicted_spacing_interval = Units::NegInfinity();

         Units::Time target_time = Units::SecondsTime(current_ownship_state.m_time) - m_assigned_spacing_goal;

         bool built;
         AircraftState target_state_projected_asg_adjusted =
               IMUtils::GetProjectedTargetState(target_adsb_history,
                                                m_ownship_kinematic_trajectory_predictor.GetHorizontalPath(),
                                                target_time,
                                                Units::RadiansAngle(current_ownship_state.GetHeadingCcwFromEastRadians() + Units::PI_RADIANS_ANGLE),
                                                built);

         if (built) {
            if (!m_transitioned_to_maintain) {
               m_im_kinematic_time_based_maintain->Prepare(m_previous_reference_im_speed_command_tas,
                                                           m_previous_im_speed_command_ias,
                                                           m_previous_reference_im_speed_command_mach,
                                                           m_tangent_plane_sequence,
                                                           m_ownship_kinematic_trajectory_predictor,
                                                           target_adsb_history,
                                                           m_im_clearance,
                                                           m_has_rf_leg,
                                                           m_rfleg_limits);

               m_transitioned_to_maintain = true;
            }

            guidanceout = m_im_kinematic_time_based_maintain->Update(three_dof_dynamics_state,
                                                                     current_ownship_state,
                                                                     target_state_projected_asg_adjusted,
                                                                     m_ownship_kinematic_trajectory_predictor,
                                                                     previous_im_guidance,
                                                                     target_adsb_history,
                                                                     m_ownship_kinematic_achieve_by_calcs,
                                                                     m_target_kinematic_traffic_reference_point_calcs,
                                                                     m_pilot_delay,
                                                                     m_target_kinematic_dtg_to_last_waypoint);

            SetActiveFilter(m_im_kinematic_time_based_maintain->GetActiveFilter());
            m_measured_spacing_interval = Units::SecondsTime(m_im_kinematic_time_based_maintain->GetMsi());
            m_im_speed_command_ias = m_im_kinematic_time_based_maintain->GetImSpeedCommandIas();
            m_im_speed_command_with_pilot_delay = m_im_kinematic_time_based_maintain->GetDelayedImSpeedCommandIas();

            m_previous_im_speed_command_ias = m_im_kinematic_time_based_maintain->GetPreviousSpeedCommandIas();
         } else {
            m_ownship_reference_lookup_index =
                  CoreUtils::FindNearestIndex(Units::MetersLength(m_ownship_kinematic_dtg_to_ptp).value(),
                                              m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances());

            if (m_ownship_reference_lookup_index > 0) {
               const Units::Speed nominal_ias =
                     Units::MetersPerSecondSpeed(m_ownship_kinematic_trajectory_predictor
                                                       .GetVerticalPathVelocityByIndex
                                                             (m_ownship_reference_lookup_index));
               m_previous_im_speed_command_ias = m_im_kinematic_time_based_maintain->GetPreviousSpeedCommandIas();
               if (m_previous_im_speed_command_ias != Units::zero() &&
                   m_previous_im_speed_command_ias < nominal_ias) {
                  guidanceout.m_ias_command = m_previous_im_speed_command_ias;
               } else {
                  guidanceout.m_ias_command = nominal_ias;
                  m_previous_im_speed_command_ias = nominal_ias;
               }
               m_im_speed_command_ias = guidanceout.m_ias_command;
               m_im_speed_command_with_pilot_delay = guidanceout.m_ias_command;
            } else {
               guidanceout.SetValid(false);
               m_is_guidance_valid = false;
            }
         }
      }
   } else {
      guidanceout.SetValid(false);
      m_is_guidance_valid = false;
   }
   return guidanceout;
}

void IMTimeBasedAchieve::CalculateIas(const Units::Length current_ownship_altitude,
                                      const DynamicsState &three_dof_dynamics_state) {
   BadaWithCalc &bada_calculator =
         m_ownship_kinematic_trajectory_predictor.GetKinematicDescent4dPredictor()->m_bada_calculator;

   Units::Speed rf_leg_limit_speed = Units::zero();
   if (m_has_rf_leg) {
      rf_leg_limit_speed = GetRFLegSpeedLimit(m_ownship_kinematic_dtg_to_ptp);
   }

   m_im_speed_command_ias =
         LimitImSpeedCommand(m_im_speed_command_ias,
                             m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex
                                   (m_ownship_reference_lookup_index),
                             m_ownship_kinematic_dtg_to_abp, bada_calculator,
                             current_ownship_altitude,
                             three_dof_dynamics_state.m_flap_configuration,
                             rf_leg_limit_speed);

   if (m_im_speed_command_ias != m_previous_im_speed_command_ias) {
      m_total_number_of_im_speed_changes++;
   }

   if (m_pilot_delay.IsPilotDelayOn()) {
      m_im_speed_command_with_pilot_delay = m_pilot_delay.UpdateIAS(m_previous_im_speed_command_ias,
                                                                    m_im_speed_command_ias,
                                                                    current_ownship_altitude,
                                                                    m_ownship_kinematic_trajectory_predictor.GetAltitudeAtFinalWaypoint());
   }
}

void IMTimeBasedAchieve::CalculateMach(const Units::Time reference_ttg,
                                       const Units::Length current_ownship_altitude) {
   BadaWithCalc &bada_calculator =
         m_ownship_kinematic_trajectory_predictor.GetKinematicDescent4dPredictor()->m_bada_calculator;

   Units::Speed temptrue =
         m_weather_prediction.getAtmosphere()->CAS2TAS(m_im_speed_command_ias, current_ownship_altitude);

   double estimated_mach = Units::MetersPerSecondSpeed(temptrue).value() /
                           sqrt(GAMMA * R.value() *
                                m_weather_prediction.getAtmosphere()->GetTemperature(current_ownship_altitude).value());

   // for nominal limits (AAES-694)
   Units::Speed nominal_ias = Units::MetersPerSecondSpeed(
         m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(
               m_ownship_reference_lookup_index));
   Units::Speed nominal_tas =
         m_weather_prediction.getAtmosphere()->CAS2TAS(nominal_ias, current_ownship_altitude);
   double nominal_mach = Units::MetersPerSecondSpeed(nominal_tas).value() /
                         sqrt(GAMMA * R.value() *
                              m_weather_prediction.getAtmosphere()->GetTemperature(current_ownship_altitude).value());

   estimated_mach = LimitImMachCommand(estimated_mach, nominal_mach, bada_calculator, current_ownship_altitude);

   if (estimated_mach != m_previous_reference_im_speed_command_mach) {
      m_total_number_of_im_speed_changes++;
   }

   if (m_pilot_delay.IsPilotDelayOn()) {
      m_im_speed_command_with_pilot_delay =
            m_pilot_delay.UpdateMach(m_previous_reference_im_speed_command_mach, estimated_mach,
                                     current_ownship_altitude,
                                     m_ownship_kinematic_trajectory_predictor.GetAltitudeAtFinalWaypoint());
   }

   m_previous_reference_im_speed_command_mach = estimated_mach;
   m_im_speed_command_ias = m_weather_prediction.getAtmosphere()->MachToIAS(estimated_mach, current_ownship_altitude);
}

void IMTimeBasedAchieve::RecordInternalObserverMetrics(const AircraftState &current_ownship_state,
                                                       const AircraftState &current_target_state,
                                                       const DynamicsState &dynamics_state,
                                                       const Units::Speed unmodified_ias,
                                                       const Units::Speed tas_command,
                                                       const Units::Speed reference_velocity,
                                                       const Units::Length reference_distance,
                                                       const Guidance &guidance) {
   if (InternalObserver::getInstance()->GetScenarioIter() >= 0) {
      InternalObserver::getInstance()->IM_command_output(current_ownship_state.m_id,
                                                         current_ownship_state.m_time,
                                                         current_ownship_state.m_z,
                                                         Units::MetersPerSecondSpeed(dynamics_state.V).value(),
                                                         Units::MetersPerSecondSpeed(
                                                               current_ownship_state.GetGroundSpeed()).value(),
                                                         Units::MetersPerSecondSpeed(
                                                               m_im_speed_command_ias).value(),
                                                         Units::MetersPerSecondSpeed(unmodified_ias).value(),
                                                         Units::MetersPerSecondSpeed(tas_command).value(),
                                                         Units::MetersPerSecondSpeed(reference_velocity).value(),
                                                         Units::MetersLength(reference_distance).value(),
                                                         Units::MetersLength(
                                                               -m_ownship_kinematic_dtg_to_ptp).value(),
                                                         Units::MetersLength(-m_ownship_kinetic_dtg_to_ptp).value());
   }


   if (InternalObserver::getInstance()->outputNM()) {
      NMObserver &nm_observer = InternalObserver::getInstance()->GetNMObserver(current_ownship_state.m_id);

      if (nm_observer.curr_NM == -2 &&
          guidance.IsValid()) {
         nm_observer.curr_NM =
               static_cast<int>(Units::NauticalMilesLength(Units::abs(m_ownship_kinematic_dtg_to_ptp)).value());
      }


      if (Units::abs(m_ownship_kinematic_dtg_to_ptp) <= Units::NauticalMilesLength(
            nm_observer.curr_NM) &&
          guidance.IsValid()) {
         nm_observer.curr_NM--;

         double lval = LowLimit(
               m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex
                     (m_ownship_reference_lookup_index));
         double hval = HighLimit(
               m_ownship_kinematic_trajectory_predictor
                     .GetVerticalPathVelocityByIndex(m_ownship_reference_lookup_index));

         double ltas = Units::MetersPerSecondSpeed(
               m_weather_prediction.getAtmosphere()->CAS2TAS(Units::MetersPerSecondSpeed(lval),
                                                             Units::FeetLength(current_ownship_state.m_z))).value();
         double htas = Units::MetersPerSecondSpeed(
               m_weather_prediction.getAtmosphere()->CAS2TAS(Units::MetersPerSecondSpeed(hval),
                                                             Units::FeetLength(current_ownship_state.m_z))).value();

         nm_observer.output_NM_values(
               Units::MetersLength(m_ownship_kinematic_dtg_to_ptp).value(),
               Units::MetersLength(m_ownship_kinetic_dtg_to_ptp).value(), current_ownship_state.m_time,
               Units::MetersPerSecondSpeed(m_im_speed_command_ias).value(),
               Units::MetersPerSecondSpeed(current_ownship_state.GetGroundSpeed()).value(),
               Units::MetersPerSecondSpeed(current_target_state.GetGroundSpeed()).value(),
               lval, hval, ltas, htas);
      }
   }

   InternalObserver::getInstance()->addAchieveRcd(static_cast<size_t>(current_ownship_state.m_id),
                                                  current_ownship_state.m_time,
                                                  Units::SecondsTime(m_target_ttg_to_trp).value(),
                                                  Units::SecondsTime(m_ownship_ttg_to_abp).value(),
                                                  Units::MetersLength(-m_ownship_kinematic_dtg_to_ptp).value(),
                                                  Units::MetersLength(reference_distance).value());
}

void IMTimeBasedAchieve::DumpParameters(const string &parameters_to_print) {
   LOG4CPLUS_DEBUG(IMTimeBasedAchieve::m_logger,
                   "--------------------------------------------------------------------" << endl);
   LOG4CPLUS_DEBUG(IMTimeBasedAchieve::m_logger, parameters_to_print.c_str() << endl << endl);

   IMKinematicAchieve::DumpParameters(parameters_to_print);

   LOG4CPLUS_DEBUG(IMTimeBasedAchieve::m_logger,
                   "--------------------------------------------------------------------" << endl);
}

const Units::Speed IMTimeBasedAchieve::GetImSpeedCommandIas() const {
   if (m_stage_of_im_operation == ACHIEVE) {
      return IMAlgorithm::GetImSpeedCommandIas();
   } else {
      return m_im_kinematic_time_based_maintain->GetImSpeedCommandIas();
   }
}

