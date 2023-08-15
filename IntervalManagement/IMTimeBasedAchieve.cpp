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

#include "imalgs/IMTimeBasedAchieve.h"

#include <stdexcept>

#include "public/CustomMath.h"
#include "public/AircraftCalculations.h"
#include "public/CoreUtils.h"

using namespace std;
using namespace aaesim::open_source;
using namespace interval_management::open_source;

log4cplus::Logger IMTimeBasedAchieve::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("IMTimeBasedAchieve"));

IMTimeBasedAchieve::IMTimeBasedAchieve()
   : m_target_state_at_traffic_alignment(),
     m_target_state_at_cdti_initiate_signal_receipt(),
     m_target_dtg_at_traffic_alignment(Units::Infinity()),
     m_time_since_traffic_alignment(Units::zero()),
     m_time_at_traffic_alignment(Units::Zero()) {
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
   m_target_state_at_traffic_alignment = obj.m_target_state_at_traffic_alignment;
   m_target_state_at_cdti_initiate_signal_receipt = obj.m_target_state_at_cdti_initiate_signal_receipt;
   m_target_dtg_at_traffic_alignment = obj.m_target_dtg_at_traffic_alignment;
   m_time_since_traffic_alignment = obj.m_time_since_traffic_alignment;
   m_time_at_traffic_alignment = obj.m_time_at_traffic_alignment;
   m_assigned_spacing_goal = obj.m_assigned_spacing_goal;
   m_predicted_spacing_interval = obj.m_predicted_spacing_interval;
   m_measured_spacing_interval = obj.m_measured_spacing_interval;
}

void IMTimeBasedAchieve::IterationReset() {
   IMKinematicAchieve::IterationReset();

   m_target_state_at_traffic_alignment = interval_management::open_source::AircraftState();
   m_target_state_at_cdti_initiate_signal_receipt = interval_management::open_source::AircraftState();

   m_target_dtg_at_traffic_alignment = Units::Infinity();
   m_time_since_traffic_alignment = Units::zero();
   m_time_at_traffic_alignment = Units::zero();
   m_predicted_spacing_interval = Units::NegInfinity();
   m_measured_spacing_interval = Units::NegInfinity();

   m_target_state_projected_asg_adjusted = interval_management::open_source::AircraftState();

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

aaesim::open_source::Guidance IMTimeBasedAchieve::Update(
      const aaesim::open_source::Guidance &previous_im_guidance,
      const aaesim::open_source::DynamicsState &three_dof_dynamics_state,
      const interval_management::open_source::AircraftState &current_ownship_state,
      const interval_management::open_source::AircraftState &current_target_state,
      const vector<interval_management::open_source::AircraftState> &target_adsb_history) {
   aaesim::open_source::Guidance guidance_out =
         IMKinematicAchieve::Update(previous_im_guidance, three_dof_dynamics_state, current_ownship_state,
                                    current_target_state, target_adsb_history);

   if (!IsImOperationComplete() && guidance_out.IsValid()) {
      m_ownship_reference_cas = Units::ZERO_SPEED;
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
         return HandleAchieveStage(current_ownship_state, current_target_state, target_adsb_history,
                                   three_dof_dynamics_state, guidance_out);
      } else {
         m_stage_of_im_operation = MAINTAIN;

         if (m_im_clearance.GetClearanceType() == IMClearance::CAPTURE) {
            if (!IsTargetAligned()) {
               if (!target_adsb_history.empty()) {
                  TestForTrafficAlignment(current_ownship_state, target_adsb_history);
               }

               AircraftSpeed aircraft_speed = guidance_out.GetSelectedSpeed();
               if (aircraft_speed.GetSpeedType() != INDICATED_AIR_SPEED) {
                  LOG4CPLUS_FATAL(m_logger,
                                  "AircraftSpeed SpeedValueType not INDICATED_AIR_SPEED.  "
                                  "Unable to update guidance for CAPTURE clearance");
               }

               m_im_speed_command_ias = Units::KnotsSpeed(aircraft_speed.GetValue());
               m_previous_im_speed_command_ias = m_im_speed_command_ias;
               guidance_out.m_ias_command = m_im_speed_command_ias;
            } else {
               return HandleMaintainStage(current_ownship_state, current_target_state, target_adsb_history,
                                          three_dof_dynamics_state, previous_im_guidance, guidance_out);
            }
         } else {
            return HandleMaintainStage(current_ownship_state, current_target_state, target_adsb_history,
                                       three_dof_dynamics_state, previous_im_guidance, guidance_out);
         }
      }
   }

   return guidance_out;
}

aaesim::open_source::Guidance IMTimeBasedAchieve::HandleAchieveStage(
      const interval_management::open_source::AircraftState &current_ownship_state,
      const interval_management::open_source::AircraftState &current_target_state,
      const vector<interval_management::open_source::AircraftState> &target_adsb_history,
      const aaesim::open_source::DynamicsState &three_dof_dynamics_state, aaesim::open_source::Guidance &guidance_out) {
   m_stage_of_im_operation = ACHIEVE;

   Units::Time reference_ttg = Units::zero();

   Units::Time ownrefttgtoend;
   Units::Length reference_distance;

   InternalObserver::getInstance()->updateFinalGS(
         current_target_state.GetId(),
         Units::MetersPerSecondSpeed(target_adsb_history.back().GetGroundSpeed()).value());
   InternalObserver::getInstance()->updateFinalGS(
         current_ownship_state.GetId(), Units::MetersPerSecondSpeed(current_ownship_state.GetGroundSpeed()).value());

   bool is_crossing_time_valid = false;

   if (m_target_aircraft_exists && !IsTargetPassedTrp()) {
      m_target_reference_lookup_index =
            CoreUtils::FindNearestIndex(Units::MetersLength(m_target_kinematic_dtg_to_last_waypoint).value(),
                                        m_target_kinematic_trajectory_predictor.GetVerticalPathDistances());

      if (m_target_reference_lookup_index == 0) {
         m_target_ttg_to_end_of_route =
               Units::SecondsTime(m_target_kinematic_trajectory_predictor.GetVerticalPathTimeByIndex(0));
         m_target_reference_altitude =
               Units::MetersLength(m_target_kinematic_trajectory_predictor.GetVerticalPathAltitudeByIndex(0));
         m_target_reference_ias =
               Units::MetersPerSecondSpeed(m_target_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(0));
         m_target_reference_gs = Units::MetersPerSecondSpeed(0);
      } else {
         m_target_ttg_to_end_of_route = Units::SecondsTime(CoreUtils::LinearlyInterpolate(
               m_target_reference_lookup_index, Units::MetersLength(m_target_kinematic_dtg_to_last_waypoint).value(),
               m_target_kinematic_trajectory_predictor.GetVerticalPathDistances(),
               m_target_kinematic_trajectory_predictor.GetVerticalPathTimes()));
         m_target_reference_altitude = Units::MetersLength(CoreUtils::LinearlyInterpolate(
               m_target_reference_lookup_index, Units::MetersLength(m_target_kinematic_dtg_to_last_waypoint).value(),
               m_target_kinematic_trajectory_predictor.GetVerticalPathDistances(),
               m_target_kinematic_trajectory_predictor.GetVerticalPathAltitudes()));
         m_target_reference_ias = Units::MetersPerSecondSpeed(CoreUtils::LinearlyInterpolate(
               m_target_reference_lookup_index, Units::MetersLength(m_target_kinematic_dtg_to_last_waypoint).value(),
               m_target_kinematic_trajectory_predictor.GetVerticalPathDistances(),
               m_target_kinematic_trajectory_predictor.GetVerticalPathVelocities()));
         m_target_reference_gs = Units::MetersPerSecondSpeed(CoreUtils::LinearlyInterpolate(
               m_target_reference_lookup_index, Units::MetersLength(m_target_kinematic_dtg_to_last_waypoint).value(),
               m_target_kinematic_trajectory_predictor.GetVerticalPathDistances(),
               m_target_kinematic_trajectory_predictor.GetVerticalPathGroundspeeds()));
      }

      m_target_ttg_to_trp =
            m_target_ttg_to_end_of_route - m_target_kinematic_traffic_reference_point_calcs.GetTimeToGoToWaypoint();
      reference_ttg = m_target_ttg_to_trp + m_assigned_spacing_goal;

   } else if ((target_adsb_history.size() >= 2) || IsTargetPassedTrp()) {
      is_crossing_time_valid = IMUtils::GetCrossingTime(
            m_target_kinematic_traffic_reference_point_calcs.GetDistanceFromWaypoint(), target_adsb_history,
            m_target_kinematic_trajectory_predictor.GetHorizontalPath(), m_target_trp_crossing_time);
      if (!is_crossing_time_valid) {
         LOG4CPLUS_WARN(IMTimeBasedAchieve::m_logger,
                        "IMTimeBasedAchieve::update ac "
                              << current_ownship_state.GetId() << "  time "
                              << current_ownship_state.GetTimeStamp().value() << endl
                              << "Non calculation of targettrpcrossingtime in GetCrossingTime;" << endl
                              << "referencettg calculation suspicious" << endl);
      }
      reference_ttg = m_target_trp_crossing_time + m_assigned_spacing_goal -
                      Units::SecondsTime(current_ownship_state.GetTimeStamp().value());

      m_target_ttg_to_trp = Units::zero();
   }

   if (m_previous_im_speed_command_ias == Units::zero()) {
      m_previous_reference_im_speed_command_tas = m_weather_prediction.getAtmosphere()->CAS2TAS(
            Units::MetersPerSecondSpeed(m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities().back()),
            Units::MetersLength(m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes().back()));
      m_previous_im_speed_command_ias =
            Units::MetersPerSecondSpeed(m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities().back());

      m_previous_reference_im_speed_command_mach =
            Units::MetersPerSecondSpeed(m_previous_reference_im_speed_command_tas).value() /
            sqrt(GAMMA * R.value() *
                 m_weather_prediction.getAtmosphere()
                       ->GetTemperature(Units::MetersLength(
                             m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes().back()))
                       .value());
   }

   if (Units::abs(m_ownship_kinematic_dtg_to_ptp) <=
       Units::abs(Units::MetersLength(m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances().back()))) {
      m_ownship_reference_lookup_index =
            CoreUtils::FindNearestIndex(Units::MetersLength(m_ownship_kinematic_dtg_to_ptp).value(),
                                        m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances());

      if (m_ownship_reference_lookup_index == 0) {
         m_ownship_reference_ttg_to_ptp =
               Units::SecondsTime(m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimeByIndex(0));
         m_ownship_ttg_to_abp =
               m_ownship_reference_ttg_to_ptp - m_ownship_kinematic_achieve_by_calcs.GetTimeToGoToWaypoint();
      } else {
         m_ownship_reference_ttg_to_ptp = Units::SecondsTime(CoreUtils::LinearlyInterpolate(
               m_ownship_reference_lookup_index, Units::MetersLength(m_ownship_kinematic_dtg_to_ptp).value(),
               m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances(),
               m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes()));
         m_ownship_ttg_to_abp =
               m_ownship_reference_ttg_to_ptp - m_ownship_kinematic_achieve_by_calcs.GetTimeToGoToWaypoint();
      }
   }

   if (is_crossing_time_valid) {
      Units::Time tmp1 = Units::SecondsTime(current_ownship_state.GetTimeStamp().value()) + m_ownship_ttg_to_abp;
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

   m_reference_precalc_index = CoreUtils::FindNearestIndex(
         Units::SecondsTime(ownrefttgtoend).value(), m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes());

   if (m_reference_precalc_index >=
       static_cast<int>(m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes().size() - 1)) {
      if (Units::SecondsTime(ownrefttgtoend).value() >=
          m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes().back()) {
         m_reference_precalc_index =
               static_cast<int>(m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes().size() - 1);
         reference_distance =
               Units::MetersLength(-m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances().back());
         m_ownship_reference_cas =
               Units::MetersPerSecondSpeed(m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities().back());
         m_ownship_reference_gs = Units::MetersPerSecondSpeed(
               m_ownship_kinematic_trajectory_predictor.GetVerticalPathGroundspeeds().back());
         m_ownship_reference_altitude =
               Units::MetersLength(m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes().back());
      } else {
         reference_distance = Units::MetersLength(
               -CoreUtils::LinearlyInterpolate(m_reference_precalc_index, Units::SecondsTime(ownrefttgtoend).value(),
                                               m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes(),
                                               m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances()));

         m_ownship_reference_cas = Units::MetersPerSecondSpeed(
               CoreUtils::LinearlyInterpolate(m_reference_precalc_index, Units::SecondsTime(ownrefttgtoend).value(),
                                              m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes(),
                                              m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities()));
         m_ownship_reference_gs = Units::MetersPerSecondSpeed(
               CoreUtils::LinearlyInterpolate(m_reference_precalc_index, Units::SecondsTime(ownrefttgtoend).value(),
                                              m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes(),
                                              m_ownship_kinematic_trajectory_predictor.GetVerticalPathGroundspeeds()));
         m_ownship_reference_altitude = Units::MetersLength(
               CoreUtils::LinearlyInterpolate(m_reference_precalc_index, Units::SecondsTime(ownrefttgtoend).value(),
                                              m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes(),
                                              m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes()));
      }
   } else if (m_reference_precalc_index == 0) {
      reference_distance =
            Units::MetersLength(-m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistanceByIndex(0));
      m_ownship_reference_cas =
            Units::MetersPerSecondSpeed(m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(0));
      m_ownship_reference_altitude =
            Units::MetersLength(m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudeByIndex(0));

   } else {
      reference_distance = Units::MetersLength(
            -CoreUtils::LinearlyInterpolate(m_reference_precalc_index, Units::SecondsTime(ownrefttgtoend).value(),
                                            m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes(),
                                            m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances()));

      m_ownship_reference_cas = Units::MetersPerSecondSpeed(
            CoreUtils::LinearlyInterpolate(m_reference_precalc_index, Units::SecondsTime(ownrefttgtoend).value(),
                                           m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes(),
                                           m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities()));
      m_ownship_reference_gs = Units::MetersPerSecondSpeed(
            CoreUtils::LinearlyInterpolate(m_reference_precalc_index, Units::SecondsTime(ownrefttgtoend).value(),
                                           m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes(),
                                           m_ownship_kinematic_trajectory_predictor.GetVerticalPathGroundspeeds()));
      m_ownship_reference_altitude = Units::MetersLength(
            CoreUtils::LinearlyInterpolate(m_reference_precalc_index, Units::SecondsTime(ownrefttgtoend).value(),
                                           m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes(),
                                           m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes()));
   }

   const Units::Length temp = reference_distance + m_ownship_kinematic_dtg_to_ptp;

   m_im_speed_command_ias = m_ownship_reference_cas + temp * m_achieve_control_gain;

   if (m_im_speed_command_ias < Units::zero()) {
      m_im_speed_command_ias = Units::zero();
   }

   m_unmodified_im_speed_command_ias = m_im_speed_command_ias;

   if (guidance_out.GetSelectedSpeed().GetSpeedType() == INDICATED_AIR_SPEED) {
      CalculateIas(Units::FeetLength(current_ownship_state.m_z), three_dof_dynamics_state);
   } else {
      CalculateMach(reference_ttg, Units::FeetLength(current_ownship_state.m_z), three_dof_dynamics_state.current_mass);
   }

   m_previous_im_speed_command_ias = m_im_speed_command_ias;
   Units::Speed tascommand = m_weather_prediction.getAtmosphere()->CAS2TAS(
         m_im_speed_command_ias, Units::FeetLength(current_ownship_state.m_z));

   RecordInternalObserverMetrics(current_ownship_state, current_target_state, three_dof_dynamics_state,
                                 m_unmodified_im_speed_command_ias, tascommand, m_ownship_reference_cas,
                                 reference_distance, guidance_out);

   if (m_pilot_delay.IsPilotDelayOn()) {
      guidance_out.m_ias_command = m_im_speed_command_with_pilot_delay;
      if (guidance_out.GetSelectedSpeed().GetSpeedType() == MACH_SPEED) {
         const auto true_airspeed_equivalent =
               m_weather_prediction.CAS2TAS(m_im_speed_command_with_pilot_delay, current_ownship_state.GetPositionZ());
         const auto mach_equivalent =
               m_weather_prediction.TAS2Mach(true_airspeed_equivalent, current_ownship_state.GetPositionZ());
         guidance_out.SetMachCommand(mach_equivalent);
      }
   } else {
      guidance_out.m_ias_command = m_im_speed_command_ias;
      if (guidance_out.GetSelectedSpeed().GetSpeedType() == MACH_SPEED) {
         const auto true_airspeed_equivalent =
               m_weather_prediction.CAS2TAS(m_im_speed_command_ias, current_ownship_state.GetPositionZ());
         const auto mach_equivalent =
               m_weather_prediction.TAS2Mach(true_airspeed_equivalent, current_ownship_state.GetPositionZ());
         guidance_out.SetMachCommand(mach_equivalent);
      }
   }

   DoAlgorithmLogging(current_ownship_state, current_target_state, reference_distance, tascommand, guidance_out,
                      is_crossing_time_valid);

   return guidance_out;
}

void IMTimeBasedAchieve::TestForTrafficAlignment(
      const interval_management::open_source::AircraftState &current_ownship_state,
      const std::vector<interval_management::open_source::AircraftState> &target_adsb_history) {
   Units::SignedRadiansAngle target_track_angle = CalculateTargetTrackAngle(target_adsb_history);
   Units::UnsignedRadiansAngle pt_to_pt_course;
   Units::Length tgt_distance;
   Units::UnsignedAngle own_planned_course;
   m_im_ownship_distance_calculator.CalculateAlongPathDistanceFromPosition(
         Units::FeetLength(target_adsb_history.back().m_x), Units::FeetLength(target_adsb_history.back().m_y),
         tgt_distance, own_planned_course, pt_to_pt_course);

   Units::UnsignedRadiansAngle track_difference = abs(own_planned_course - target_track_angle);
   if (track_difference < Units::DegreesAngle(15.0)) {
      SaveTargetStateAtTrafficAlignment(Units::SecondsTime(current_ownship_state.GetTimeStamp().value()),
                                        target_adsb_history.back(), tgt_distance);
      // LOG4CPLUS_DEBUG(m_logger,"Alignment captured at adsb history time: " <<
      // target_adsb_history.back().GetTimeStamp().value()
      //<< " target track: " << Units::UnsignedDegreesAngle(target_track_angle) << " own planned course: " <<
      // Units::UnsignedDegreesAngle(own_planned_course));
   } else {
      track_difference = abs(pt_to_pt_course - target_track_angle);
      if (track_difference < Units::DegreesAngle(15.0)) {
         SaveTargetStateAtTrafficAlignment(Units::SecondsTime(current_ownship_state.GetTimeStamp().value()),
                                           target_adsb_history.back(), tgt_distance);
         // LOG4CPLUS_DEBUG(m_logger,"Alignment captured at adsb history time: " <<
         // target_adsb_history.back().GetTimeStamp().value()
         //<< " target track: " << Units::UnsignedDegreesAngle(target_track_angle) << " pt-pt-course: " <<
         // Units::UnsignedDegreesAngle(pt_to_pt_course));
      }
   }
}

aaesim::open_source::Guidance IMTimeBasedAchieve::HandleMaintainStage(
      const interval_management::open_source::AircraftState &current_ownship_state,
      const interval_management::open_source::AircraftState &current_target_state,
      const vector<interval_management::open_source::AircraftState> &target_adsb_history,
      const aaesim::open_source::DynamicsState &three_dof_dynamics_state,
      const aaesim::open_source::Guidance &previous_im_guidance, aaesim::open_source::Guidance &guidance_out) {
   m_ownship_ttg_to_abp = Units::zero();
   m_ownship_kinematic_dtg_to_abp = Units::zero();
   m_predicted_spacing_interval = Units::NegInfinity();

   Units::Time target_time = Units::SecondsTime(current_ownship_state.GetTimeStamp().value()) - m_assigned_spacing_goal;

   bool built;
   m_target_state_projected_asg_adjusted = IMUtils::GetProjectedTargetState(
         m_im_ownship_distance_calculator, target_adsb_history,
         m_ownship_kinematic_trajectory_predictor.GetHorizontalPath(), target_time,
         Units::RadiansAngle(current_ownship_state.GetHeadingCcwFromEastRadians() + Units::PI_RADIANS_ANGLE), built);

   if (built) {
      if (!m_transitioned_to_maintain) {
         m_im_kinematic_time_based_maintain->Prepare(
               m_previous_reference_im_speed_command_tas, m_previous_im_speed_command_ias, m_speed_limiter,
               m_previous_reference_im_speed_command_mach, m_ownship_kinematic_trajectory_predictor,
               m_im_ownship_distance_calculator, target_adsb_history, m_im_clearance,
               m_speed_limiter.GetRfLegSpeedLimits());
         m_ownship_kinematic_achieve_by_calcs.ComputeCrossingTime(current_ownship_state);

         m_transitioned_to_maintain = true;
      }

      guidance_out = m_im_kinematic_time_based_maintain->Update(
            three_dof_dynamics_state, current_ownship_state, m_target_state_projected_asg_adjusted,
            m_ownship_kinematic_trajectory_predictor, previous_im_guidance, target_adsb_history,
            m_ownship_kinematic_achieve_by_calcs, m_target_kinematic_traffic_reference_point_calcs, m_pilot_delay,
            m_target_kinematic_dtg_to_last_waypoint);

      SetActiveFilter(m_im_kinematic_time_based_maintain->GetActiveFilter());

      m_measured_spacing_interval = CalculateMeasuredSpacingInterval(current_ownship_state, current_target_state);

      m_im_speed_command_ias = m_im_kinematic_time_based_maintain->GetImSpeedCommandIas();
      m_im_speed_command_with_pilot_delay = m_im_kinematic_time_based_maintain->GetDelayedImSpeedCommandIas();
      m_unmodified_im_speed_command_ias = m_im_kinematic_time_based_maintain->GetUnmodifiedImSpeedCommandIas();
      m_previous_im_speed_command_ias = m_im_kinematic_time_based_maintain->GetPreviousSpeedCommandIas();
      return guidance_out;
   }

   m_ownship_reference_lookup_index =
         CoreUtils::FindNearestIndex(Units::MetersLength(m_ownship_kinematic_dtg_to_ptp).value(),
                                     m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances());

   if (m_ownship_reference_lookup_index > 0) {
      const Units::Speed nominal_ias = Units::MetersPerSecondSpeed(
            m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(m_ownship_reference_lookup_index));
      m_previous_im_speed_command_ias = m_im_kinematic_time_based_maintain->GetPreviousSpeedCommandIas();
      if (m_previous_im_speed_command_ias != Units::zero() && m_previous_im_speed_command_ias < nominal_ias) {
         guidance_out.m_ias_command = m_previous_im_speed_command_ias;
      } else {
         guidance_out.m_ias_command = nominal_ias;
         m_previous_im_speed_command_ias = nominal_ias;
      }
      m_im_speed_command_ias = guidance_out.m_ias_command;
      m_im_speed_command_with_pilot_delay = guidance_out.m_ias_command;
   } else {
      guidance_out.SetValid(false);
   }

   return guidance_out;
}

const Units::Time IMTimeBasedAchieve::CalculateMeasuredSpacingInterval(
      const interval_management::open_source::AircraftState &current_ownship_state,
      const interval_management::open_source::AircraftState &current_target_state) {

   if (m_target_kinematic_traffic_reference_point_calcs.IsWaypointSet() &&
       GetTargetKinematicDtgToTrp() > Units::zero()) {

      // special calculation if target has not passed TRP
      Units::Length target_dtg_trp =
            m_target_kinematic_traffic_reference_point_calcs.ComputeDistanceToWaypoint(current_target_state);
      Units::SecondsTime target_ttg_trp = target_dtg_trp / current_target_state.GetGroundSpeed();

      Units::SecondsTime time_since_abp = Units::SecondsTime(current_ownship_state.GetTimeStamp().value()) -
                                          m_ownship_kinematic_achieve_by_calcs.GetCrossingTime();
      Units::SecondsTime result = -(target_ttg_trp + time_since_abp);
      LOG4CPLUS_DEBUG(m_logger, "Special MSI calculation:  target TTG to TRP = "
                                      << target_ttg_trp << ", ownship time since ABP = " << time_since_abp
                                      << ", MSI = " << result);
      return result;
   }

   if (m_im_clearance.GetClearanceType() == IMClearance::CAPTURE) {
      if (!HasOwnshipReachedTargetAlongPathPositionAtAlignment()) {
         m_time_since_traffic_alignment =
               Units::SecondsTime(current_ownship_state.GetTimeStamp().value()) - m_time_at_traffic_alignment;

         return m_time_since_traffic_alignment + ((m_ownship_kinematic_dtg_to_ptp - m_target_dtg_at_traffic_alignment) /
                                                  m_target_state_at_traffic_alignment.GetGroundSpeed());
      }
   }

   return Units::SecondsTime(m_im_kinematic_time_based_maintain->GetMsi());
}

void IMTimeBasedAchieve::SaveTargetStateAtTrafficAlignment(
      Units::Time ownship_current_time,
      const interval_management::open_source::AircraftState &target_state_at_traffic_alignment,
      const Units::Length target_dtg_at_alignment) {
   m_is_target_aligned = true;
   m_time_at_traffic_alignment = ownship_current_time;
   m_target_dtg_at_traffic_alignment = target_dtg_at_alignment;
   m_target_state_at_traffic_alignment = target_state_at_traffic_alignment;
}

void IMTimeBasedAchieve::CalculateIas(const Units::Length current_ownship_altitude,
                                      const aaesim::open_source::DynamicsState &three_dof_dynamics_state) {
   m_im_speed_command_ias = m_speed_limiter.LimitSpeedCommand(
         m_previous_im_speed_command_ias, m_im_speed_command_ias,
         m_ownship_kinematic_trajectory_predictor.GetVerticalPathCasByIndex(m_ownship_reference_lookup_index),
         m_ownship_kinematic_dtg_to_abp, m_ownship_kinematic_dtg_to_ptp, current_ownship_altitude,
         three_dof_dynamics_state.flap_configuration);

   if (m_im_speed_command_ias != m_previous_im_speed_command_ias) {
      m_total_number_of_im_speed_changes++;
   }

   if (m_pilot_delay.IsPilotDelayOn()) {
      m_im_speed_command_with_pilot_delay =
            m_pilot_delay.UpdateIAS(m_previous_im_speed_command_ias, m_im_speed_command_ias, current_ownship_altitude,
                                    m_ownship_kinematic_trajectory_predictor.GetAltitudeAtFinalWaypoint());
   }
}

void IMTimeBasedAchieve::CalculateMach(const Units::Time reference_ttg, const Units::Length current_ownship_altitude,
                                       const Units::Mass current_mass) {
   Units::Speed temptrue =
         m_weather_prediction.getAtmosphere()->CAS2TAS(m_im_speed_command_ias, current_ownship_altitude);

   double unbounded_estimated_mach =
         Units::MetersPerSecondSpeed(temptrue).value() /
         sqrt(GAMMA * R.value() *
              m_weather_prediction.getAtmosphere()->GetTemperature(current_ownship_altitude).value());

   BoundedValue<double, 0, 2> estimated_mach(0);
   if (unbounded_estimated_mach > 2)
      estimated_mach = 2;  // AAES-1308
   else
      estimated_mach = unbounded_estimated_mach;

   Units::Speed nominal_ias = Units::MetersPerSecondSpeed(
         m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(m_ownship_reference_lookup_index));
   Units::Speed nominal_tas = m_weather_prediction.getAtmosphere()->CAS2TAS(nominal_ias, current_ownship_altitude);
   BoundedValue<double, 0, 2> nominal_mach(
         Units::MetersPerSecondSpeed(nominal_tas).value() /
         sqrt(GAMMA * R.value() *
              m_weather_prediction.getAtmosphere()->GetTemperature(current_ownship_altitude).value()));

   estimated_mach = m_speed_limiter.LimitMachCommand(
         BoundedValue<double, 0, 2>(m_previous_reference_im_speed_command_mach), estimated_mach, nominal_mach,
         current_mass, current_ownship_altitude, m_weather_prediction);

   if (estimated_mach != BoundedValue<double, 0, 2>(m_previous_reference_im_speed_command_mach)) {
      m_total_number_of_im_speed_changes++;
   }

   if (m_pilot_delay.IsPilotDelayOn()) {
      m_im_speed_command_with_pilot_delay = m_pilot_delay.UpdateMach(
            m_previous_reference_im_speed_command_mach, estimated_mach, current_ownship_altitude,
            m_ownship_kinematic_trajectory_predictor.GetAltitudeAtFinalWaypoint());
   }

   m_previous_reference_im_speed_command_mach = estimated_mach;
   m_im_speed_command_ias = m_weather_prediction.getAtmosphere()->MachToIAS(estimated_mach, current_ownship_altitude);
}

void IMTimeBasedAchieve::RecordInternalObserverMetrics(
      const interval_management::open_source::AircraftState &current_ownship_state,
      const interval_management::open_source::AircraftState &current_target_state,
      const aaesim::open_source::DynamicsState &dynamics_state, const Units::Speed unmodified_ias,
      const Units::Speed tas_command, const Units::Speed reference_velocity, const Units::Length reference_distance,
      const aaesim::open_source::Guidance &guidance) {
   if (InternalObserver::getInstance()->outputNM()) {
      NMObserver &nm_observer = InternalObserver::getInstance()->GetNMObserver(current_ownship_state.GetId());

      if (nm_observer.curr_NM == -2 && guidance.IsValid()) {
         nm_observer.curr_NM =
               static_cast<int>(Units::NauticalMilesLength(Units::abs(m_ownship_kinematic_dtg_to_ptp)).value());
      }

      if (Units::abs(m_ownship_kinematic_dtg_to_ptp) <= Units::NauticalMilesLength(nm_observer.curr_NM) &&
          guidance.IsValid()) {
         nm_observer.curr_NM--;

         double lval = m_speed_limiter.LowLimit(m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(
               m_ownship_reference_lookup_index));
         double hval =
               m_speed_limiter.HighLimit(m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(
                     m_ownship_reference_lookup_index));

         double ltas = Units::MetersPerSecondSpeed(
                             m_weather_prediction.getAtmosphere()->CAS2TAS(
                                   Units::MetersPerSecondSpeed(lval), Units::FeetLength(current_ownship_state.m_z)))
                             .value();
         double htas = Units::MetersPerSecondSpeed(
                             m_weather_prediction.getAtmosphere()->CAS2TAS(
                                   Units::MetersPerSecondSpeed(hval), Units::FeetLength(current_ownship_state.m_z)))
                             .value();

         nm_observer.output_NM_values(
               Units::MetersLength(m_ownship_kinematic_dtg_to_ptp).value(),
               Units::MetersLength(Units::negInfinity()).value(), current_ownship_state.GetTimeStamp().value(),
               Units::MetersPerSecondSpeed(m_im_speed_command_ias).value(),
               Units::MetersPerSecondSpeed(current_ownship_state.GetGroundSpeed()).value(),
               Units::MetersPerSecondSpeed(current_target_state.GetGroundSpeed()).value(), lval, hval, ltas, htas);
      }
   }

   InternalObserver::getInstance()->addAchieveRcd(
         static_cast<size_t>(current_ownship_state.GetId()), current_ownship_state.GetTimeStamp().value(),
         Units::SecondsTime(m_target_ttg_to_trp).value(), Units::SecondsTime(m_ownship_ttg_to_abp).value(),
         Units::MetersLength(-m_ownship_kinematic_dtg_to_ptp).value(), Units::MetersLength(reference_distance).value());
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
