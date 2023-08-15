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

#include "imalgs/IMDistBasedAchieve.h"

#include "public/SimulationTime.h"
#include "public/CustomMath.h"
#include "public/CoreUtils.h"
#include "public/AircraftCalculations.h"

using namespace interval_management::open_source;

log4cplus::Logger IMDistBasedAchieve::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("IMDistBasedAchieve"));
const Units::Length IMDistBasedAchieve::DEFAULT_DISTANCE_BASED_ASSIGNED_SPACING_GOAL = Units::NegInfinity();

IMDistBasedAchieve::IMDistBasedAchieve()
   : m_true_distances(nullptr), m_assigned_spacing_goal(DEFAULT_DISTANCE_BASED_ASSIGNED_SPACING_GOAL) {

   if (m_im_kinematic_dist_based_maintain == nullptr) {
      m_im_kinematic_dist_based_maintain = std::make_shared<IMKinematicDistBasedMaintain>();
   }

   m_true_distances = std::make_shared<TrueDistances>();
}

IMDistBasedAchieve::IMDistBasedAchieve(const IMDistBasedAchieve &obj) { Copy(obj); }

void IMDistBasedAchieve::Copy(const IMDistBasedAchieve &obj) {
   IMKinematicAchieve::Copy(obj);
   m_im_kinematic_dist_based_maintain = obj.m_im_kinematic_dist_based_maintain;
   m_true_distances = obj.m_true_distances;
   m_assigned_spacing_goal = obj.m_assigned_spacing_goal;
}

IMDistBasedAchieve::~IMDistBasedAchieve() = default;

void IMDistBasedAchieve::Initialize(const OwnshipPredictionParameters &ownship_prediction_parameters,
                                    const AircraftIntent &ownship_aircraft_intent,
                                    aaesim::open_source::WeatherPrediction &weather_prediction) {
   IMKinematicAchieve::Initialize(ownship_prediction_parameters, ownship_aircraft_intent, weather_prediction);

   m_im_kinematic_dist_based_maintain->SetQuantizeFlag(GetQuantizeFlag());

   if (m_assigned_spacing_goal == DEFAULT_DISTANCE_BASED_ASSIGNED_SPACING_GOAL) {
      throw std::runtime_error("mAssignedSpacingGoal must be explicitly set");
   }
}

void IMDistBasedAchieve::IterationReset() {
   IMKinematicAchieve::IterationReset();

   m_predicted_spacing_interval = Units::NegInfinity();
   m_measured_spacing_interval = Units::NegInfinity();
   m_target_state_projected_on_ownships_path_at_adjusted_distance = interval_management::open_source::AircraftState();

   m_im_kinematic_dist_based_maintain->IterationReset();
   m_true_distances->Clear();
}

bool IMDistBasedAchieve::load(DecodedStream *input) {
   bool loaded = IMKinematicAchieve::load(input);

   m_assigned_spacing_goal = Units::NauticalMilesLength(m_assigned_spacing_goal_from_input_file);

   if (m_im_kinematic_dist_based_maintain == nullptr) {
      m_im_kinematic_dist_based_maintain = std::make_shared<IMKinematicDistBasedMaintain>();
   }
   m_im_kinematic_dist_based_maintain->InitializeScenario(this, m_maintain_control_gain);

   return loaded;
}

aaesim::open_source::Guidance IMDistBasedAchieve::Update(
      const aaesim::open_source::Guidance &previous_im_guidance,
      const aaesim::open_source::DynamicsState &three_dof_dynamics_state,
      const interval_management::open_source::AircraftState &current_ownship_state,
      const interval_management::open_source::AircraftState &current_target_state,
      const vector<interval_management::open_source::AircraftState> &target_adsb_history) {
   aaesim::open_source::Guidance im_guidance =
         IMKinematicAchieve::Update(previous_im_guidance, three_dof_dynamics_state, current_ownship_state,
                                    current_target_state, target_adsb_history);

   if (!IsImOperationComplete() && im_guidance.IsValid()) {
      Units::Time reference_ttg = Units::zero();
      Units::Time ownship_ttg_from_abp_to_ptp = Units::zero();
      Units::Length ownship_reference_dtg_to_ptp = Units::zero();
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
         m_stage_of_im_operation = ACHIEVE;

         InternalObserver::getInstance()->updateFinalGS(
               current_target_state.GetId(),
               Units::MetersPerSecondSpeed(target_adsb_history.back().GetGroundSpeed()).value());
         InternalObserver::getInstance()->updateFinalGS(
               current_ownship_state.GetId(),
               Units::MetersPerSecondSpeed(current_ownship_state.GetGroundSpeed()).value());

         Units::Time target_reference_ttg_to_ptp = Units::zero();
         Units::Time target_reference_ttg_to_trp = Units::zero();

         m_target_reference_lookup_index =
               CoreUtils::FindNearestIndex(Units::MetersLength(m_target_kinematic_dtg_to_last_waypoint).value(),
                                           m_target_kinematic_trajectory_predictor.GetVerticalPathDistances());

         if (m_target_reference_lookup_index >=
             m_target_kinematic_trajectory_predictor.GetVerticalPathDistances().size() - 1) {
            target_reference_ttg_to_ptp =
                  Units::SecondsTime(m_target_kinematic_trajectory_predictor.GetVerticalPathTimes().back());
            m_target_reference_lookup_index =
                  static_cast<int>(m_target_kinematic_trajectory_predictor.GetVerticalPathTimes().size() - 1);
            m_target_reference_altitude =
                  Units::MetersLength(m_target_kinematic_trajectory_predictor.GetVerticalPathAltitudes().back());
            m_target_reference_gs = Units::MetersPerSecondSpeed(
                  m_target_kinematic_trajectory_predictor.GetVerticalPathGroundspeeds().back());
            m_target_reference_ias = Units::MetersPerSecondSpeed(
                  m_target_kinematic_trajectory_predictor.GetVerticalPathVelocities().back());
         } else if (m_target_reference_lookup_index == 0) {
            target_reference_ttg_to_ptp =
                  Units::SecondsTime(m_target_kinematic_trajectory_predictor.GetVerticalPathTimeByIndex(0));
            m_target_reference_altitude =
                  Units::MetersLength(m_target_kinematic_trajectory_predictor.GetVerticalPathAltitudeByIndex(0));
            m_target_reference_gs = Units::ZERO_SPEED;
            m_target_reference_ias = Units::ZERO_SPEED;
         } else {
            target_reference_ttg_to_ptp = Units::SecondsTime(CoreUtils::LinearlyInterpolate(
                  m_target_reference_lookup_index, Units::MetersLength(m_target_kinematic_dtg_to_last_waypoint).value(),
                  m_target_kinematic_trajectory_predictor.GetVerticalPathDistances(),
                  m_target_kinematic_trajectory_predictor.GetVerticalPathTimes()));
            m_target_reference_altitude = Units::MetersLength(CoreUtils::LinearlyInterpolate(
                  m_target_reference_lookup_index, Units::MetersLength(m_target_kinematic_dtg_to_last_waypoint).value(),
                  m_target_kinematic_trajectory_predictor.GetVerticalPathDistances(),
                  m_target_kinematic_trajectory_predictor.GetVerticalPathAltitudes()));
            m_target_reference_gs = Units::MetersPerSecondSpeed(CoreUtils::LinearlyInterpolate(
                  m_target_reference_lookup_index, Units::MetersLength(m_target_kinematic_dtg_to_last_waypoint).value(),
                  m_target_kinematic_trajectory_predictor.GetVerticalPathDistances(),
                  m_target_kinematic_trajectory_predictor.GetVerticalPathGroundspeeds()));
            m_target_reference_ias = Units::MetersPerSecondSpeed(CoreUtils::LinearlyInterpolate(
                  m_target_reference_lookup_index, Units::MetersLength(m_target_kinematic_dtg_to_last_waypoint).value(),
                  m_target_kinematic_trajectory_predictor.GetVerticalPathDistances(),
                  m_target_kinematic_trajectory_predictor.GetVerticalPathVelocities()));
         }

         target_reference_ttg_to_trp =
               target_reference_ttg_to_ptp - m_target_kinematic_traffic_reference_point_calcs.GetTimeToGoToWaypoint();

         reference_ttg = target_reference_ttg_to_trp;

         if (m_previous_im_speed_command_ias == Units::zero()) {
            m_previous_reference_im_speed_command_tas = m_weather_prediction.getAtmosphere()->CAS2TAS(
                  Units::MetersPerSecondSpeed(
                        m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities().back()),
                  Units::MetersLength(m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes().back()));

            m_previous_im_speed_command_ias = Units::MetersPerSecondSpeed(
                  m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities().back());
            m_previous_reference_im_speed_command_mach =
                  Units::MetersPerSecondSpeed(m_previous_reference_im_speed_command_tas).value() /
                  sqrt(GAMMA * R.value() *
                       m_weather_prediction.getAtmosphere()
                             ->GetTemperature(Units::MetersLength(
                                   m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes().back()))
                             .value());
         }

         // Get Own TTG and pre-planned airspeed index from ABP* (ABP-Spacing) to end of trajectory
         Units::MetersLength ownship_reference_distance =
               m_ownship_kinematic_achieve_by_calcs.GetDistanceFromWaypoint() + m_assigned_spacing_goal;

         m_ownship_reference_lookup_index = CoreUtils::FindNearestIndex(
               ownship_reference_distance.value(), m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances());

         // Note that this calculation includes the ASG. Be careful how used.
         ownship_ttg_from_abp_to_ptp = Units::SecondsTime(
               CoreUtils::LinearlyInterpolate(m_ownship_reference_lookup_index, ownship_reference_distance.value(),
                                              m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances(),
                                              m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes()));

         if (m_ownship_kinematic_dtg_to_ptp <=
             Units::abs(
                   Units::MetersLength(m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances().back()))) {
            m_ownship_reference_lookup_index =
                  CoreUtils::FindNearestIndex(Units::MetersLength(m_ownship_kinematic_dtg_to_ptp).value(),
                                              m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances());

            if (m_ownship_reference_lookup_index >=
                m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances().size() - 1) {
               m_ownship_reference_ttg_to_ptp =
                     Units::SecondsTime(m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes().back());
               m_ownship_ttg_to_abp = m_ownship_reference_ttg_to_ptp - ownship_ttg_from_abp_to_ptp;
               m_ownship_reference_lookup_index =
                     static_cast<int>(m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes().size() - 1);
            } else if (m_ownship_reference_lookup_index == 0) {
               m_ownship_reference_ttg_to_ptp =
                     Units::SecondsTime(m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimeByIndex(0));
               m_ownship_ttg_to_abp = m_ownship_reference_ttg_to_ptp - ownship_ttg_from_abp_to_ptp;
            } else {
               m_ownship_reference_ttg_to_ptp = Units::SecondsTime(CoreUtils::LinearlyInterpolate(
                     m_ownship_reference_lookup_index, Units::MetersLength(m_ownship_kinematic_dtg_to_ptp).value(),
                     m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances(),
                     m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes()));
               m_ownship_ttg_to_abp = m_ownship_reference_ttg_to_ptp - ownship_ttg_from_abp_to_ptp;
            }
         }

         m_predicted_spacing_interval = CalculatePredictedSpacingInterval(target_reference_ttg_to_trp);
         m_measured_spacing_interval = Units::NegInfinity();

         if (m_threshold_flag) {
            Units::Time error_threshold = GetErrorThreshold(
                  m_ownship_kinematic_dtg_to_ptp -
                  (m_ownship_kinematic_achieve_by_calcs.GetDistanceFromWaypoint() + m_assigned_spacing_goal));

            if (Units::abs(m_ownship_ttg_to_abp - reference_ttg) > error_threshold) {
               reference_ttg = reference_ttg + error_threshold * (m_ownship_ttg_to_abp - reference_ttg) /
                                                     Units::abs(m_ownship_ttg_to_abp - reference_ttg);
            } else {
               reference_ttg = m_ownship_ttg_to_abp;
            }
         }

         // TODO: a variable with this name already exists, and points to a different value. See
         // m_ownship_reference_ttg_to_ptp
         Units::Time ownrefttgtoend = reference_ttg + ownship_ttg_from_abp_to_ptp;

         m_reference_precalc_index =
               CoreUtils::FindNearestIndex(Units::SecondsTime(ownrefttgtoend).value(),
                                           m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes());

         if (m_reference_precalc_index >= m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes().size() - 1) {

            m_reference_precalc_index =
                  static_cast<int>(m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes().size() - 1);

            ownship_reference_dtg_to_ptp =
                  Units::MetersLength(-m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances().back());
            m_ownship_reference_cas = Units::MetersPerSecondSpeed(
                  m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities().back());
            m_ownship_reference_gs = Units::MetersPerSecondSpeed(
                  m_ownship_kinematic_trajectory_predictor.GetVerticalPathGroundspeeds().back());
            m_ownship_reference_altitude =
                  Units::MetersLength(m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes().back());
         } else if (m_reference_precalc_index == 0) {
            ownship_reference_dtg_to_ptp =
                  Units::MetersLength(-m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistanceByIndex(0));
            m_ownship_reference_cas = Units::MetersPerSecondSpeed(
                  m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(0));
            m_ownship_reference_gs = Units::ZERO_SPEED;
            m_ownship_reference_altitude =
                  Units::MetersLength(m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudeByIndex(0));
         } else {

            ownship_reference_dtg_to_ptp = Units::MetersLength(
                  -CoreUtils::LinearlyInterpolate(m_reference_precalc_index, Units::SecondsTime(ownrefttgtoend).value(),
                                                  m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes(),
                                                  m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances()));

            m_ownship_reference_cas = Units::MetersPerSecondSpeed(
                  CoreUtils::LinearlyInterpolate(m_reference_precalc_index, Units::SecondsTime(ownrefttgtoend).value(),
                                                 m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes(),
                                                 m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities()));
            m_ownship_reference_gs = Units::MetersPerSecondSpeed(CoreUtils::LinearlyInterpolate(
                  m_reference_precalc_index, Units::SecondsTime(ownrefttgtoend).value(),
                  m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes(),
                  m_ownship_kinematic_trajectory_predictor.GetVerticalPathGroundspeeds()));
            m_ownship_reference_altitude = Units::MetersLength(
                  CoreUtils::LinearlyInterpolate(m_reference_precalc_index, Units::SecondsTime(ownrefttgtoend).value(),
                                                 m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes(),
                                                 m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes()));
         }

         m_im_speed_command_ias =
               m_ownship_reference_cas +
               (ownship_reference_dtg_to_ptp + m_ownship_kinematic_dtg_to_ptp) * m_achieve_control_gain;

         m_unmodified_im_speed_command_ias = m_im_speed_command_ias;

         if (im_guidance.GetSelectedSpeed().GetSpeedType() == INDICATED_AIR_SPEED) {
            CalculateIas(current_ownship_state.GetPositionZ(), three_dof_dynamics_state);
         } else {
            CalculateMach(reference_ttg, current_ownship_state.GetPositionZ(), three_dof_dynamics_state.current_mass);
         }

         m_previous_im_speed_command_ias = m_im_speed_command_ias;

         Units::Speed im_speed_command_tas = m_weather_prediction.getAtmosphere()->CAS2TAS(
               m_im_speed_command_ias, current_ownship_state.GetPositionZ());

         RecordData(im_guidance, current_ownship_state, current_target_state, three_dof_dynamics_state,
                    m_unmodified_im_speed_command_ias, im_speed_command_tas, m_ownship_reference_cas,
                    ownship_reference_dtg_to_ptp, target_reference_ttg_to_trp);

         if (m_pilot_delay.IsPilotDelayOn()) {
            im_guidance.m_ias_command = m_im_speed_command_with_pilot_delay;
            if (im_guidance.GetSelectedSpeed().GetSpeedType() == MACH_SPEED) {
               const auto true_airspeed_equivalent = m_weather_prediction.CAS2TAS(m_im_speed_command_with_pilot_delay,
                                                                                  current_ownship_state.GetPositionZ());
               const auto mach_equivalent =
                     m_weather_prediction.TAS2Mach(true_airspeed_equivalent, current_ownship_state.GetPositionZ());
               im_guidance.SetMachCommand(mach_equivalent);
            }
         } else {
            im_guidance.m_ias_command = m_im_speed_command_ias;
            if (im_guidance.GetSelectedSpeed().GetSpeedType() == MACH_SPEED) {
               const auto true_airspeed_equivalent =
                     m_weather_prediction.CAS2TAS(m_im_speed_command_ias, current_ownship_state.GetPositionZ());
               const auto mach_equivalent =
                     m_weather_prediction.TAS2Mach(true_airspeed_equivalent, current_ownship_state.GetPositionZ());
               im_guidance.SetMachCommand(mach_equivalent);
            }
         }
      } else {
         m_stage_of_im_operation = MAINTAIN;

         m_ownship_ttg_to_abp = Units::zero();
         m_ownship_kinematic_dtg_to_abp = Units::zero();
         m_predicted_spacing_interval = Units::NegInfinity();

         if (!m_transitioned_to_maintain) {
            m_distance_calculator_target_on_ownship_hpath = AlongPathDistanceCalculator(
                  m_ownship_kinematic_trajectory_predictor.GetHorizontalPath(),
                  TrajectoryIndexProgressionDirection::UNDEFINED, true);  // use a wide cross-track corridor
            m_position_calculator_target_on_ownship_hpath =
                  PositionCalculator(m_ownship_kinematic_trajectory_predictor.GetHorizontalPath(),
                                     TrajectoryIndexProgressionDirection::UNDEFINED);
         }

         // Project target state onto ownship's path
         Units::Length target_distance_along_ownships_path;
         bool dtg_result = m_distance_calculator_target_on_ownship_hpath.CalculateAlongPathDistanceFromPosition(
               current_target_state.GetPositionX(), current_target_state.GetPositionY(),
               target_distance_along_ownships_path);
         if (!dtg_result) {
            // No known recovery from this; throw.
            std::string msg("Unable to project target onto ownship's hpath.");
            LOG4CPLUS_ERROR(m_logger, msg);
            throw std::logic_error(msg);
         }

         if (m_distance_calculator_target_on_ownship_hpath.IsPassedEndOfRoute() || !m_target_aircraft_exists) {
            // Operation is completing. IM Speed must be defaulted; the control law cannot be used.
            m_measured_spacing_interval = Units::NegInfinity();
            if (im_guidance.GetSelectedSpeed().GetSpeedType() == INDICATED_AIR_SPEED) {
               CalculateIas(current_ownship_state.GetPositionZ(), three_dof_dynamics_state);
            } else {
               CalculateMach(Units::negInfinity(), current_ownship_state.GetPositionZ(),
                             three_dof_dynamics_state.current_mass);
            }
            m_im_kinematic_dist_based_maintain->SetImSpeedCommandIas(m_im_speed_command_ias);  // keep the maintain
                                                                                               // object in sync
            m_previous_im_speed_command_ias = m_im_speed_command_ias;

            if (m_pilot_delay.IsPilotDelayOn()) {
               im_guidance.m_ias_command = m_im_speed_command_with_pilot_delay;
            } else {
               im_guidance.m_ias_command = m_im_speed_command_ias;
            }

         } else {
            // Get target's along path position on ownship's path as an AircraftState object
            const Units::Length target_distance_with_asg =
                  target_distance_along_ownships_path + m_assigned_spacing_goal;
            Units::FeetLength target_projected_position_with_asg_x, target_projected_position_with_asg_y;
            Units::UnsignedAngle course;
            m_position_calculator_target_on_ownship_hpath.CalculatePositionFromAlongPathDistance(
                  target_distance_with_asg, target_projected_position_with_asg_x, target_projected_position_with_asg_y,
                  course);
            m_target_state_projected_on_ownships_path_at_adjusted_distance.m_x =
                  target_projected_position_with_asg_x.value();
            m_target_state_projected_on_ownships_path_at_adjusted_distance.m_y =
                  target_projected_position_with_asg_y.value();
            m_target_state_projected_on_ownships_path_at_adjusted_distance.m_z = current_target_state.m_z;
            m_target_state_projected_on_ownships_path_at_adjusted_distance.m_xd = current_target_state.m_xd;
            m_target_state_projected_on_ownships_path_at_adjusted_distance.m_yd = current_target_state.m_yd;
            m_target_state_projected_on_ownships_path_at_adjusted_distance.m_zd = current_target_state.m_zd;

            if (!m_transitioned_to_maintain) {
               m_im_kinematic_dist_based_maintain->Prepare(
                     m_previous_reference_im_speed_command_tas, m_previous_im_speed_command_ias, m_speed_limiter,
                     m_previous_reference_im_speed_command_mach, m_ownship_kinematic_trajectory_predictor,
                     m_im_ownship_distance_calculator, target_adsb_history, m_im_clearance,
                     m_speed_limiter.GetRfLegSpeedLimits());
               m_transitioned_to_maintain = true;
            }

            im_guidance = m_im_kinematic_dist_based_maintain->Update(
                  three_dof_dynamics_state, current_ownship_state,
                  m_target_state_projected_on_ownships_path_at_adjusted_distance, target_distance_with_asg,
                  target_distance_along_ownships_path, m_ownship_kinematic_trajectory_predictor, previous_im_guidance,
                  target_adsb_history, m_ownship_kinematic_achieve_by_calcs,
                  m_target_kinematic_traffic_reference_point_calcs, m_pilot_delay);

            SetActiveFilter(m_im_kinematic_dist_based_maintain->GetActiveFilter());

            if (m_target_kinematic_traffic_reference_point_calcs.IsWaypointSet() &&
                GetTargetKinematicDtgToTrp() > Units::zero()) {
               // special case:  target has not passed TRP, MSI will be negative
               Units::Length ownship_dtg_to_abp =
                     m_ownship_kinematic_achieve_by_calcs.ComputeDistanceToWaypoint(current_ownship_state);
               m_measured_spacing_interval = ownship_dtg_to_abp - GetTargetKinematicDtgToTrp();  // AAES-1036
               LOG4CPLUS_TRACE(m_logger, "Special MSI = " << Units::MetersLength(m_measured_spacing_interval));
            } else {
               // normal case:  target has passed TRP
               m_measured_spacing_interval = Units::NauticalMilesLength(m_im_kinematic_dist_based_maintain->GetMsi());
            }
            m_im_speed_command_ias = m_im_kinematic_dist_based_maintain->GetImSpeedCommandIas();
            m_im_speed_command_with_pilot_delay = m_im_kinematic_dist_based_maintain->GetDelayedImSpeedCommandIas();
            m_unmodified_im_speed_command_ias = m_im_kinematic_dist_based_maintain->GetUnmodifiedImSpeedCommandIas();
            m_previous_im_speed_command_ias = m_im_kinematic_dist_based_maintain->GetPreviousSpeedCommandIas();
         }
      }
   } else {
      im_guidance.SetValid(false);
   }

   return im_guidance;
}

void IMDistBasedAchieve::CalculateIas(const Units::Length current_ownship_altitude,
                                      const aaesim::open_source::DynamicsState &three_dof_dynamics_state) {
   if (!IsTargetPassedTrp()) {
      m_im_speed_command_ias = m_speed_limiter.LimitSpeedCommand(
            m_previous_im_speed_command_ias, m_im_speed_command_ias,
            m_ownship_kinematic_trajectory_predictor.GetVerticalPathCasByIndex(m_ownship_reference_lookup_index),
            m_ownship_kinematic_dtg_to_abp, m_ownship_kinematic_dtg_to_ptp, current_ownship_altitude,
            three_dof_dynamics_state.flap_configuration);
   } else {
      m_predicted_spacing_interval = Units::NegInfinity();

      m_ownship_reference_lookup_index =
            CoreUtils::FindNearestIndex(Units::MetersLength(m_ownship_kinematic_dtg_to_ptp).value(),
                                        m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances());

      Units::Speed nominal_profile_ias = Units::MetersPerSecondSpeed(
            m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(m_ownship_reference_lookup_index));

      m_im_speed_command_ias = m_speed_limiter.LimitSpeedCommand(
            m_previous_im_speed_command_ias, Units::min(nominal_profile_ias, m_previous_im_speed_command_ias),
            m_ownship_kinematic_trajectory_predictor.GetVerticalPathCasByIndex(m_ownship_reference_lookup_index),
            m_ownship_kinematic_dtg_to_abp, m_ownship_kinematic_dtg_to_ptp, current_ownship_altitude,
            three_dof_dynamics_state.flap_configuration);
   }

   if (m_im_speed_command_ias != m_previous_im_speed_command_ias) {
      m_total_number_of_im_speed_changes++;
   }

   if (m_pilot_delay.IsPilotDelayOn()) {
      m_im_speed_command_with_pilot_delay =
            m_pilot_delay.UpdateIAS(m_previous_im_speed_command_ias, m_im_speed_command_ias, current_ownship_altitude,
                                    m_ownship_kinematic_trajectory_predictor.GetAltitudeAtFinalWaypoint());
   }
}

void IMDistBasedAchieve::CalculateMach(const Units::Time reference_ttg, const Units::Length current_ownship_altitude,
                                       const Units::Mass current_mass) {
   BoundedValue<double, 0, 2> estimated_mach(m_previous_reference_im_speed_command_mach);

   if (!IsTargetPassedTrp()) {
      if (!WithinErrorThreshold(
                (m_ownship_kinematic_dtg_to_ptp - m_ownship_kinematic_achieve_by_calcs.GetDistanceFromWaypoint()),
                m_ownship_ttg_to_abp, reference_ttg) ||
          !m_threshold_flag) {
         BoundedValue<double, 0, 3> highbound_estimated_mach(m_previous_reference_im_speed_command_mach);
         Units::Speed estimated_true_airspeed =
               m_weather_prediction.getAtmosphere()->CAS2TAS(m_im_speed_command_ias, current_ownship_altitude);

         highbound_estimated_mach =
               BoundedValue<double, 0, 3>(Units::MetersPerSecondSpeed(estimated_true_airspeed).value() /
                                          sqrt(GAMMA * R.value() *
                                               m_weather_prediction.getAtmosphere()
                                                     ->GetTemperature(Units::FeetLength(current_ownship_altitude))
                                                     .value()));

         // Make sure velocity is within nominal limits (AAES-694)
         Units::MetersPerSecondSpeed nominal_indicated_airspeed(
               m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(
                     m_ownship_reference_lookup_index));
         Units::Speed nominal_true_airspeed =
               m_weather_prediction.getAtmosphere()->CAS2TAS(nominal_indicated_airspeed, current_ownship_altitude);
         BoundedValue<double, 0, 2> nominal_mach =
               BoundedValue<double, 0, 2>(Units::MetersPerSecondSpeed(nominal_true_airspeed).value() /
                                          sqrt(GAMMA * R.value() *
                                               m_weather_prediction.getAtmosphere()
                                                     ->GetTemperature(Units::FeetLength(current_ownship_altitude))
                                                     .value()));

         double unbounded_estimated_mach(highbound_estimated_mach);
         if (unbounded_estimated_mach > 2) unbounded_estimated_mach = 2;
         estimated_mach =
               m_speed_limiter.LimitMachCommand(BoundedValue<double, 0, 2>(m_previous_reference_im_speed_command_mach),
                                                BoundedValue<double, 0, 2>(unbounded_estimated_mach), nominal_mach,
                                                current_mass, current_ownship_altitude, m_weather_prediction);
      }
   } else {
      m_predicted_spacing_interval = Units::NegInfinity();

      m_ownship_reference_lookup_index =
            CoreUtils::FindNearestIndex(Units::MetersLength(m_ownship_kinematic_dtg_to_ptp).value(),
                                        m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances());

      Units::MetersPerSecondSpeed nominal_indicated_airspeed(
            m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(m_ownship_reference_lookup_index));
      Units::Speed nominal_true_airspeed =
            m_weather_prediction.getAtmosphere()->CAS2TAS(nominal_indicated_airspeed, current_ownship_altitude);
      double nominal_mach = Units::MetersPerSecondSpeed(nominal_true_airspeed).value() /
                            sqrt(GAMMA * R.value() *
                                 m_weather_prediction.getAtmosphere()
                                       ->GetTemperature(Units::FeetLength(current_ownship_altitude))
                                       .value());

      estimated_mach = std::min(nominal_mach, m_previous_reference_im_speed_command_mach);
   }

   if (estimated_mach != m_previous_reference_im_speed_command_mach) {
      m_total_number_of_im_speed_changes++;
   }

   m_im_speed_command_ias = m_weather_prediction.getAtmosphere()->MachToIAS(estimated_mach, current_ownship_altitude);

   if (m_pilot_delay.IsPilotDelayOn()) {
      m_im_speed_command_with_pilot_delay = m_pilot_delay.UpdateMach(
            m_previous_reference_im_speed_command_mach, estimated_mach, current_ownship_altitude,
            m_ownship_kinematic_trajectory_predictor.GetAltitudeAtFinalWaypoint());
   }
   m_previous_reference_im_speed_command_mach = estimated_mach;
}

void IMDistBasedAchieve::RecordData(aaesim::open_source::Guidance &im_guidance,
                                    const interval_management::open_source::AircraftState &current_ownship_state,
                                    const interval_management::open_source::AircraftState &current_target_state,
                                    const aaesim::open_source::DynamicsState &three_dof_dynamics_state,
                                    Units::Speed unmodified_im_speed_command_ias, Units::Speed im_speed_command_tas,
                                    Units::Speed ownship_reference_cas, Units::Length ownship_reference_dtg_to_ptp,
                                    Units::Time target_reference_ttg_to_trp) {
   m_true_distances->Add(Units::SecondsTime(current_ownship_state.GetTimeStamp().value()),
                         Units::MetersLength(Units::infinity()));

   if (InternalObserver::getInstance()->outputNM()) {
      NMObserver &nm_observer = InternalObserver::getInstance()->GetNMObserver(current_ownship_state.GetId());

      if (nm_observer.curr_NM == -2 && im_guidance.IsValid()) {
         nm_observer.curr_NM = static_cast<int>(Units::NauticalMilesLength(m_ownship_kinematic_dtg_to_ptp).value());
      }
      if (m_ownship_kinematic_dtg_to_ptp <= Units::NauticalMilesLength(nm_observer.curr_NM) && im_guidance.IsValid()) {
         nm_observer.curr_NM--;

         double minimum_allowable_reference_im_speed_cas =
               m_speed_limiter.LowLimit(m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(
                     m_ownship_reference_lookup_index));
         double maximum_allowable_reference_im_speed_cas =
               m_speed_limiter.HighLimit(m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(
                     m_ownship_reference_lookup_index));

         double minimum_allowable_reference_im_speed_tas =
               Units::MetersPerSecondSpeed(m_weather_prediction.getAtmosphere()->CAS2TAS(
                                                 Units::MetersPerSecondSpeed(minimum_allowable_reference_im_speed_cas),
                                                 current_ownship_state.GetPositionZ()))
                     .value();
         double maximum_allowable_reference_im_speed_tas =
               Units::MetersPerSecondSpeed(m_weather_prediction.getAtmosphere()->CAS2TAS(
                                                 Units::MetersPerSecondSpeed(maximum_allowable_reference_im_speed_cas),
                                                 current_ownship_state.GetPositionZ()))
                     .value();

         nm_observer.output_NM_values(
               Units::MetersLength(m_ownship_kinematic_dtg_to_ptp).value(),
               Units::MetersLength(Units::infinity()).value(), current_ownship_state.GetTimeStamp().value(),
               Units::MetersPerSecondSpeed(m_im_speed_command_ias).value(),
               Units::MetersPerSecondSpeed(current_ownship_state.GetGroundSpeed()).value(),
               Units::MetersPerSecondSpeed(current_target_state.GetGroundSpeed()).value(),
               minimum_allowable_reference_im_speed_cas, maximum_allowable_reference_im_speed_cas,
               minimum_allowable_reference_im_speed_tas, maximum_allowable_reference_im_speed_tas);
      }
   }

   InternalObserver::getInstance()->addAchieveRcd(
         static_cast<size_t>(current_ownship_state.GetId()), current_ownship_state.GetTimeStamp().value(),
         Units::SecondsTime(target_reference_ttg_to_trp).value(), Units::SecondsTime(m_ownship_ttg_to_abp).value(),
         Units::MetersLength(-m_ownship_kinematic_dtg_to_ptp).value(),
         Units::MetersLength(ownship_reference_dtg_to_ptp).value());
}

const Units::MetersLength IMDistBasedAchieve::CalculatePredictedSpacingInterval(
      const Units::Time target_reference_ttg_to_trp) {
   const auto ownship_reference_dtg_to_ptp_index =
         CoreUtils::FindNearestIndex(Units::MetersLength(m_ownship_kinematic_dtg_to_ptp).value(),
                                     m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances());

   const Units::Time own_ttg_to_ptp = Units::SecondsTime(CoreUtils::LinearlyInterpolate(
         ownship_reference_dtg_to_ptp_index, Units::MetersLength(m_ownship_kinematic_dtg_to_ptp).value(),
         m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances(),
         m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes()));

   const Units::Time ownship_kinematic_ttg_to_abp =
         own_ttg_to_ptp - m_ownship_kinematic_achieve_by_calcs.GetTimeToGoToWaypoint();
   const Units::Time ownship_target_delta_ttg = ownship_kinematic_ttg_to_abp - target_reference_ttg_to_trp;
   const Units::SecondsTime predicted_spacing_interval_lookup_ttg =
         ownship_target_delta_ttg + m_ownship_kinematic_achieve_by_calcs.GetTimeToGoToWaypoint();
   const Units::Length ownship_dtg_from_abp_to_ptp = m_ownship_kinematic_achieve_by_calcs.GetDistanceFromWaypoint();
   const auto index_ttg = CoreUtils::FindNearestIndex(predicted_spacing_interval_lookup_ttg.value(),
                                                      m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes());

   if ((ownship_target_delta_ttg < Units::zero()) && (index_ttg == 0)) {
      // This calculation handles negative PSI situations in which there is no maintain stage afterward. In this
      // situation, the PSI must be guessed using ownship's predicted groundspeed.
      const Units::Speed ownship_groundspeed_at_ptp =
            Units::MetersPerSecondSpeed(m_ownship_kinematic_trajectory_predictor.GetVerticalPathGroundspeeds().front());
      const Units::Time ownship_time_past_ptp = target_reference_ttg_to_trp - own_ttg_to_ptp;
      const Units::Length ownship_distance_passed_abp =
            ownship_groundspeed_at_ptp * ownship_time_past_ptp + ownship_dtg_from_abp_to_ptp;
      const Units::Length predicted_spacing_interval = -ownship_distance_passed_abp;

      // expect this to be negative
      return predicted_spacing_interval;
   } else {
      // This calculation handles:
      // ** positive PSI situations
      // ** negative PSI situations in which there is a valid 4D prediction that goes beyond the ABP
      const Units::Length ownship_reference_dtg_at_predicted_spacing_interval_lookup_distance = Units::MetersLength(
            CoreUtils::LinearlyInterpolate(index_ttg, predicted_spacing_interval_lookup_ttg.value(),
                                           m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes(),
                                           m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances()));

      const Units::Length predicted_spacing_interval =
            ownship_reference_dtg_at_predicted_spacing_interval_lookup_distance - ownship_dtg_from_abp_to_ptp;

      return predicted_spacing_interval;
   }
}

IMDistBasedAchieve &IMDistBasedAchieve::operator=(const IMDistBasedAchieve &obj) {
   if (this != &obj) {
      Copy(obj);
   }

   return *this;
}

void IMDistBasedAchieve::DumpParameters(const std::string &parameters_to_print) {
   LOG4CPLUS_DEBUG(IMDistBasedAchieve::m_logger, "\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\" << std::endl);
   LOG4CPLUS_DEBUG(IMDistBasedAchieve::m_logger, parameters_to_print.c_str() << std::endl << std::endl);

   IMKinematicAchieve::DumpParameters(parameters_to_print);

   LOG4CPLUS_DEBUG(
         IMDistBasedAchieve::m_logger,
         "assigned spacing goal: " << Units::NauticalMilesLength(m_assigned_spacing_goal).value() << std::endl);
   LOG4CPLUS_DEBUG(IMDistBasedAchieve::m_logger, "\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\" << std::endl);
}
