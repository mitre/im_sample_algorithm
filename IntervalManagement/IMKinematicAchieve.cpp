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

#include <iomanip>
#include "math/CustomMath.h"
#include "public/AircraftCalculations.h"
#include "public/CoreUtils.h"
#include "public/Environment.h"
#include "public/StandardAtmosphere.h"
#include "imalgs/IMClearanceLoader.h"
#include "imalgs/IMKinematicAchieve.h"
#include "imalgs/IMScenario.h"

using namespace interval_management;
using namespace interval_management::open_source;

log4cplus::Logger IMKinematicAchieve::logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("IMKinematicAchieve"));
const int IMKinematicAchieve::MINIMUM_FAS_TRACK_COUNT(5);
const Units::FeetLength IMKinematicAchieve::TARGET_ALTITUDE_TOLERANCE(3000);
const Units::SecondsTime IMKinematicAchieve::TRACK_ANGLE_TAU(3.0);

IMKinematicAchieve::IMKinematicAchieve()
   : m_ownship_kinematic_trajectory_predictor(),
     m_target_kinematic_trajectory_predictor(),
     m_ownship_kinematic_achieve_by_calcs(),
     m_traffic_reference_point(),
     m_target_kinematic_traffic_reference_point_calcs(),
     m_ownship_aircraft_intent(),
     m_ownship_distance_calculator(),
     m_target_distance_calculator(),
     m_im_ownship_distance_calculator(),
     m_ownship_track_angle_history(),
     m_target_track_angle_history(),
     m_assigned_spacing_goal_from_input_file(0),
     m_target_altitude_failure_count(0),
     m_fas_intent_valid(false),
     m_compute_ownship_kinematic_trajectory(true),
     m_compute_target_kinematic_trajectory(true),
     m_target_aircraft_exists(false),
     m_target_history_exists(false),
     m_is_target_aligned(false),
     m_new_trajectory_prediction_available(),
     m_blend_wind(IMScenario::BLEND_WIND_DEFAULT),
     m_tangent_plane_sequence(nullptr) {
   IterClearIMKinAch();
}

void IMKinematicAchieve::IterationReset() {
   IMAchieve::IterationReset();
   IterClearIMKinAch();
}

void IMKinematicAchieve::IterClearIMKinAch() {
   m_target_altitude_failure_count = 0;

   m_new_trajectory_prediction_available = false;
   m_compute_ownship_kinematic_trajectory = true;
   m_compute_target_kinematic_trajectory = true;

   m_target_aircraft_exists = false;
   m_target_history_exists = false;

   aaesim::open_source::KinematicTrajectoryPredictor dummytraj;

   m_ownship_kinematic_trajectory_predictor = dummytraj;
   m_target_kinematic_trajectory_predictor = dummytraj;

   m_ownship_kinematic_achieve_by_calcs = interval_management::open_source::AchievePointCalcs();
   m_target_kinematic_traffic_reference_point_calcs = interval_management::open_source::AchievePointCalcs();

   m_fas_intent_valid = false;
   m_is_target_aligned = false;
}

void IMKinematicAchieve::Initialize(const OwnshipPredictionParameters &ownship_prediction_parameters,
                                    const AircraftIntent &ownship_aircraft_intent,
                                    WeatherPrediction &weather_prediction) {
   IMAchieve::Initialize(ownship_prediction_parameters, ownship_aircraft_intent, weather_prediction);

   SetTangentPlaneSequence(std::move(ownship_aircraft_intent.GetTangentPlaneSequence()));
   m_ownship_track_angle_history.clear();
   m_target_track_angle_history.clear();
   m_fas_intent_valid = false;
   m_is_target_aligned = false;
   interval_management::open_source::FIMSpeedQuantizer quantizer = interval_management::open_source::FIMSpeedQuantizer(
         m_loaded_middle_to_final_quantize_transition_distance, m_loaded_first_to_middle_quantize_transition_distance,
         m_loaded_speed_quantize_final_phase, m_loaded_speed_quantize_middle_phase,
         m_loaded_speed_quantize_first_phase);
   m_speed_limiter = interval_management::open_source::FIMSpeedLimiter(
         m_loaded_use_speed_quantization, m_loaded_use_speed_limiting, ownship_prediction_parameters.flap_speeds,
         ownship_prediction_parameters.flight_envelope, ownship_prediction_parameters.mass_data,
         ownship_prediction_parameters.aerodynamics, quantizer);

   // --- Prepare Ownship objects -------------
   // Strip any ownship waypoints after the PTP
   m_ownship_aircraft_intent = ownship_aircraft_intent;
   TrimAircraftIntentAfterWaypoint(m_ownship_aircraft_intent, m_im_clearance.GetPlannedTerminationPoint());

   // Create the kinematic trajectory predictors using ownship assumptions
   m_ownship_kinematic_trajectory_predictor = aaesim::open_source::KinematicTrajectoryPredictor(
         ownship_prediction_parameters.maximum_allowable_bank_angle, ownship_prediction_parameters.transition_ias,
         ownship_prediction_parameters.transition_mach, ownship_prediction_parameters.transition_altitude,
         ownship_prediction_parameters.expected_cruise_altitude);
   m_ownship_kinematic_trajectory_predictor.CalculateWaypoints(m_ownship_aircraft_intent, weather_prediction);

   // Initialize these objects with an estimated horizontal path. The horizontal paths will be updated to a more precise
   // calculation during initial trajectory prediction.
   if (m_im_clearance.GetClearanceType() == IMClearance::MAINTAIN ||
       m_im_clearance.GetClearanceType() == IMClearance::CAPTURE) {
      m_im_ownship_distance_calculator = AlongPathDistanceCalculator::CreateForCaptureClearance(
            m_ownship_kinematic_trajectory_predictor.EstimateHorizontalTrajectory(weather_prediction));
   } else {
      m_im_ownship_distance_calculator = AlongPathDistanceCalculator(
            m_ownship_kinematic_trajectory_predictor.EstimateHorizontalTrajectory(weather_prediction),
            TrajectoryIndexProgressionDirection::UNDEFINED);
   }
   m_ownship_distance_calculator = AlongPathDistanceCalculator(
         m_ownship_kinematic_trajectory_predictor.EstimateHorizontalTrajectory(weather_prediction),
         TrajectoryIndexProgressionDirection::UNDEFINED);

   // --- Prepare Target objects -------------
   m_target_kinematic_trajectory_predictor = aaesim::open_source::KinematicTrajectoryPredictor(
         ownship_prediction_parameters.maximum_allowable_bank_angle, ownship_prediction_parameters.transition_ias,
         ownship_prediction_parameters.transition_mach, ownship_prediction_parameters.transition_altitude,
         ownship_prediction_parameters.expected_cruise_altitude);

   m_target_kinematic_trajectory_predictor.CalculateWaypoints(m_target_aircraft_intent, weather_prediction);
   m_target_distance_calculator = AlongPathDistanceCalculator(
         m_target_kinematic_trajectory_predictor.EstimateHorizontalTrajectory(weather_prediction),
         TrajectoryIndexProgressionDirection::DECREMENTING);
}

void IMKinematicAchieve::ResetDefaults() {
   if (m_blend_wind != IMScenario::BLEND_WIND_DEFAULT) {
      LOG4CPLUS_WARN(logger, "mBlendWind reset to " << IMScenario::BLEND_WIND_DEFAULT << RESET_MSG);
      m_blend_wind = IMScenario::BLEND_WIND_DEFAULT;
   }

   IMAchieve::ResetDefaults();
}

void IMKinematicAchieve::TrimAircraftIntentAfterWaypoint(AircraftIntent &aircraft_intent,
                                                         const std::string &waypoint_name) {

   if (waypoint_name.empty()) return;  // can only trim a real waypoint

   int index = 0;
   const int maxIndex = aircraft_intent.GetNumberOfWaypoints();

   LOG4CPLUS_DEBUG(logger, "Trim AircraftIntent to " << waypoint_name);
   LOG4CPLUS_TRACE(logger, "Intent before trim:" << std::endl << aircraft_intent);

   for (index = 0; index < maxIndex; index++) {
      if (aircraft_intent.GetWaypointName(index) == waypoint_name) {
         aircraft_intent.SetNumberOfWaypoints(index + 1);
         break;
      }
   }

   LOG4CPLUS_TRACE(logger, "Intent after trim:" << std::endl << aircraft_intent);

   if (index >= maxIndex) {
      // NOTE: This is here for completeness. IMClearance::validate() should ensure this cannot occur.
      std::string msg = "ERROR: Could not find trim point (" + waypoint_name + ") in list of Waypoints";
      LOG4CPLUS_ERROR(IMKinematicAchieve::logger, msg);
      throw std::logic_error(msg);
   }
}

bool IMKinematicAchieve::load(DecodedStream *input) {
   set_stream(input);

   register_loadable_with_brackets("configuration", &m_configuration, true);

   LoaderDeprecatedMetaInfo spacingDeprecatedInfo;
   spacingDeprecatedInfo.isDeprecated = true;
   spacingDeprecatedInfo.supersededByTagName = "IMClearance.Assigned_Spacing_Goal";

   register_var("spacing", &m_assigned_spacing_goal_from_input_file, spacingDeprecatedInfo);

   interval_management::IMClearanceLoader imloader;
   register_loadable_with_brackets("clearance", &imloader, false);

   m_loaded = complete();

   m_im_clearance = imloader.BuildClearance();
   CopyParametersFromConfiguration();

   return m_loaded;
}

aaesim::open_source::Guidance IMKinematicAchieve::Update(
      const aaesim::open_source::Guidance &prevguidance, const aaesim::open_source::DynamicsState &dynamicsstate,
      const interval_management::open_source::AircraftState &owntruthstate,
      const interval_management::open_source::AircraftState &targetsyncstate,
      const vector<interval_management::open_source::AircraftState> &targethistory) {
   aaesim::open_source::Guidance guidance_out =
         IMAchieve::Update(prevguidance, dynamicsstate, owntruthstate, targetsyncstate, targethistory);

   m_target_aircraft_exists = targetsyncstate.GetId() != IMUtils::UNINITIALIZED_AIRCRAFT_ID;
   m_target_history_exists = !targethistory.empty();

   if (m_ownship_track_angle_history.size() < MINIMUM_FAS_TRACK_COUNT) {
      m_ownship_track_angle_history.push_back(Units::SignedRadiansAngle(atan2(owntruthstate.m_yd, owntruthstate.m_xd)));
   }

   if (m_target_aircraft_exists) {
      if (m_target_track_angle_history.size() < MINIMUM_FAS_TRACK_COUNT) {
         m_target_track_angle_history.push_back(
               Units::SignedRadiansAngle(atan2(targetsyncstate.m_yd, targetsyncstate.m_xd)));
      }
   }

   UpdatePositionMetrics(owntruthstate, targetsyncstate);

   CheckPredictionAccuracy(owntruthstate, targetsyncstate);

   if (m_im_clearance.GetClearanceType() == IMClearance::ClearanceType::FAS) {
      size_t ownship_track_count = m_ownship_track_angle_history.size();
      size_t target_track_count = m_target_track_angle_history.size();
      if (ownship_track_count < MINIMUM_FAS_TRACK_COUNT || target_track_count < MINIMUM_FAS_TRACK_COUNT) {
         guidance_out = prevguidance;
         guidance_out.SetValid(false);

         m_compute_ownship_kinematic_trajectory = true;
         m_compute_target_kinematic_trajectory = true;
         return guidance_out;
      }

      if (!m_fas_intent_valid) {
         ComputeFASTrajectories(owntruthstate, targetsyncstate);
      }
   }

   HandleTrajectoryPrediction(owntruthstate, targetsyncstate, targethistory);

   if ((m_target_aircraft_exists || m_target_history_exists) && m_received_one_valid_target_state &&
       !m_compute_target_kinematic_trajectory) {
      CalculateTargetDtgToImPoints(targetsyncstate);
      guidance_out.SetValid(true);
   } else {
      guidance_out.SetValid(false);
   }

   return guidance_out;
}

void IMKinematicAchieve::HandleTrajectoryPrediction(
      const interval_management::open_source::AircraftState &owntruthstate,
      const interval_management::open_source::AircraftState &targetsyncstate,
      const vector<interval_management::open_source::AircraftState> &target_adsb_history) {
   m_new_trajectory_prediction_available =
         m_compute_ownship_kinematic_trajectory || m_compute_target_kinematic_trajectory;

   if (m_compute_ownship_kinematic_trajectory) {
      LOG4CPLUS_DEBUG(logger, "compute ownship kinematic trajectory: " << owntruthstate.GetId());
      m_ownship_kinematic_trajectory_predictor.CalculateWaypoints(m_ownship_aircraft_intent, m_weather_prediction);

      std::vector<HorizontalPath> ownship_horizontal_path =
            m_ownship_kinematic_trajectory_predictor.GetHorizontalPath();
      if (ownship_horizontal_path.empty()) {
         ownship_horizontal_path =
               m_ownship_kinematic_trajectory_predictor.EstimateHorizontalTrajectory(m_weather_prediction);
      }

      Units::MetersLength ownship_distance_to_go = Units::MetersLength(Units::infinity());
      Units::RadiansAngle dummy_course = Units::RadiansAngle(0);
      std::vector<HorizontalPath>::size_type dummy_index = 0;
      if (!AircraftCalculations::CalculateDistanceAlongPathFromPosition(
                Units::FeetLength(owntruthstate.m_x), Units::FeetLength(owntruthstate.m_y), ownship_horizontal_path, 0,
                ownship_distance_to_go, dummy_course, dummy_index)) {
         ownship_distance_to_go = Units::MetersLength(Units::infinity());
      }

      m_ownship_kinematic_trajectory_predictor.BuildTrajectoryPrediction(
            m_weather_prediction, Units::FeetLength(owntruthstate.m_z), ownship_distance_to_go);

      std::shared_ptr<aaesim::open_source::KinematicDescent4DPredictor> kinematic_descent_4d_predictor =
            m_ownship_kinematic_trajectory_predictor.GetKinematicDescent4dPredictor();
      CalculateRFLegPhase(m_ownship_kinematic_trajectory_predictor.GetPrecalcWaypoints(),
                          Units::MetersSecondAcceleration(kinematic_descent_4d_predictor->GetDecelerationRateFPA()),
                          kinematic_descent_4d_predictor->GetVerticalPath(),
                          m_ownship_kinematic_trajectory_predictor.GetHorizontalPath());

      m_ownship_kinematic_achieve_by_calcs = interval_management::open_source::AchievePointCalcs(
            m_achieve_by_point, m_ownship_aircraft_intent,
            m_ownship_kinematic_trajectory_predictor.GetKinematicDescent4dPredictor()->GetVerticalPath(),
            m_ownship_kinematic_trajectory_predictor.GetHorizontalPath());

      m_ownship_distance_calculator.UpdateHorizontalTrajectory(
            m_ownship_kinematic_trajectory_predictor.GetHorizontalPath());
      m_im_ownship_distance_calculator.UpdateHorizontalTrajectory(
            m_ownship_kinematic_trajectory_predictor.GetHorizontalPath());

      m_compute_ownship_kinematic_trajectory = false;
   }

   CalculateOwnshipDtgToPlannedTerminationPoint(owntruthstate);
   CalculateOwnshipDtgToAchieveByPoint();

   if (m_target_aircraft_exists && m_target_history_exists && m_compute_target_kinematic_trajectory) {
      if (!InAchieveStage()) {
         LOG4CPLUS_DEBUG(logger, "Skipping target prediction for ac " << owntruthstate.GetId());
      } else {
         LOG4CPLUS_DEBUG(logger, "ac " << owntruthstate.GetId()
                                       << " compute target kinematic trajectory: " << targetsyncstate.GetId());
         const Units::Speed targetgroundspeed =
               Units::FeetPerSecondSpeed(sqrt(pow(targetsyncstate.m_xd, 2) + pow(targetsyncstate.m_yd, 2)));
         const Units::Angle targetgroundtrack = Units::RadiansAngle(atan2(targetsyncstate.m_yd, targetsyncstate.m_xd));

         // Get Winds at Target Aircraft's current altitude
         Units::MetersPerSecondSpeed Vwx, Vwy;
         Units::HertzFrequency dVwx_dh, dVwy_dh;
         m_weather_prediction.getAtmosphere()->CalculateWindGradientAtAltitude(
               Units::FeetLength(targetsyncstate.m_z), m_weather_prediction.east_west, Vwx, dVwx_dh);
         m_weather_prediction.getAtmosphere()->CalculateWindGradientAtAltitude(
               Units::FeetLength(targetsyncstate.m_z), m_weather_prediction.north_south, Vwy, dVwy_dh);

         // calculate wind parallel and perpendicular to the ground track
         Units::Speed Vw_para = Vwx * cos(targetgroundtrack) + Vwy * sin(targetgroundtrack);
         Units::Speed Vw_perp = -Vwx * sin(targetgroundtrack) + Vwy * cos(targetgroundtrack);

         // Estimate current airspeed (Mach)
         // 1. estimate target mach
         // 1a. ensure limited by values below
         // 1b. quantize at 0.02 value
         //
         // 2. Find highest CAS restriction on target route & use
         // 2a. else, use ownship descent CAS
         // 2b. don't allow below 270
         const Units::FeetLength estimated_cruise_altitude(target_adsb_history.front().m_z);
         const Units::Speed targetvtas = Units::sqrt(Units::sqr(targetgroundspeed - Vw_para) + Units::sqr(Vw_perp));
         double estimated_cruise_mach = Units::MetersPerSecondSpeed(targetvtas).value() /
                                        sqrt(GAMMA * R.value() *
                                             m_weather_prediction.getAtmosphere()
                                                   ->GetTemperature(Units::FeetLength(targetsyncstate.m_z))
                                                   .value());
         const double min_mach_descent = 0.7;
         const double max_mach_descent = 0.86;
         estimated_cruise_mach = std::min(estimated_cruise_mach, max_mach_descent);
         estimated_cruise_mach = std::max(estimated_cruise_mach, min_mach_descent);
         estimated_cruise_mach = quantize(estimated_cruise_mach, 0.02);  // we don't want this value being a float

         const Units::KnotsSpeed min_allowable_descent_cas(270.0);
         const Units::Speed target_descent_cas =
               max(IMUtils::FindLargestSpeedConstraint(m_target_aircraft_intent), min_allowable_descent_cas);
         Units::Length target_transition_altitude =
               m_weather_prediction.getAtmosphere()->GetMachIASTransition(target_descent_cas, estimated_cruise_mach);
         if (estimated_cruise_altitude < target_transition_altitude) {
            // there isn't enough information to guess at a cruise mach condition. Set mach to
            // zero and transition altitude to negInfinity so that no mach transition is predicted.
            estimated_cruise_mach = 0;
            target_transition_altitude = Units::Length(Units::negInfinity());
         }

         if (m_im_clearance.GetClearanceType() == IMClearance::ClearanceType::FAS) {
            m_target_kinematic_trajectory_predictor.GetKinematicDescent4dPredictor()->SetMembers(
                  estimated_cruise_mach, target_descent_cas, estimated_cruise_altitude,
                  m_weather_prediction.getAtmosphere()->GetMachIASTransition(target_descent_cas,
                                                                             estimated_cruise_mach));
         } else {
            m_target_kinematic_trajectory_predictor.GetKinematicDescent4dPredictor()->SetMembers(
                  estimated_cruise_mach, target_descent_cas, max(target_transition_altitude, estimated_cruise_altitude),
                  m_weather_prediction.getAtmosphere()->GetMachIASTransition(target_descent_cas,
                                                                             estimated_cruise_mach));
         }
         m_target_kinematic_trajectory_predictor.CalculateWaypoints(m_target_aircraft_intent, m_weather_prediction);

         Units::RadiansAngle dummy_course = Units::RadiansAngle(0);
         std::vector<HorizontalPath>::size_type dummy_index = 0;
         std::vector<HorizontalPath> target_horizontal_path =
               m_target_kinematic_trajectory_predictor.GetHorizontalPath();
         Units::MetersLength target_distance_to_go = Units::MetersLength(Units::infinity());
         Units::MetersLength target_start_altitude_msl = Units::FeetLength(targetsyncstate.m_z);

         if (target_horizontal_path.empty()) {
            target_horizontal_path =
                  m_target_kinematic_trajectory_predictor.EstimateHorizontalTrajectory(m_weather_prediction);
         }

         if (!IsTargetPassedTrp()) {
            if (!AircraftCalculations::CalculateDistanceAlongPathFromPosition(
                      Units::FeetLength(targetsyncstate.m_x), Units::FeetLength(targetsyncstate.m_y),
                      target_horizontal_path, 0, target_distance_to_go, dummy_course, dummy_index)) {
               target_distance_to_go = Units::MetersLength(Units::infinity());
            }
         }

         const std::string trp_name{m_im_clearance.GetTrafficReferencePoint()};
         if (trp_name == "CALCULATED_TRP") {
            // calculate TRP
            Units::MetersLength x, y;
            size_t index;
            interval_management::open_source::AchievePointCalcs::ComputeDefaultTRP(
                  m_ownship_kinematic_achieve_by_calcs, m_ownship_aircraft_intent, m_target_aircraft_intent,
                  target_horizontal_path, m_traffic_reference_point, x, y, index);
            m_target_aircraft_intent.InsertWaypointAtIndex(m_traffic_reference_point, index);
         } else {
            // retrieve TRP from target intent
            const size_t trp_index(m_target_aircraft_intent.GetWaypointIndexByName(trp_name));
            m_traffic_reference_point = m_target_aircraft_intent.GetWaypoint(trp_index);
         }

         TrimAircraftIntentAfterWaypoint(m_target_aircraft_intent, m_traffic_reference_point.GetName());

         SetTrafficReferencePointConstraints(owntruthstate, targetsyncstate);

         m_target_kinematic_trajectory_predictor.CalculateWaypoints(m_target_aircraft_intent, m_weather_prediction);
         m_target_kinematic_trajectory_predictor.BuildTrajectoryPrediction(
               m_weather_prediction, target_start_altitude_msl, target_distance_to_go);
         m_target_kinematic_traffic_reference_point_calcs = interval_management::open_source::AchievePointCalcs(
               m_im_clearance.GetTrafficReferencePoint(), m_target_aircraft_intent,
               m_target_kinematic_trajectory_predictor.GetKinematicDescent4dPredictor()->GetVerticalPath(),
               m_target_kinematic_trajectory_predictor.GetHorizontalPath());

         m_target_distance_calculator.UpdateHorizontalTrajectory(
               m_target_kinematic_trajectory_predictor.GetHorizontalPath());
      }
      m_compute_target_kinematic_trajectory = false;
   }
}

/**
 * Checks weather, atmosphere, and trajectory prediction accuracy,
 * performs wind blending, and sets flags to recompute kinematic
 * trajectories if needed.
 */
void IMKinematicAchieve::CheckPredictionAccuracy(
      const interval_management::open_source::AircraftState &owntruthstate,
      const interval_management::open_source::AircraftState &targettruthstate) {

   if (IsBlendWind() && !m_compute_ownship_kinematic_trajectory) {
      CalculateOwnshipDtgToPlannedTerminationPoint(owntruthstate);

      int ownship_reference_index =
            CoreUtils::FindNearestIndex(Units::MetersLength(m_ownship_kinematic_dtg_to_ptp).value(),
                                        m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances());
      if (ownship_reference_index + 1 >= m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes().size()) {
         // end
         m_ownship_reference_cas =
               Units::MetersPerSecondSpeed(m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities().back());
         m_ownship_reference_altitude =
               Units::MetersLength(m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes().back());
      } else if (ownship_reference_index == 0) {
         // beginning
         m_ownship_reference_cas =
               Units::MetersPerSecondSpeed(m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(0));
         m_ownship_reference_altitude =
               Units::MetersLength(m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudeByIndex(0));
      } else {
         // normal case, middle somewhere, so interpolate
         m_ownship_reference_cas = Units::MetersPerSecondSpeed(CoreUtils::LinearlyInterpolate(
               ownship_reference_index, Units::MetersLength(m_ownship_kinematic_dtg_to_ptp).value(),
               m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances(),
               m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities()));
         m_ownship_reference_altitude = Units::MetersLength(CoreUtils::LinearlyInterpolate(
               ownship_reference_index, Units::MetersLength(m_ownship_kinematic_dtg_to_ptp).value(),
               m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances(),
               m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes()));
      }

      std::shared_ptr<Atmosphere> sensed_atmosphere(StandardAtmosphere::MakeInstance(
            owntruthstate.GetSensedTemperature(), Units::FeetLength(owntruthstate.m_z)));

      if (!m_predicted_wind_evaluator->ArePredictedWindsAccurate(
                IMUtils::ConvertToAaesimAircraftState(owntruthstate), m_weather_prediction, m_ownship_reference_cas,
                m_ownship_reference_altitude, sensed_atmosphere.get())) {

         Wind::UpdatePredictedWindsAtAltitudeFromSensedWind(IMUtils::ConvertToAaesimAircraftState(owntruthstate),
                                                            m_weather_prediction);
         m_compute_ownship_kinematic_trajectory = true;
         m_weather_prediction.SetAtmosphere(sensed_atmosphere);
         LOG4CPLUS_DEBUG(logger, "Bad wind prediction at t="
                                       << owntruthstate.GetTimeStamp().value() << ", target ac exists="
                                       << m_target_aircraft_exists << ", ref_cas=" << m_ownship_reference_cas
                                       << ", ref_alt=" << Units::FeetLength(m_ownship_reference_altitude)
                                       << ", true_alt=" << Units::FeetLength(owntruthstate.m_z) << ", recalc="
                                       << "true");
      }
   }

   if (IsBlendWind() && m_target_aircraft_exists && !m_compute_target_kinematic_trajectory &&
       m_stage_of_im_operation != FlightStage::MAINTAIN) {
      CalculateTargetDtgToImPoints(targettruthstate);

      int target_reference_index =
            CoreUtils::FindNearestIndex(Units::MetersLength(m_target_kinematic_dtg_to_last_waypoint).value(),
                                        m_target_kinematic_trajectory_predictor.GetVerticalPathDistances());
      if (target_reference_index + 1 >= m_target_kinematic_trajectory_predictor.GetVerticalPathTimes().size()) {
         // end
         m_target_reference_altitude =
               Units::MetersLength(m_target_kinematic_trajectory_predictor.GetVerticalPathAltitudes().back());
      } else if (target_reference_index == 0) {
         // beginning
         m_target_reference_altitude =
               Units::MetersLength(m_target_kinematic_trajectory_predictor.GetVerticalPathAltitudeByIndex(0));
      } else {
         // normal case, middle somewhere, so interpolate
         m_target_reference_altitude = Units::MetersLength(CoreUtils::LinearlyInterpolate(
               target_reference_index, Units::MetersLength(m_target_kinematic_dtg_to_last_waypoint).value(),
               m_target_kinematic_trajectory_predictor.GetVerticalPathDistances(),
               m_target_kinematic_trajectory_predictor.GetVerticalPathAltitudes()));
      }

      Units::FeetLength target_altitude_error = m_target_reference_altitude - Units::FeetLength(targettruthstate.m_z);
      if (abs(target_altitude_error) > TARGET_ALTITUDE_TOLERANCE) {
         if (++m_target_altitude_failure_count >= 2) {
            m_compute_target_kinematic_trajectory = true;
         }
         LOG4CPLUS_DEBUG(logger, "Target altitude ref=" << Units::FeetLength(m_target_reference_altitude)
                                                        << ", true=" << Units::FeetLength(targettruthstate.m_z) << ", "
                                                        << m_target_altitude_failure_count << " iterations at t="
                                                        << targettruthstate.GetTimeStamp().value());
      } else {
         m_target_altitude_failure_count = 0;
      }
   }
}

void IMKinematicAchieve::CalculateOwnshipDtgToPlannedTerminationPoint(
      const interval_management::open_source::AircraftState &current_ownship_state) {

   m_ownship_distance_calculator.CalculateAlongPathDistanceFromPosition(Units::FeetLength(current_ownship_state.m_x),
                                                                        Units::FeetLength(current_ownship_state.m_y),
                                                                        m_ownship_kinematic_dtg_to_ptp);
}

void IMKinematicAchieve::CalculateTargetDtgToImPoints(
      const interval_management::open_source::AircraftState &current_lead_state) {
   if (m_target_aircraft_exists) {
      if (!IsTargetPassedLastWaypoint()) {
         m_target_distance_calculator.CalculateAlongPathDistanceFromPosition(Units::FeetLength(current_lead_state.m_x),
                                                                             Units::FeetLength(current_lead_state.m_y),
                                                                             m_target_kinematic_dtg_to_last_waypoint);
         if (!IsTargetPassedTrp()) {
            m_target_kinematic_dtg_to_trp = m_target_kinematic_dtg_to_last_waypoint -
                                            m_target_kinematic_traffic_reference_point_calcs.GetDistanceFromWaypoint();
         } else {
            m_target_kinematic_dtg_to_trp = Units::ZERO_LENGTH;
         }
      } else {
         m_target_kinematic_dtg_to_trp = Units::ZERO_LENGTH;
         m_target_kinematic_dtg_to_last_waypoint = Units::ZERO_LENGTH;
      }
   }
}

void IMKinematicAchieve::CalculateRFLegPhase(const std::vector<PrecalcWaypoint> &waypoints,
                                             const Units::Acceleration deceleration_rate_flight_path_angle,
                                             const VerticalPath &vertical_path,
                                             const std::vector<HorizontalPath> &horizontal_trajectory) {
   const bool has_rf_leg_on_route = std::any_of(
         waypoints.begin(), waypoints.end(),
         [](const PrecalcWaypoint &waypoint) { return waypoint.m_radius_rf_leg > Units::MetersLength(1.0); });

   if (has_rf_leg_on_route) {
      bool upper_limit_message_written = false;

      Units::HertzFrequency delta_speed_factor;
      Units::Length last_distance = Units::ZERO_LENGTH;
      Units::Length base_distance = Units::ZERO_LENGTH;
      Units::Speed base_ias = Units::ZERO_SPEED;
      Units::Speed restricted_speed = Units::ZERO_SPEED;

      RFLegPhase last_leg_phase = NON_RF_LEG;

      int last_index = 0;  // index for last distance in vertical path that has been used
      int htraj_index = 0;

      auto num_waypoints = waypoints.size();
      for (int i = 0; i < num_waypoints; ++i) {
         if (i == num_waypoints - 1) {
            m_speed_limiter.AddRfLegSpeedLimit(
                  Units::MetersLength(
                        vertical_path.along_path_distance_m[vertical_path.along_path_distance_m.size() - 1]),
                  restricted_speed);
            break;
         }

         if (waypoints[i].m_radius_rf_leg.value() > 1.0) {  // ON_RF_LEG
            // if preceding leg was NON_RF_LEG, end no-restriction segment
            if (last_leg_phase == NON_RF_LEG) {
               m_speed_limiter.AddRfLegSpeedLimit(last_distance, Units::zero());
               restricted_speed = waypoints[i].m_precalc_constraints.constraint_speedHi;
               m_speed_limiter.AddRfLegSpeedLimit(last_distance, restricted_speed);
               last_leg_phase = ON_RF_LEG;
               last_distance = waypoints[i].m_precalc_constraints.constraint_along_path_distance;
            } else if (last_leg_phase == PRE_RF_LEG) {
               // continue with previous deceleration line until it reaches constraint speed
               if (!upper_limit_message_written) {
                  upper_limit_message_written = true;
                  LOG4CPLUS_WARN(logger, "Pre-RF-Leg upper limit ias ("
                                               << Units::MetersPerSecondSpeed((last_distance - base_distance) *
                                                                              delta_speed_factor)
                                                        .value()
                                               << ") is less than constraint speed at precalc waypoint " << i
                                               << " at distance " << Units::MetersLength(last_distance).value() << ".");
               }
               Units::Speed delta_speed = waypoints[i].m_precalc_constraints.constraint_speedHi - base_ias;
               Units::Length delta_distance = Units::MetersLength(delta_speed / delta_speed_factor);
               if (delta_distance >
                   (waypoints[i].m_precalc_constraints.constraint_along_path_distance - base_distance)) {
                  last_leg_phase = PRE_RF_LEG;
               } else {
                  restricted_speed = waypoints[i + 1].m_precalc_constraints.constraint_speedHi;
                  m_speed_limiter.AddRfLegSpeedLimit(base_distance + delta_distance, restricted_speed);
                  last_leg_phase = ON_RF_LEG;
               }
               last_distance = waypoints[i].m_precalc_constraints.constraint_along_path_distance;
            } else {
               // last_leg_phase == ON_RF_LEG
               // start new deceleration line,
               // restriction is minimum of deceleration line and constraint speed
               if (waypoints[i].m_precalc_constraints.constraint_speedHi - restricted_speed <
                   Units::MetersPerSecondSpeed(0.0001)) {
                  // continue with same restricted speed
                  last_distance = waypoints[i].m_precalc_constraints.constraint_along_path_distance;
                  continue;
               }

               base_ias = restricted_speed;
               base_distance = last_distance;
               m_speed_limiter.AddRfLegSpeedLimit(last_distance, restricted_speed);

               // get ground speed to calculate deceleration factor
               bool dist_found = false;
               for (auto j = last_index; j < vertical_path.along_path_distance_m.size(); j++) {
                  if (last_distance < Units::MetersLength(vertical_path.along_path_distance_m[j])) {
                     last_index = j;
                     dist_found = true;
                     // Get groundspeed from horizontal trajectory
                     for (htraj_index = horizontal_trajectory.size() - 1; htraj_index >= 0; htraj_index--) {
                        if (horizontal_trajectory[htraj_index].m_path_length_cumulative_meters <=
                            vertical_path.along_path_distance_m[j]) {
                           Units::MetersPerSecondSpeed ground_speed =
                                 horizontal_trajectory[htraj_index].m_turn_info.groundspeed;
                           delta_speed_factor = deceleration_rate_flight_path_angle / ground_speed;
                           break;
                        }
                     }
                     // Need test for "not found in h_traj" error condition?  Should not happen.
                     break;
                  }
               }

               if (!dist_found) {  // before beginning of route - should not happen
                  LOG4CPLUS_WARN(logger,
                                 "While attempting to create upper speed restriction for RF_leg, distance is more than "
                                 "route length");
                  last_distance = waypoints[i].m_precalc_constraints.constraint_along_path_distance;
                  continue;
               }

               Units::Speed delta_speed = waypoints[i].m_precalc_constraints.constraint_speedHi - base_ias;
               Units::Length delta_distance = Units::MetersLength(delta_speed / delta_speed_factor);
               if (delta_distance > waypoints[i].m_precalc_constraints.constraint_along_path_distance - last_distance) {
                  last_leg_phase = PRE_RF_LEG;
               } else {
                  restricted_speed = waypoints[i].m_precalc_constraints.constraint_speedHi;
                  m_speed_limiter.AddRfLegSpeedLimit(last_distance + delta_distance, restricted_speed);
                  last_leg_phase = ON_RF_LEG;
               }
               last_distance = waypoints[i].m_precalc_constraints.constraint_along_path_distance;
            }
         } else {  // NON_RF_LEG
            if (last_distance == Units::ZERO_LENGTH) {
               m_speed_limiter.AddRfLegSpeedLimit(Units::zero(), Units::zero());
               last_distance = waypoints[i].m_precalc_constraints.constraint_along_path_distance;
            } else if (last_leg_phase == NON_RF_LEG) {
               last_distance = waypoints[i].m_precalc_constraints.constraint_along_path_distance;
            } else if (last_leg_phase == ON_RF_LEG) {
               m_speed_limiter.AddRfLegSpeedLimit(last_distance, restricted_speed);
               // start deceleration line..
               // restriction is minumum of deceleration line and nominal + HIGH_LIMIT
               Units::Length segment_end = waypoints[i].m_precalc_constraints.constraint_along_path_distance;
               for (int j = last_index; j < vertical_path.along_path_distance_m.size(); j++) {
                  if (last_distance < Units::MetersLength(vertical_path.along_path_distance_m[j])) {
                     last_index = j;
                     break;
                  }
               }
               base_ias = restricted_speed;
               base_distance = last_distance;

               // get ground speed to calculate deceleration factor
               bool dist_found = false;
               for (auto j = last_index; j < vertical_path.along_path_distance_m.size(); j++) {
                  if (last_distance < Units::MetersLength(vertical_path.along_path_distance_m[j])) {
                     last_index = j;
                     dist_found = true;
                     // Get groundspeed from horizontal trajectory
                     for (htraj_index = horizontal_trajectory.size() - 1; htraj_index >= 0; htraj_index--) {
                        if (horizontal_trajectory[htraj_index].m_path_length_cumulative_meters <=
                            vertical_path.along_path_distance_m[j]) {
                           Units::MetersPerSecondSpeed ground_speed =
                                 horizontal_trajectory[htraj_index - 1].m_turn_info.groundspeed;
                           delta_speed_factor = deceleration_rate_flight_path_angle / ground_speed;
                           break;
                        }
                     }
                     break;
                  }
               }

               if (!dist_found) {
                  LOG4CPLUS_WARN(logger,
                                 "While attempting to create upper speed restriction for RF_leg, distance is more than "
                                 "route length");
                  last_distance = waypoints[i].m_precalc_constraints.constraint_along_path_distance;
                  continue;
               }

               // find nominal speed and compare deceleration line
               for (; last_index < vertical_path.along_path_distance_m.size(); last_index++) {
                  if (Units::MetersLength(vertical_path.along_path_distance_m[last_index]) > segment_end) {
                     break;
                  }
                  Units::Length flight_distance =
                        Units::MetersLength(vertical_path.along_path_distance_m[last_index]) - base_distance;
                  restricted_speed = (flight_distance * delta_speed_factor) + base_ias;
                  if (Units::MetersPerSecondSpeed(vertical_path.cas_mps[last_index]) - restricted_speed >
                      Units::MetersPerSecondSpeed(2.5)) {
                     if (!upper_limit_message_written) {
                        upper_limit_message_written = true;
                        LOG4CPLUS_WARN(
                              logger,
                              "Pre-RF-Leg upper limit ias ("
                                    << Units::MetersPerSecondSpeed(restricted_speed).value()
                                    << ") is less than predicted airspeed ("
                                    << Units::MetersPerSecondSpeed(vertical_path.cas_mps[last_index]).value()
                                    << ") at distance "
                                    << Units::MetersLength(vertical_path.along_path_distance_m[last_index]).value()
                                    << ".");
                     }
                  }

                  if (restricted_speed >=
                      Units::MetersPerSecondSpeed(m_speed_limiter.HighLimit(vertical_path.cas_mps[last_index]))) {
                     // reached nominal upper limit
                     break;
                  }
               }

               if (Units::MetersLength(vertical_path.along_path_distance_m[last_index]) < segment_end) {
                  // deceleration line reached
                  // nominal + HIGH_LIMIT
                  m_speed_limiter.AddRfLegSpeedLimit(
                        Units::MetersLength(vertical_path.along_path_distance_m[last_index]),
                        Units::MetersPerSecondSpeed(restricted_speed));
                  m_speed_limiter.AddRfLegSpeedLimit(
                        Units::MetersLength(vertical_path.along_path_distance_m[last_index]), Units::zero());
                  last_leg_phase = NON_RF_LEG;
                  last_distance = segment_end;
                  restricted_speed = Units::ZERO_SPEED;
               } else {  // at end of segment, below nominal + HIGH_LIMIT
                  last_leg_phase = PRE_RF_LEG;
                  last_distance = segment_end;
               }
            } else if (last_leg_phase == PRE_RF_LEG) {
               // deceleration line is less than constraint speed
               Units::Length segment_end = waypoints[i].m_precalc_constraints.constraint_along_path_distance;
               for (; last_index < vertical_path.along_path_distance_m.size(); last_index++) {
                  if (Units::MetersLength(vertical_path.along_path_distance_m[last_index]) > segment_end) {
                     break;
                  }

                  Units::Length flight_distance =
                        Units::MetersLength(vertical_path.along_path_distance_m[last_index]) - base_distance;
                  restricted_speed = (flight_distance * delta_speed_factor) + base_ias;
                  if (restricted_speed < Units::MetersPerSecondSpeed(vertical_path.cas_mps[last_index])) {
                     if (!upper_limit_message_written) {
                        upper_limit_message_written = true;
                        LOG4CPLUS_WARN(
                              logger,
                              "Pre-RF-Leg upper limit ias ("
                                    << Units::MetersPerSecondSpeed(restricted_speed).value()
                                    << ") is less than predicted airspeed ("
                                    << Units::MetersPerSecondSpeed(vertical_path.cas_mps[last_index]).value()
                                    << ") at distance "
                                    << Units::MetersLength(vertical_path.along_path_distance_m[last_index]).value()
                                    << ".");
                     }
                  }

                  if (restricted_speed >=
                      Units::MetersPerSecondSpeed(m_speed_limiter.HighLimit(vertical_path.cas_mps[last_index]))) {
                     break;
                  }
               }

               if (Units::MetersLength(vertical_path.along_path_distance_m[last_index]) <= segment_end) {
                  m_speed_limiter.AddRfLegSpeedLimit(
                        Units::MetersLength(vertical_path.along_path_distance_m[last_index]), restricted_speed);
                  m_speed_limiter.AddRfLegSpeedLimit(
                        Units::MetersLength(vertical_path.along_path_distance_m[last_index]), Units::zero());
                  last_leg_phase = NON_RF_LEG;
                  restricted_speed = Units::ZERO_SPEED;
               }
               last_distance = segment_end;
            }
         }
      }
   }
}

void IMKinematicAchieve::ComputeFASTrajectories(
      const interval_management::open_source::AircraftState &owntruthstate,
      const interval_management::open_source::AircraftState &targettruthstate) {

   using namespace std;

   LOG4CPLUS_DEBUG(logger, "Calculating FAS trajectories on AC " << owntruthstate.GetId() << " at "
                                                                 << owntruthstate.GetTimeStamp().value());

   // find merge point
   Units::MetersLength x1, y1, x2, y2, x3, y3;  // point parameters
   string achieve_by_waypoint_name = m_im_clearance.GetAchieveByPoint();
   int achieve_by_waypoint_index = m_ownship_aircraft_intent.GetWaypointIndexByName(achieve_by_waypoint_name);

   size_t target_end_waypoint_ix = m_target_aircraft_intent.GetNumberOfWaypoints() - 1;
   Waypoint target_end_waypoint{m_target_aircraft_intent.GetWaypoint(target_end_waypoint_ix)};

   LOG4CPLUS_DEBUG(logger, "Achieve by = " << achieve_by_waypoint_name);

   Units::Angle merge_angle_mean;
   Units::MetersLength delta_y, delta_x;
   if (m_im_clearance.IsVectorAircraft()) {
      merge_angle_mean = IMUtils::CalculateTrackAngle(m_ownship_track_angle_history);
      x2 = m_ownship_aircraft_intent.GetWaypointX(achieve_by_waypoint_index);
      y2 = m_ownship_aircraft_intent.GetWaypointY(achieve_by_waypoint_index);
      x3 = Units::FeetLength(owntruthstate.m_x);
      y3 = Units::FeetLength(owntruthstate.m_y);
      delta_x = Units::FeetLength(targettruthstate.m_x) - m_target_aircraft_intent.GetWaypointX(target_end_waypoint_ix);
      delta_y = Units::FeetLength(targettruthstate.m_y) - m_target_aircraft_intent.GetWaypointY(target_end_waypoint_ix);
   } else {
      merge_angle_mean = IMUtils::CalculateTrackAngle(m_target_track_angle_history);
      x2 = m_target_aircraft_intent.GetWaypointX(target_end_waypoint_ix);
      y2 = m_target_aircraft_intent.GetWaypointY(target_end_waypoint_ix);
      x3 = Units::FeetLength(targettruthstate.m_x);
      y3 = Units::FeetLength(targettruthstate.m_y);
      delta_x =
            Units::FeetLength(owntruthstate.m_x) - m_ownship_aircraft_intent.GetWaypointX(achieve_by_waypoint_index);
      delta_y =
            Units::FeetLength(owntruthstate.m_y) - m_ownship_aircraft_intent.GetWaypointY(achieve_by_waypoint_index);
   }
   Units::DegreesAngle reverse_final_approach_angle = Units::arctan2(delta_y.value(), delta_x.value());
   LOG4CPLUS_DEBUG(logger, reverse_final_approach_angle);
   x1 = x2 + Units::NauticalMilesLength(50) * Units::cos(reverse_final_approach_angle);
   y1 = y2 + Units::NauticalMilesLength(50) * Units::sin(reverse_final_approach_angle);

   // randomize merge angle around the mean
   Units::Angle merge_angle_std = m_im_clearance.GetFinalApproachSpacingMergeAngleStd();
   Units::Angle merge_angle = Scenario::m_rand.GaussianSample(merge_angle_mean, merge_angle_std);
   Units::DegreesAngle final_approach_angle = reverse_final_approach_angle + Units::PI_RADIANS_ANGLE;
   LOG4CPLUS_DEBUG(logger, "Average track angle = " << Units::DegreesAngle(merge_angle_mean)
                                                    << ", randomized merge angle = " << Units::DegreesAngle(merge_angle)
                                                    << ", final_approach_angle = " << final_approach_angle);

   Units::MetersLength xMerge, yMerge;
   IMUtils::CalculateMergePoint(x1, y1, x2, y2, x3, y3, xMerge, yMerge, merge_angle);

   LOG4CPLUS_DEBUG(logger, "Merge point is (" << xMerge << "," << yMerge << ")");

   // construct intents
   // final AC:  current_pos last_wp
   // vector AC:  current_pos merge_point last_wp
   AircraftIntent own_intent(m_ownship_aircraft_intent), target_intent(m_target_aircraft_intent);
   own_intent.ClearWaypoints();
   target_intent.ClearWaypoints();
   own_intent.SetPlannedCruiseAltitude(Units::FeetLength(owntruthstate.m_z));
   target_intent.SetPlannedCruiseAltitude(Units::FeetLength(targettruthstate.m_z));

   Waypoint ptp_waypoint{m_ownship_aircraft_intent.GetWaypoint(achieve_by_waypoint_index)};
   own_intent.InsertWaypointAtIndex(ptp_waypoint, 0);
   target_intent.InsertWaypointAtIndex(target_end_waypoint, 0);

   pair<Units::Length, Units::Length> final_wpt_coords(x2, y2);
   pair<Units::Length, Units::Length> merge_coords(xMerge, yMerge);
   pair<Units::Length, Units::Length> current_coords(x3, y3);

   Units::MetersLength merge_to_lastwpt_distance =
         CoreUtils::CalculateEuclideanDistance(merge_coords, final_wpt_coords);
   Units::MetersLength current_to_lastwpt_distance =
         CoreUtils::CalculateEuclideanDistance(current_coords, final_wpt_coords);
   Units::MetersLength current_to_merge_distance = CoreUtils::CalculateEuclideanDistance(current_coords, merge_coords);

   LOG4CPLUS_DEBUG(logger, "current = (" << Units::MetersLength(current_coords.first) << ","
                                         << Units::MetersLength(current_coords.second) << ")");
   LOG4CPLUS_DEBUG(logger, "merge = (" << Units::MetersLength(merge_coords.first) << ","
                                       << Units::MetersLength(merge_coords.second) << ")");
   LOG4CPLUS_DEBUG(logger, "last wpt = (" << Units::MetersLength(final_wpt_coords.first) << ","
                                          << Units::MetersLength(final_wpt_coords.second) << ")");
   LOG4CPLUS_DEBUG(logger, "Current to merge = " << current_to_merge_distance);
   LOG4CPLUS_DEBUG(logger, "Current to lastwpt = " << current_to_lastwpt_distance);
   LOG4CPLUS_DEBUG(logger, "Merge to lastwpt = " << merge_to_lastwpt_distance);

   // convert states to waypoints, using ownship's sensed winds
   own_intent.InsertWaypointAtIndex(MakeWaypointFromState(owntruthstate, owntruthstate.GetSensedWindEastComponent(),
                                                          owntruthstate.GetSensedWindNorthComponent()),
                                    0);

   // Get predicted winds at target aircraft's current altitude & create waypoint
   Units::MetersPerSecondSpeed target_wind_x, target_wind_y;
   Units::HertzFrequency dVwx_dh, dVwy_dh;
   m_weather_prediction.getAtmosphere()->CalculateWindGradientAtAltitude(
         Units::FeetLength(targettruthstate.m_z), m_weather_prediction.east_west, target_wind_x, dVwx_dh);
   m_weather_prediction.getAtmosphere()->CalculateWindGradientAtAltitude(
         Units::FeetLength(targettruthstate.m_z), m_weather_prediction.north_south, target_wind_y, dVwy_dh);
   target_intent.InsertWaypointAtIndex(MakeWaypointFromState(targettruthstate, target_wind_x, target_wind_y), 0);

   const Units::DegreesAngle tolerance(10.0);
   const Units::NauticalMilesLength too_close(.85);  // see AAES-939, AAES-1061
   if (Units::abs(final_approach_angle - merge_angle) < tolerance) {
      LOG4CPLUS_WARN(logger, "Merge point discarded because merge angle is less than 10 deg: "
                                   << Units::DegreesAngle(Units::abs(final_approach_angle - merge_angle)));
   } else if (current_to_lastwpt_distance < merge_to_lastwpt_distance) {
      LOG4CPLUS_WARN(logger, "Merge point discarded because current position is closer to end.");
   } else if (current_to_lastwpt_distance < current_to_merge_distance) {
      LOG4CPLUS_WARN(logger, "Merge point discarded because it is past end.");
   } else if (Units::abs(current_to_merge_distance) < too_close) {
      LOG4CPLUS_WARN(logger, "Merge point discarded because it is near current location.");
   } else if (Units::abs(merge_to_lastwpt_distance) < too_close) {
      LOG4CPLUS_WARN(logger, "Merge point discarded because it is near PTP.");
   } else {
      if (m_im_clearance.IsVectorAircraft()) {
         own_intent.InsertPairAtIndex("merge", xMerge, yMerge, 1);
      } else {
         target_intent.InsertPairAtIndex("merge", xMerge, yMerge, 1);
      }
   }
   LOG4CPLUS_TRACE(logger, "Ownship AircraftIntent:" << endl << own_intent);
   LOG4CPLUS_TRACE(logger, "Target AircraftIntent:" << endl << target_intent);

   m_ownship_aircraft_intent = own_intent;
   m_target_aircraft_intent = target_intent;

   m_compute_ownship_kinematic_trajectory = true;
   m_compute_target_kinematic_trajectory = true;
   m_fas_intent_valid = true;
}

Waypoint IMKinematicAchieve::MakeWaypointFromState(const interval_management::open_source::AircraftState aircraft_state,
                                                   Units::Speed wind_x, Units::Speed wind_y) const {
   // calculate fields needed for a Waypoint object

   // lat, lon
   EarthModel::LocalPositionEnu local_pos;
   local_pos.x = Units::FeetLength(aircraft_state.m_x);
   local_pos.y = Units::FeetLength(aircraft_state.m_y);
   local_pos.z = Units::FeetLength(aircraft_state.m_z);

   EarthModel::GeodeticPosition geo_pos;
   m_tangent_plane_sequence->convertLocalToGeodetic(local_pos, geo_pos);

   // ias
   Units::Speed groundspeed_x = Units::FeetPerSecondSpeed(aircraft_state.m_xd) - wind_x;
   Units::Speed groundspeed_y = Units::FeetPerSecondSpeed(aircraft_state.m_yd) - wind_y;
   Units::KnotsSpeed tas_estimate = sqrt(groundspeed_x * groundspeed_x + groundspeed_y * groundspeed_y);
   Units::KnotsSpeed cas_estimate;
   cas_estimate = m_weather_prediction.TAS2CAS(tas_estimate, local_pos.z);

   return Waypoint("state", geo_pos.latitude, geo_pos.longitude, local_pos.z, local_pos.z, cas_estimate, local_pos.z,
                   cas_estimate);
}

const Units::SignedAngle IMKinematicAchieve::CalculateTargetTrackAngle(
      const vector<interval_management::open_source::AircraftState> &target_adsb_history) {
   vector<interval_management::open_source::AircraftState>::const_reverse_iterator critr = target_adsb_history.rbegin();
   double current_time = (*critr).GetTimeStamp().value();
   double lower = current_time - (Units::SecondsTime(TRACK_ANGLE_TAU)).value();

   // Use RadiansAngle here so that circular wrapping does not happen while summing below
   Units::RadiansAngle sum = Units::SignedRadiansAngle(atan2((*critr).m_yd, (*critr).m_xd));
   bool too_close_to_boundary = abs(sum) > Units::SignedDegreesAngle(170.0) || abs(sum) < Units::SignedDegreesAngle(10);
   if (too_close_to_boundary) {
      sum = Units::UnsignedRadiansAngle(sum);
   }

   unsigned long cnt = 1;
   for (++critr; critr != target_adsb_history.rend(); ++critr) {
      if ((*critr).GetTimeStamp().value() < lower) {
         break;
      } else {
         cnt++;
         if (too_close_to_boundary) {
            // use interval0,360)
            sum += Units::UnsignedRadiansAngle(atan2((*critr).m_yd, (*critr).m_xd));
         } else {
            sum += Units::SignedRadiansAngle(atan2((*critr).m_yd, (*critr).m_xd));
         }
      }
   }

   return Units::SignedRadiansAngle(sum / cnt);
}

void IMKinematicAchieve::SetTrafficReferencePointConstraints(
      const interval_management::open_source::AircraftState &owntruthstate,
      const interval_management::open_source::AircraftState &targetsyncstate) {

   // adjust TRP speed and altitude constraints

   LOG4CPLUS_DEBUG(logger, "Old TRP constraints:  " << m_traffic_reference_point);
   Units::KnotsSpeed target_trp_speed;
   Units::FeetLength target_trp_altitude;

   // Speed case 1:  Is ownship using an arrival procedure?   Probably.
   // Speed case 1a:  Ownship speed constant from now to ABP?  Use target's current speed.
   // Speed case 1b:  Use ownship's CAS at ABP.
   // Speed case 2:  No arrival procedure
   // Speed case 2a:  Last upstream limit
   // Speed case 2b:  Current target CAS (if no 2a)

   // Altitude case 1:  Probably not already on approach, so not implemented.
   // Altitude case 2:  Is ownship using an arrival procedure?  Probably.
   // Altitude case 2a:  Ownship's altitude constant from now to ABP?  Use target's current altitude.
   // Altitude case 2b:  Use ownship's predicted altitude at ABP.
   // Altitude case 3a:  Last upstream altitude limit
   // Altitude case 3b:  Current altitude (if no 3a)

   {
      // Speed case 1 / Altitude case 2
      // We need ownship's predicted speed and altitude at ABP
      Units::MetersLength abp_dtg = m_ownship_kinematic_achieve_by_calcs.GetDistanceFromWaypoint();
      int ownship_achieve_by_index = CoreUtils::FindNearestIndex(
            abp_dtg.value(), m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances());
      Units::Speed ownship_cas_at_abp;
      Units::FeetLength ownship_altitude_at_abp;
      if (ownship_achieve_by_index + 1 >= m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes().size()) {
         // end
         ownship_cas_at_abp =
               Units::MetersPerSecondSpeed(m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities().back());
         ownship_altitude_at_abp =
               Units::MetersLength(m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes().back());
      } else if (ownship_achieve_by_index == 0) {
         // beginning
         ownship_cas_at_abp =
               Units::MetersPerSecondSpeed(m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(0));
         ownship_altitude_at_abp =
               Units::MetersLength(m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudeByIndex(0));
      } else {
         // normal case, middle somewhere, so interpolate
         ownship_cas_at_abp = Units::MetersPerSecondSpeed(
               CoreUtils::LinearlyInterpolate(ownship_achieve_by_index, abp_dtg.value(),
                                              m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances(),
                                              m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities()));
         ownship_altitude_at_abp = Units::MetersLength(
               CoreUtils::LinearlyInterpolate(ownship_achieve_by_index, abp_dtg.value(),
                                              m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances(),
                                              m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes()));
      }

      // calculate ownship CAS
      Units::MetersPerSecondSpeed ownship_tas_x =
            owntruthstate.GetSpeedXd() - owntruthstate.GetSensedWindEastComponent();
      Units::MetersPerSecondSpeed ownship_tas_y =
            owntruthstate.GetSpeedYd() - owntruthstate.GetSensedWindNorthComponent();
      Units::MetersPerSecondSpeed ownship_tas(hypot(ownship_tas_x.value(), ownship_tas_y.value()));
      Units::KnotsSpeed ownship_current_cas;
      ownship_current_cas = m_weather_prediction.TAS2CAS(ownship_tas, owntruthstate.GetPositionZ());

      if (abs(ownship_current_cas - ownship_cas_at_abp) < Units::KnotsSpeed(5)) {
         // assume CAS is "constant" in the intervening range:  Speed case 1a
         Units::Speed estimated_wind_speed_x, estimated_wind_speed_y;
         Units::Frequency estimated_wind_gradient_x, estimated_wind_gradient_y;
         m_weather_prediction.GetForecastAtmosphere()->CalculateWindGradientAtAltitude(
               targetsyncstate.GetPositionZ(), m_weather_prediction.east_west, estimated_wind_speed_x,
               estimated_wind_gradient_x);
         m_weather_prediction.GetForecastAtmosphere()->CalculateWindGradientAtAltitude(
               targetsyncstate.GetPositionZ(), m_weather_prediction.north_south, estimated_wind_speed_y,
               estimated_wind_gradient_y);

         interval_management::open_source::AircraftState target_copy_state;
         target_copy_state.Create(
               targetsyncstate.GetId(), targetsyncstate.GetTimeStamp(), targetsyncstate.GetPosition(),
               targetsyncstate.GetSpeedXd(), targetsyncstate.GetSpeedYd(), targetsyncstate.GetSpeedZd(),
               targetsyncstate.GetGamma(), Units::MetersPerSecondSpeed(estimated_wind_speed_x),
               Units::MetersPerSecondSpeed(estimated_wind_speed_y), targetsyncstate.GetSensedWindParallelComponent(),
               targetsyncstate.GetSensedWindPerpendicularComponent(), targetsyncstate.GetSensedTemperature(),
               targetsyncstate.GetPsi());
         Units::Speed target_tas(target_copy_state.GetTrueAirspeed());
         Units::Speed target_cas = m_weather_prediction.TAS2CAS(target_tas, targetsyncstate.GetPositionZ());
         target_trp_speed = target_cas;
      } else {
         // Speed case 1b
         target_trp_speed = ownship_cas_at_abp;
      }

      const Units::FeetLength constant_altitude_tolerance(500.0);
      if (Units::abs(ownship_altitude_at_abp - owntruthstate.GetPositionZ()) < constant_altitude_tolerance) {
         // assume altitude is "constant" in the intervening range:  Altitude case 2a
         target_trp_altitude = targetsyncstate.GetPositionZ();
      } else {
         // Altitude case 2b
         target_trp_altitude = ownship_altitude_at_abp;
      }
   }  // end Speed case 1 / Altitude case 2

   if (m_traffic_reference_point.GetNominalIas() == Units::zero()) {
      m_traffic_reference_point.SetSpeedConstraintHigh(target_trp_speed);
      m_traffic_reference_point.SetNominalIas(target_trp_speed);
   }
   if (m_traffic_reference_point.GetAltitude() == Units::zero()) {
      m_traffic_reference_point.SetAltitudeConstraintHigh(target_trp_altitude);
      m_traffic_reference_point.SetAltitude(target_trp_altitude);
   }
   m_traffic_reference_point.SetMach(0);

   LOG4CPLUS_DEBUG(logger, "New TRP constraints:  " << m_traffic_reference_point);
   m_target_aircraft_intent.UpdateWaypoint(m_traffic_reference_point);
}
