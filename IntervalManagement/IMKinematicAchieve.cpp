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
#include "math/CustomMath.h"
#include "public/AircraftCalculations.h"
#include "public/CoreUtils.h"
#include "public/Environment.h"
#include "public/StandardAtmosphere.h"
#include "imalgs/IMKinematicAchieve.h"
#include "imalgs/IMScenario.h"

log4cplus::Logger IMKinematicAchieve::logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("IMKinematicAchieve"));
const int IMKinematicAchieve::MINIMUM_FAS_TRACK_COUNT(5);
const Units::FeetLength IMKinematicAchieve::TARGET_ALTITUDE_TOLERANCE(3000);
const Units::SecondsTime IMKinematicAchieve::TRACK_ANGLE_TAU(3.0);

IMKinematicAchieve::IMKinematicAchieve()
      : m_blend_wind(IMScenario::BLEND_WIND_DEFAULT) {
   IterClearIMKinAch();
}

IMKinematicAchieve::~IMKinematicAchieve() = default;


void IMKinematicAchieve::IterationReset() {
   IMAchieve::IterationReset();
   IterClearIMKinAch();
}

void IMKinematicAchieve::IterClearIMKinAch() {
   /**
    * Do not clear loaded parameters here. Only field members that are specific to this class and need to be reset.
    */

   m_new_trajectory_prediction_available = false;
   m_compute_ownship_kinematic_trajectory = true;
   m_compute_target_kinematic_trajectory = true;
   m_ownship_kinematic_trajectory_dumped = false;
   m_target_kinematic_trajectory_dumped = false;

   m_target_aircraft_exists = false;
   m_target_history_exists = false;

   KinematicTrajectoryPredictor dummytraj;

   m_ownship_kinematic_trajectory_predictor = dummytraj;
   m_target_kinematic_trajectory_predictor = dummytraj;

   m_ownship_kinematic_achieve_by_calcs = AchievePointCalcs();
   m_target_kinematic_traffic_reference_point_calcs = AchievePointCalcs();

   m_fas_intent_valid = false;
   m_is_target_aligned = false;
}

// Initialize replaces HorizontalPath with a HorizontalPath the terminates at the planned_termination_point
void IMKinematicAchieve::Initialize(const KineticTrajectoryPredictor &ownship_kinetic_trajectory_predictor,
                                    const KineticTrajectoryPredictor &target_kinetic_trajectory_predictor,
                                    std::shared_ptr<TangentPlaneSequence> tangent_plane_sequence,
                                    AircraftIntent &target_aircraft_intent,
                                    const IMClearance &im_clearance,
                                    const std::string &achieve_by_point,
                                    WeatherPrediction &weather_prediction) {
   IMAchieve::Initialize(ownship_kinetic_trajectory_predictor, target_kinetic_trajectory_predictor,
                         tangent_plane_sequence, target_aircraft_intent,
                         im_clearance, achieve_by_point, weather_prediction);

   m_ownship_track_angle_history.clear();
   m_target_track_angle_history.clear();
   m_fas_intent_valid = false;
   m_is_target_aligned = false;

   // Setup the IM Achieve algorithm to only know about waypoints down to the PTP. This
   // is accomplished by updating the aircraft intent. Note that only the IM intent should
   // be modified. The target intent is supposed to be provided only down to the PTP anyway
   // (and we can't assume in code that we know where the PTP is for non-coincident routes).
   m_ownship_aircraft_intent = ownship_kinetic_trajectory_predictor.GetAircraftIntent();
   TrimAircraftIntentAfterWaypoint(m_ownship_aircraft_intent, m_im_clearance.GetPlannedTerminationPoint());

   // Initialize these objects with kinetic horizontal path. The horizontal paths will be updated to kinematic before
   // the first call.
   if (im_clearance.GetClearanceType() == IMClearance::MAINTAIN
       || im_clearance.GetClearanceType() == IMClearance::CAPTURE) {
      m_im_ownship_distance_calculator = AlongPathDistanceCalculator::CreateForCaptureClearance(
            ownship_kinetic_trajectory_predictor.GetHorizontalPath());
   } else {
      m_im_ownship_distance_calculator = AlongPathDistanceCalculator(
            ownship_kinetic_trajectory_predictor.GetHorizontalPath(),
            TrajectoryIndexProgressionDirection::UNDEFINED);
   }
   m_ownship_distance_calculator = AlongPathDistanceCalculator(
         ownship_kinetic_trajectory_predictor.GetHorizontalPath(),
         TrajectoryIndexProgressionDirection::DECREMENTING);

   m_target_distance_calculator = AlongPathDistanceCalculator(target_kinetic_trajectory_predictor.GetHorizontalPath(),
                                                              TrajectoryIndexProgressionDirection::DECREMENTING);
   m_ownship_kinetic_distance_calculator = AlongPathDistanceCalculator(
         ownship_kinetic_trajectory_predictor.GetHorizontalPath(),
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
   int index = 0;
   const int maxIndex = aircraft_intent.GetNumberOfWaypoints();

   for (index = 0; index < maxIndex; index++) {
      if (aircraft_intent.GetWaypointName(index) == waypoint_name) {
         aircraft_intent.SetNumberOfWaypoints(index + 1);
         break;
      }
   }

   if (index >= maxIndex) {
      // NOTE: This is here for completeness. IMClearance::validate() should ensure this cannot be occur.
      std::string msg = "ERROR: Could not find plannedTerminationPoint (" + waypoint_name + ") in list of Waypoints";
      LOG4CPLUS_ERROR(IMKinematicAchieve::logger, msg);
      throw std::logic_error("Could not find plannedTerminationPoint in list of Waypoints");
   }
}


bool IMKinematicAchieve::load(DecodedStream *input) {
   set_stream(input);
   m_assigned_spacing_goal_from_input_file = 0;

   Units::HertzFrequency achievecontrolgainload(m_achieve_control_gain);
   Units::HertzFrequency maintaincontrolgainload(m_maintain_control_gain);
   Units::NauticalMilesLength distquantize1load(m_middle_to_final_quantize_transition_distance);
   Units::NauticalMilesLength distquantize2load(m_first_to_middle_quantize_transition_distance);
   Units::KnotsSpeed speedquantize1load(m_speed_quantize_final_phase);
   Units::KnotsSpeed speedquantize2load(m_speed_quantize_middle_phase);
   Units::KnotsSpeed speedquantize3load(m_speed_quantize_first_phase);
   Units::SecondsTime timethresholdload(m_time_threshold);
   Units::NauticalMilesLength errordistanceload(m_error_distance);
   Units::SecondsPerNauticalMileInvertedSpeed slopeload(m_slope);

   LoaderDeprecatedMetaInfo spacingDeprecatedInfo;
   spacingDeprecatedInfo.isDeprecated = true;
   spacingDeprecatedInfo.supersededByTagName = "IMClearance.Assigned_Spacing_Goal";

   register_var("spacing", &m_assigned_spacing_goal_from_input_file, spacingDeprecatedInfo);
   register_var("limit", &m_limit_flag);

   register_var("k_achieve", &achievecontrolgainload);
   register_var("k_maintain", &maintaincontrolgainload);

   // speed quantization values (have defaults if not loaded)
   register_var("dist_quantize_1", &distquantize1load);
   register_var("dist_quantize_2", &distquantize2load);
   register_var("quant1", &speedquantize1load);
   register_var("quant2", &speedquantize2load);
   register_var("quant3", &speedquantize3load);

   // error threshold values (have defaults if not loaded)
   register_var("error_distance", &errordistanceload);
   register_var("threshold1", &timethresholdload);
   register_var("threshold2", &slopeload);

   register_var("use_error_threshold", &m_threshold_flag);
   register_var("use_speed_quantize", &m_quantize_flag);

   double max_speed_deviation_percentage(DEFAULT_SPEED_DEVIATION_PERCENTAGE);
   register_var("max_speed_deviation_percentage", &max_speed_deviation_percentage);

   m_loaded = complete();

   if (achievecontrolgainload == Units::ZERO_FREQUENCY) {
      std::string msg = "0 achieve control gain encountered-check scenario";
      LOG4CPLUS_FATAL(logger, msg);
      throw std::logic_error(msg);
   }

   if (maintaincontrolgainload == Units::ZERO_FREQUENCY) {
      std::string msg = "load: 0 maintain control gain encountered-check scenario";
      LOG4CPLUS_FATAL(logger, msg);
      throw std::logic_error(msg);
   }

   if (m_loaded) {
      m_achieve_control_gain = achievecontrolgainload;
      m_maintain_control_gain = maintaincontrolgainload;
      m_middle_to_final_quantize_transition_distance = distquantize1load;
      m_first_to_middle_quantize_transition_distance = distquantize2load;
      m_speed_quantize_final_phase = speedquantize1load;
      m_speed_quantize_middle_phase = speedquantize2load;
      m_speed_quantize_first_phase = speedquantize3load;
      m_time_threshold = timethresholdload;
      m_error_distance = errordistanceload;
      m_slope = slopeload;
      SetMaxSpeedDeviationPercentage(max_speed_deviation_percentage);
   }

   return m_loaded;
}

Guidance IMKinematicAchieve::Update(const Guidance &prevguidance,
                                    const DynamicsState &dynamicsstate,
                                    const AircraftState &owntruthstate,
                                    const AircraftState &targetsyncstate,
                                    const vector<AircraftState> &targethistory) {
   Guidance guidance_out =
         IMAchieve::Update(prevguidance, dynamicsstate, owntruthstate, targetsyncstate, targethistory);

   m_target_aircraft_exists = targetsyncstate.m_id != IMUtils::UNINITIALIZED_AIRCRAFT_ID;
   m_target_history_exists = !targethistory.empty();

   m_ownship_track_angle_history.push_back(
         Units::SignedRadiansAngle(atan2(
               owntruthstate.m_yd, owntruthstate.m_xd)));
   if (m_ownship_track_angle_history.size() > MINIMUM_FAS_TRACK_COUNT) {
      m_ownship_track_angle_history.pop_back();
   }
   if (m_target_aircraft_exists) {
      m_target_track_angle_history.push_back(
            Units::SignedRadiansAngle(atan2(
                  targetsyncstate.m_yd, targetsyncstate.m_xd)));
      if (m_target_track_angle_history.size() > MINIMUM_FAS_TRACK_COUNT) {
         m_target_track_angle_history.pop_back();
      }

   }
   // This is output only -- no algorithms
   UpdatePositionMetrics(owntruthstate, targetsyncstate);

   CheckPredictionAccuracy(owntruthstate, targetsyncstate);

   // special handling for Final Approach Spacing
   if (m_im_clearance.GetClearanceType() == IMClearance::ClearanceType::FAS) {
      // Do we have enough info to start FAS?
      size_t ownship_track_count = m_ownship_track_angle_history.size();
      size_t target_track_count = m_target_track_angle_history.size();
      if (ownship_track_count < MINIMUM_FAS_TRACK_COUNT ||
          target_track_count < MINIMUM_FAS_TRACK_COUNT) {
         guidance_out = prevguidance;
         guidance_out.SetValid(false);

         // trigger computation as soon as we do have enough
         m_compute_ownship_kinematic_trajectory = true;
         m_compute_target_kinematic_trajectory = true;
         return guidance_out;
      }

      // We know neither is <5, so if one of them is equal, it's time
      // for our first FAS calculation.  Also get recompute cases.
      if (!m_fas_intent_valid) {
         ComputeFASTrajectories(owntruthstate, targetsyncstate);
      }
   }

   HandleTrajectoryPrediction(owntruthstate, targetsyncstate, targethistory);

   if ((m_target_aircraft_exists || m_target_history_exists) &&
       m_received_one_valid_target_state && !m_compute_target_kinematic_trajectory) {
      CalculateOwnshipDtgToPlannedTerminationPoint(owntruthstate);
      CalculateOwnshipDtgToAchieveByPoint();
      CalculateTargetDtgToImPoints(targetsyncstate);

      guidance_out.SetValid(true);
   } else {
      guidance_out.SetValid(false);
   }

   return guidance_out;
}

void IMKinematicAchieve::HandleTrajectoryPrediction(const AircraftState &owntruthstate,
                                                    const AircraftState &targetsyncstate,
                                                    const vector<AircraftState> &target_adsb_history) {
   m_new_trajectory_prediction_available =
         m_compute_ownship_kinematic_trajectory || m_compute_target_kinematic_trajectory;

   if (m_compute_ownship_kinematic_trajectory) {
      LOG4CPLUS_DEBUG(logger, "compute ownship kinematic trajectory: " << owntruthstate.m_id);
      Units::RadiansAngle dummy_course = Units::RadiansAngle(0);
      std::vector<HorizontalPath>::size_type dummy_index = 0;

      Units::MetersLength ownship_distance_to_go = Units::MetersLength(Units::infinity());
      const std::vector<HorizontalPath> ownship_horizontal_path = m_ownship_kinematic_trajectory_predictor.GetHorizontalPath();
      m_ownship_kinematic_trajectory_predictor.SetMembers(*m_ownship_kinetic_trajectory_predictor);
      m_ownship_kinematic_trajectory_predictor.CalculateWaypoints(m_ownship_aircraft_intent);
      // If there is a previous horizontal path, use it to find distance to go
      if (!ownship_horizontal_path.empty()) {
         if (!AircraftCalculations::CalculateDistanceAlongPathFromPosition(Units::FeetLength(owntruthstate.m_x),
                                                                           Units::FeetLength(owntruthstate.m_y),
                                                                           ownship_horizontal_path, 0,
                                                                           ownship_distance_to_go,
                                                                           dummy_course, dummy_index)) {
            ownship_distance_to_go = Units::MetersLength(Units::infinity());
         }
      }
      m_ownship_kinematic_trajectory_predictor.BuildTrajectoryPrediction(m_weather_prediction,
                                                                         Units::FeetLength(owntruthstate.m_z),
                                                                         ownship_distance_to_go);
      m_has_rf_leg = CalculateRFLegPhase();
      m_ownship_kinematic_achieve_by_calcs = AchievePointCalcs(m_achieve_by_point,
                                                               m_ownship_aircraft_intent,
                                                               m_ownship_kinematic_trajectory_predictor.GetKinematicDescent4dPredictor()->GetVerticalPath(),
                                                               m_ownship_kinematic_trajectory_predictor.GetHorizontalPath());

      m_ownship_distance_calculator.UpdateHorizontalTrajectory(
            m_ownship_kinematic_trajectory_predictor.GetHorizontalPath());
      m_im_ownship_distance_calculator.UpdateHorizontalTrajectory(
            m_ownship_kinematic_trajectory_predictor.GetHorizontalPath());

      if (!m_ownship_kinematic_trajectory_dumped) {
         InternalObserver::getInstance()->dumpOwnKinTraj(owntruthstate.m_id,
                                                         m_ownship_kinematic_trajectory_predictor.GetKinematicDescent4dPredictor()->GetVerticalPath());
         m_ownship_kinematic_trajectory_dumped = true;
      }

      m_compute_ownship_kinematic_trajectory = false;
   }

   if (m_target_aircraft_exists && m_target_history_exists && m_compute_target_kinematic_trajectory) {
      LOG4CPLUS_DEBUG(logger,
                      "ac " << owntruthstate.m_id << " compute target kinematic trajectory: " << targetsyncstate.m_id);
      const Units::Speed targetgroundspeed = Units::FeetPerSecondSpeed(
            sqrt(pow(targetsyncstate.m_xd, 2) + pow(targetsyncstate.m_yd, 2)));
      const Units::Angle targetgroundtrack = Units::RadiansAngle(atan2(targetsyncstate.m_yd, targetsyncstate.m_xd));

      // Get Winds at Target Aircraft's current altitude
      Units::MetersPerSecondSpeed Vwx, Vwy;
      Units::HertzFrequency dVwx_dh, dVwy_dh;
      m_weather_prediction.getAtmosphere()->CalculateWindGradientAtAltitude(Units::FeetLength(targetsyncstate.m_z),
                                                                            m_weather_prediction.east_west, Vwx,
                                                                            dVwx_dh);
      m_weather_prediction.getAtmosphere()->CalculateWindGradientAtAltitude(Units::FeetLength(targetsyncstate.m_z),
                                                                            m_weather_prediction.north_south, Vwy,
                                                                            dVwy_dh);

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
      double estimated_cruise_mach = Units::MetersPerSecondSpeed(targetvtas).value() / sqrt(GAMMA * R.value() *
                                                                                            m_weather_prediction.getAtmosphere()->GetTemperature(
                                                                                                  Units::FeetLength(
                                                                                                        targetsyncstate.m_z)).value());
      const double min_mach_descent = 0.7;
      const double max_mach_descent = 0.86;
      estimated_cruise_mach = std::min(estimated_cruise_mach, max_mach_descent);
      estimated_cruise_mach = std::max(estimated_cruise_mach, min_mach_descent);
      estimated_cruise_mach = quantize(estimated_cruise_mach, 0.02);  // we don't want this value being a float

      const Units::KnotsSpeed min_allowable_descent_cas(270.0);
      const Units::Speed target_descent_cas = max(IMUtils::FindLargestSpeedConstraint(m_target_aircraft_intent),
                                                  min_allowable_descent_cas);
      Units::Length target_transition_altitude =
            m_weather_prediction.getAtmosphere()->GetMachIASTransition(target_descent_cas, estimated_cruise_mach);
      if (estimated_cruise_altitude < target_transition_altitude) {
         // there isn't enough information to guess at a cruise mach condition. Set mach to
         // zero so that no mach transition is predicted.I vaguely remember talking
         estimated_cruise_mach = 0;
         target_transition_altitude = Units::Length(Units::negInfinity());
      }

      m_target_kinematic_trajectory_predictor.SetMembers(*m_ownship_kinetic_trajectory_predictor);

      if (m_im_clearance.GetClearanceType() == IMClearance::ClearanceType::FAS) {
         m_target_kinematic_trajectory_predictor.GetKinematicDescent4dPredictor()->SetMembers(
               estimated_cruise_mach, target_descent_cas, estimated_cruise_altitude,
               m_weather_prediction.getAtmosphere()->GetMachIASTransition(target_descent_cas, estimated_cruise_mach));
      } else {
         m_target_kinematic_trajectory_predictor.GetKinematicDescent4dPredictor()->SetMembers(
               estimated_cruise_mach, target_descent_cas, max(target_transition_altitude, estimated_cruise_altitude),
               m_weather_prediction.getAtmosphere()->GetMachIASTransition(target_descent_cas, estimated_cruise_mach));
      }

      Units::RadiansAngle dummy_course = Units::RadiansAngle(0);
      std::vector<HorizontalPath>::size_type dummy_index = 0;
      std::vector<HorizontalPath> target_horizontal_path = m_target_kinematic_trajectory_predictor.GetHorizontalPath();
      Units::MetersLength target_distance_to_go = Units::MetersLength(Units::infinity());
      Units::MetersLength target_start_altitude_msl = Units::FeetLength(targetsyncstate.m_z);

      if (!target_horizontal_path.empty()) {
         if (!AircraftCalculations::CalculateDistanceAlongPathFromPosition(Units::FeetLength(targetsyncstate.m_x),
                                                                           Units::FeetLength(targetsyncstate.m_y),
                                                                           target_horizontal_path, 0,
                                                                           target_distance_to_go,
                                                                           dummy_course, dummy_index)) {
            target_distance_to_go = Units::MetersLength(Units::infinity());
         }
      }

      m_target_kinematic_trajectory_predictor.CalculateWaypoints(m_target_aircraft_intent);
      m_target_kinematic_trajectory_predictor.BuildTrajectoryPrediction(m_weather_prediction, target_start_altitude_msl,
                                                                        target_distance_to_go);
      m_target_kinematic_traffic_reference_point_calcs = AchievePointCalcs(
            m_im_clearance.GetTrafficReferencePoint(),
            m_target_aircraft_intent,
            m_target_kinematic_trajectory_predictor.GetKinematicDescent4dPredictor()->GetVerticalPath(),
            m_target_kinematic_trajectory_predictor.GetHorizontalPath(),
            m_ownship_kinematic_achieve_by_calcs,
            m_ownship_aircraft_intent);


      m_target_distance_calculator.UpdateHorizontalTrajectory(
            m_target_kinematic_trajectory_predictor.GetHorizontalPath());

      if (!m_target_kinematic_trajectory_dumped) {
         InternalObserver::getInstance()->dumpTargetKinTraj(targetsyncstate.m_id,
                                                            m_target_kinematic_trajectory_predictor.GetKinematicDescent4dPredictor()->GetVerticalPath());
         m_target_kinematic_trajectory_dumped = true;
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
      const AircraftState &owntruthstate,
      const AircraftState &targettruthstate) {

   if (IsBlendWind() && !m_compute_ownship_kinematic_trajectory) {

      // calculate ownship reference data
      CalculateOwnshipDtgToPlannedTerminationPoint(owntruthstate);
      // Don't set m_ownship_reference_lookup_index here because it contradicts IMDistBasedAchieve
      int ownship_reference_index = CoreUtils::FindNearestIndex(
            Units::MetersLength(m_ownship_kinematic_dtg_to_ptp).value(),
            m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances());
      if (ownship_reference_index + 1 >= m_ownship_kinematic_trajectory_predictor.GetVerticalPathTimes().size()) {
         // end
         m_ownship_reference_cas = Units::MetersPerSecondSpeed(
               m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities().back());
         m_ownship_reference_altitude = Units::MetersLength(
               m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes().back());
      } else if (ownship_reference_index == 0) {
         // beginning
         m_ownship_reference_cas = Units::MetersPerSecondSpeed(
               m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocityByIndex(0));
         m_ownship_reference_altitude = Units::MetersLength(
               m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudeByIndex(0));
      } else {
         // normal case, middle somewhere, so interpolate
         m_ownship_reference_cas = Units::MetersPerSecondSpeed(
               CoreUtils::LinearlyInterpolate(ownship_reference_index,
                                              Units::MetersLength(m_ownship_kinematic_dtg_to_ptp).value(),
                                              m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances(),
                                              m_ownship_kinematic_trajectory_predictor.GetVerticalPathVelocities()));
         m_ownship_reference_altitude = Units::MetersLength(
               CoreUtils::LinearlyInterpolate(ownship_reference_index,
                                              Units::MetersLength(m_ownship_kinematic_dtg_to_ptp).value(),
                                              m_ownship_kinematic_trajectory_predictor.GetVerticalPathDistances(),
                                              m_ownship_kinematic_trajectory_predictor.GetVerticalPathAltitudes()));
      }

      std::shared_ptr<Atmosphere> sensed_atmosphere(
            StandardAtmosphere::MakeInstance(owntruthstate.m_sensed_temperature, Units::FeetLength(owntruthstate.m_z)));

      if (!m_predicted_wind_evaluator->ArePredictedWindsAccurate(
            owntruthstate, m_weather_prediction, m_ownship_reference_cas,
            m_ownship_reference_altitude, sensed_atmosphere.get())) {

//         bool recalc = true;

//         if (recalc) {
         Wind::UpdatePredictedWindsAtAltitudeFromSensedWind(owntruthstate, m_weather_prediction);
         m_compute_ownship_kinematic_trajectory = true;
         // propagate sensed atmosphere
         m_weather_prediction.SetAtmosphere(sensed_atmosphere);
         //m_ownship_kinetic_trajectory_predictor->SetAtmosphere(sensed_atmosphere);
         //m_ownship_kinematic_trajectory_predictor->SetAtmosphere(IMsensed_atmosphere);
//         }
         LOG4CPLUS_DEBUG(logger, "Bad wind prediction at t=" << owntruthstate.m_time <<
                                                             ", target ac exists=" << m_target_aircraft_exists <<
                                                             ", ref_cas=" << m_ownship_reference_cas <<
                                                             ", ref_alt="
                                                             << Units::FeetLength(m_ownship_reference_altitude) <<
                                                             ", true_alt=" << Units::FeetLength(owntruthstate.m_z) <<
                                                             ", recalc=" << "true");

      }
   }

   if (IsBlendWind() &&
       m_target_aircraft_exists &&
       !m_compute_target_kinematic_trajectory &&
       m_stage_of_im_operation != FlightStage::MAINTAIN) {
      // calculate target reference altitude -- m_target_reference_altitude
      CalculateTargetDtgToImPoints(targettruthstate);
      int target_reference_index = CoreUtils::FindNearestIndex(
            Units::MetersLength(m_target_kinematic_dtg_to_last_waypoint).value(),
            m_target_kinematic_trajectory_predictor.GetVerticalPathDistances());
      if (target_reference_index + 1 >= m_target_kinematic_trajectory_predictor.GetVerticalPathTimes().size()) {
         // end
         m_target_reference_altitude = Units::MetersLength(
               m_target_kinematic_trajectory_predictor.GetVerticalPathAltitudes().back());
      } else if (target_reference_index == 0) {
         // beginning
         m_target_reference_altitude = Units::MetersLength(
               m_target_kinematic_trajectory_predictor.GetVerticalPathAltitudeByIndex(0));
      } else {
         // normal case, middle somewhere, so interpolate
         m_target_reference_altitude = Units::MetersLength(
               CoreUtils::LinearlyInterpolate(target_reference_index,
                                              Units::MetersLength(m_target_kinematic_dtg_to_last_waypoint).value(),
                                              m_target_kinematic_trajectory_predictor.GetVerticalPathDistances(),
                                              m_target_kinematic_trajectory_predictor.GetVerticalPathAltitudes()));
      }

      // target reported altitude -- targettruthstate.m_z
      // different by 3000 feet?
      Units::FeetLength target_altitude_error =
            m_target_reference_altitude - Units::FeetLength(targettruthstate.m_z);
      if (abs(target_altitude_error) > TARGET_ALTITUDE_TOLERANCE) {
         // for 2 iterations?
         if (++m_target_altitude_failure_count >= 2) {
            m_compute_target_kinematic_trajectory = true;
         }
         LOG4CPLUS_DEBUG(logger, "Target altitude ref=" <<
                                                        Units::FeetLength(m_target_reference_altitude) <<
                                                        ", true=" << Units::FeetLength(targettruthstate.m_z) <<
                                                        ", " << m_target_altitude_failure_count <<
                                                        " iterations at t=" << targettruthstate.m_time);
      } else {
         m_target_altitude_failure_count = 0;
      }
   }
}


void IMKinematicAchieve::CalculateOwnshipDtgToPlannedTerminationPoint(const AircraftState &current_ownship_state) {

   m_ownship_distance_calculator.CalculateAlongPathDistanceFromPosition(Units::FeetLength(current_ownship_state.m_x),
                                                                        Units::FeetLength(current_ownship_state.m_y),
                                                                        m_ownship_kinematic_dtg_to_ptp);
   m_ownship_kinetic_distance_calculator.CalculateAlongPathDistanceFromPosition(
         Units::FeetLength(current_ownship_state.m_x),
         Units::FeetLength(current_ownship_state.m_y),
         m_ownship_kinetic_dtg_to_ptp);


}

void IMKinematicAchieve::CalculateTargetDtgToImPoints(const AircraftState &current_lead_state) {
   if (m_target_aircraft_exists) {
      if (!IsTargetPassedLastWaypoint()) {
         m_target_distance_calculator.CalculateAlongPathDistanceFromPosition(Units::FeetLength(current_lead_state.m_x),
                                                                             Units::FeetLength(current_lead_state.m_y),
                                                                             m_target_kinematic_dtg_to_last_waypoint);
         if (!IsTargetPassedTrp()) {
            m_target_kinematic_dtg_to_trp =
                  m_target_kinematic_dtg_to_last_waypoint -
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

bool IMKinematicAchieve::CalculateRFLegPhase() {
   bool has_rf_leg = false;
   std::vector<PrecalcWaypoint> waypoints = m_ownship_kinematic_trajectory_predictor.GetPrecalcWaypoints();
   int num_waypoints = waypoints.size();
   for (int i = 0; i < num_waypoints; i++) {
      if (waypoints[i].m_radius_rf_leg.value() > 1.0) {
         has_rf_leg = true;
         break;
      }
   }

   if (!has_rf_leg) {
      return false;
   }

   bool upper_limit_message_written = false;

   std::shared_ptr<KinematicDescent4DPredictor> kd4dp = m_ownship_kinematic_trajectory_predictor.GetKinematicDescent4dPredictor();

   Units::Acceleration decel_rate = Units::MetersSecondAcceleration(kd4dp->GetDecelerationRateFPA());
   Units::HertzFrequency delta_speed_factor;

   double last_distance = 0;
   double base_distance = 0;
   double base_ias = 0;
   RFLegPhase last_leg_phase = NON_RF_LEG;
   VerticalPath vp = kd4dp->GetVerticalPath();
   std::vector<HorizontalPath> htraj = m_ownship_kinematic_trajectory_predictor.GetHorizontalPath();

   int last_index = 0; // index for last distance in vertical path that has been used
   int htraj_index = 0;
   double restricted_speed = 0; // mps
   Units::MetersPerSecondSpeed ground_speed = Units::ZERO_SPEED;

   for (int i = 0; i < num_waypoints; i++) {
      if (i == num_waypoints - 1) {
         m_rfleg_limits.push_back(std::pair<Units::Length, Units::Speed>(Units::MetersLength(vp.x[vp.x.size() - 1]),
                                                                         Units::MetersPerSecondSpeed(
                                                                               restricted_speed)));
         break;
      }
      if (waypoints[i].m_radius_rf_leg.value() > 1.0) { // ON_RF_LEG
         // if preceding leg was NON_RF_LEG, end no-restriction segment
         if (last_leg_phase == NON_RF_LEG) {
            m_rfleg_limits.push_back(
                  std::pair<Units::Length, Units::Speed>(Units::MetersLength(last_distance), Units::zero()));
            restricted_speed = waypoints[i].m_precalc_constraints.constraint_speedHi;
            m_rfleg_limits.push_back(std::pair<Units::Length, Units::Speed>(Units::MetersLength(last_distance),
                                                                            Units::MetersPerSecondSpeed(
                                                                                  restricted_speed)));
            last_leg_phase = ON_RF_LEG;
            last_distance = waypoints[i].m_precalc_constraints.constraint_dist;
         } else if (last_leg_phase == PRE_RF_LEG) {
            // continue with previous deceleration line until it reaches constraint speed
            if (!upper_limit_message_written) {
               upper_limit_message_written = true;
               LOG4CPLUS_WARN(logger, "Pre-RF-Leg upper limit ias ("
                     << Units::MetersPerSecondSpeed(Units::MetersLength(last_distance - base_distance)
                                                    * delta_speed_factor).value()
                     << ") is less than constraint speed at precalc waypoint "
                     << i << " at distance " << last_distance << ".");
            }
            double delta_speed = waypoints[i].m_precalc_constraints.constraint_speedHi - base_ias;
            double delta_distance = Units::MetersLength(Units::MetersPerSecondSpeed(delta_speed)
                                                        / (delta_speed_factor)).value();
            if (delta_distance > (waypoints[i].m_precalc_constraints.constraint_dist - base_distance)) {
               last_leg_phase = PRE_RF_LEG;
            } else {
               restricted_speed = waypoints[i + 1].m_precalc_constraints.constraint_speedHi;
               m_rfleg_limits.push_back(
                     std::pair<Units::Length, Units::Speed>(Units::MetersLength(base_distance + delta_distance),
                                                            Units::MetersPerSecondSpeed(restricted_speed)));
               last_leg_phase = ON_RF_LEG;
            }
            last_distance = waypoints[i].m_precalc_constraints.constraint_dist;
         } else { // last_leg_phase == ON_RF_LEG
            // start new deceleration line,
            // restriction is minimum of deceleration line and constraint speed
            if (waypoints[i].m_precalc_constraints.constraint_speedHi - restricted_speed < 0.0001) {
               // continue with same restricted speed
               last_distance = waypoints[i].m_precalc_constraints.constraint_dist;
               continue;
            }

            base_ias = restricted_speed;
            base_distance = last_distance;
            m_rfleg_limits.push_back(std::pair<Units::Length, Units::Speed>(Units::MetersLength(last_distance),
                                                                            Units::MetersPerSecondSpeed(
                                                                                  restricted_speed)));

            // get ground speed to calculate deceleration factor
            bool dist_found = false;
            for (int j = last_index; j < vp.x.size(); j++) {
               if (last_distance < vp.x[j]) {
                  last_index = j;
                  dist_found = true;
                  // Get groundspeed from horizontal trajectory
                  for (htraj_index = htraj.size() - 1; htraj_index >= 0; htraj_index--) {
                     if (htraj[htraj_index].m_path_length_cumulative_meters <= vp.x[j]) {
                        ground_speed = htraj[htraj_index].m_turn_info.groundspeed;
                        delta_speed_factor = decel_rate / ground_speed;
                        break;
                     }
                  }
                  // Need test for "not found in h_traj" error condition?  Should not happen.
                  break;
               }
            }
            if (!dist_found) { // before beginning of route - should not happen
               LOG4CPLUS_WARN(logger,
                              "While attempting to create upper speed restriction for RF_leg, distance is more than route length");
               last_distance = waypoints[i].m_precalc_constraints.constraint_dist;
               continue;
            }

            double delta_speed = waypoints[i].m_precalc_constraints.constraint_speedHi - base_ias;
            double delta_distance = Units::MetersLength(Units::MetersPerSecondSpeed(delta_speed)
                                                        / (delta_speed_factor)).value();
            if (delta_distance > (waypoints[i].m_precalc_constraints.constraint_dist - last_distance)) {
               last_leg_phase = PRE_RF_LEG;
            } else {
               restricted_speed = waypoints[i].m_precalc_constraints.constraint_speedHi;
               m_rfleg_limits.push_back(
                     std::pair<Units::Length, Units::Speed>(Units::MetersLength(last_distance + delta_distance),
                                                            Units::MetersPerSecondSpeed(restricted_speed)));
               last_leg_phase = ON_RF_LEG;
            }
            last_distance = waypoints[i].m_precalc_constraints.constraint_dist;
         }
      } else { // NON_RF_LEG
         if (last_distance == 0) {
            m_rfleg_limits.push_back(std::pair<Units::Length, Units::Speed>(Units::zero(), Units::zero()));
            last_distance = waypoints[i].m_precalc_constraints.constraint_dist;
         } else if (last_leg_phase == NON_RF_LEG) {
            last_distance = waypoints[i].m_precalc_constraints.constraint_dist;
         } else if (last_leg_phase == ON_RF_LEG) {
            m_rfleg_limits.push_back(std::pair<Units::Length, Units::Speed>(Units::MetersLength(last_distance),
                                                                            Units::MetersPerSecondSpeed(
                                                                                  restricted_speed)));
            // start deceleration line..
            // restriction is minumum of deceleration line and nominal + HIGH_LIMIT
            double segment_end = waypoints[i].m_precalc_constraints.constraint_dist;
            for (int j = last_index; j < vp.x.size(); j++) {
               if (last_distance < vp.x[j]) {
                  last_index = j;
                  break;
               }
            }
            base_ias = restricted_speed;
            base_distance = last_distance;

            // get ground speed to calculate deceleration factor
            bool dist_found = false;
            for (int j = last_index; j < vp.x.size(); j++) {
               if (last_distance < vp.x[j]) {
                  last_index = j;
                  dist_found = true;
                  // Get groundspeed from horizontal trajectory
                  for (htraj_index = htraj.size() - 1; htraj_index >= 0; htraj_index--) {
                     if (htraj[htraj_index].m_path_length_cumulative_meters <= vp.x[j]) {
                        ground_speed = htraj[htraj_index - 1].m_turn_info.groundspeed;
                        delta_speed_factor = decel_rate / ground_speed;
                        break;
                     }
                  }
                  // Need test for "not found in h_traj" error condition?  Should not happen.
                  break;
               }
            }
            if (!dist_found) { // before beginning of route - should not happen
               LOG4CPLUS_WARN(logger,
                              "While attempting to create upper speed restriction for RF_leg, distance is more than route length");
               last_distance = waypoints[i].m_precalc_constraints.constraint_dist;
               continue;
            }
            // find nominal speed and compare deceleration line
            for (; last_index < vp.x.size(); last_index++) {
               if (vp.x[last_index] > segment_end) {
                  break;
               }
               Units::MetersLength flight_distance(vp.x[last_index] - base_distance);
               restricted_speed = Units::MetersPerSecondSpeed(flight_distance * delta_speed_factor).value() + base_ias;
               if (vp.v[last_index] - restricted_speed > 2.5) { // allow up to 5 knots difference
                  if (!upper_limit_message_written) {
                     upper_limit_message_written = true;
                     LOG4CPLUS_WARN(logger, "Pre-RF-Leg upper limit ias (" << restricted_speed
                                                                           << ") is less than predicted airspeed ("
                                                                           << vp.v[last_index] << ") at distance "
                                                                           << vp.x[last_index] << ".");
                  }
               }
               if (restricted_speed >= HighLimit(vp.v[last_index])) {
                  // reached nominal upper limit
                  break;
               }
            }

            if (vp.x[last_index] < segment_end) { // deceleration line reached nominal + HIGH_LIMIT
               m_rfleg_limits.push_back(std::pair<Units::Length, Units::Speed>(Units::MetersLength(vp.x[last_index]),
                                                                               Units::MetersPerSecondSpeed(
                                                                                     restricted_speed)));
               m_rfleg_limits.push_back(
                     std::pair<Units::Length, Units::Speed>(Units::MetersLength(vp.x[last_index]), Units::zero()));
               last_leg_phase = NON_RF_LEG;
               last_distance = segment_end;
               restricted_speed = 0;
            } else {  // at end of segment, below nominal + HIGH_LIMIT
               last_leg_phase = PRE_RF_LEG;
               last_distance = segment_end;
            }
         } else if (last_leg_phase == PRE_RF_LEG) {
            // deceleration line is less than constraint speed
            double segment_end = waypoints[i].m_precalc_constraints.constraint_dist;
            for (; last_index < vp.x.size(); last_index++) {
               if (vp.x[last_index] > segment_end) {
                  break;
               }
               Units::MetersLength flight_distance(vp.x[last_index] - base_distance);
               restricted_speed = Units::MetersPerSecondSpeed(flight_distance * delta_speed_factor).value() + base_ias;
               if (restricted_speed < vp.v[last_index]) {
                  if (!upper_limit_message_written) {
                     upper_limit_message_written = true;
                     LOG4CPLUS_WARN(logger, "Pre-RF-Leg upper limit ias (" << restricted_speed
                                                                           << ") is less than predicted airspeed ("
                                                                           << vp.v[last_index] << ") at distance "
                                                                           << vp.x[last_index] << ".");
                  }
               }
               if (restricted_speed >= HighLimit(vp.v[last_index])) {
                  break;
               }
            }
            if (vp.x[last_index] <= segment_end) {
               m_rfleg_limits.push_back(std::pair<Units::Length, Units::Speed>(Units::MetersLength(vp.x[last_index]),
                                                                               Units::MetersPerSecondSpeed(
                                                                                     restricted_speed)));
               m_rfleg_limits.push_back(
                     std::pair<Units::Length, Units::Speed>(Units::MetersLength(vp.x[last_index]), Units::zero()));
               last_leg_phase = NON_RF_LEG;
               restricted_speed = 0;
            }
            last_distance = segment_end;
         }
      }
   }


   return has_rf_leg;

}

void IMKinematicAchieve::ComputeFASTrajectories(
      const AircraftState &owntruthstate,
      const AircraftState &targettruthstate) {

   using namespace std;

   LOG4CPLUS_DEBUG(logger, "Calculating FAS trajectories on AC " <<
                                                                 owntruthstate.m_id << " at " << owntruthstate.m_time);

   // find merge point
   Units::MetersLength x1, y1, x2, y2, x3, y3;  // point parameters
   string achieve_by_waypoint_name = m_im_clearance.GetAchieveByPoint();
   int achieve_by_waypoint_index = m_ownship_aircraft_intent.GetWaypointIndexByName(achieve_by_waypoint_name);
   Waypoint ptp_waypoint(m_ownship_aircraft_intent.GetWaypoint(achieve_by_waypoint_index));

   size_t target_end_waypoint_ix = m_target_aircraft_intent.GetNumberOfWaypoints() - 1;
   Waypoint target_end_waypoint(m_target_aircraft_intent.GetWaypoint(target_end_waypoint_ix));

   LOG4CPLUS_DEBUG(logger, "Achieve by = " << achieve_by_waypoint_name);

   // use CalculateTrackAngle to find merge angle
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
   Units::Angle merge_angle = Scenario::m_rand.GaussianSample(
         merge_angle_mean, merge_angle_std);
   Units::DegreesAngle final_approach_angle = reverse_final_approach_angle + Units::PI_RADIANS_ANGLE;
   LOG4CPLUS_DEBUG(logger, "Average track angle = " <<
                                                    Units::DegreesAngle(merge_angle_mean) <<
                                                    ", randomized merge angle = " <<
                                                    Units::DegreesAngle(merge_angle) <<
                                                    ", final_approach_angle = " <<
                                                    final_approach_angle);

   Units::MetersLength xMerge, yMerge;
   IMUtils::CalculateMergePoint(x1, y1, x2, y2, x3, y3,
                                xMerge, yMerge, merge_angle);

   LOG4CPLUS_DEBUG(logger, "Merge point is (" << xMerge << "," << yMerge << ")");

   // construct intents
   // final AC:  current_pos last_wp
   // vector AC:  current_pos merge_point last_wp
   AircraftIntent own_intent(m_ownship_aircraft_intent), target_intent(m_target_aircraft_intent);
   own_intent.ClearWaypoints();
   target_intent.ClearWaypoints();
   own_intent.SetPlannedCruiseAltitude(Units::FeetLength(owntruthstate.m_z));
   target_intent.SetPlannedCruiseAltitude(Units::FeetLength(targettruthstate.m_z));

   own_intent.InsertWaypointAtIndex(ptp_waypoint, 0);
   target_intent.InsertWaypointAtIndex(target_end_waypoint, 0);

   pair<Units::Length, Units::Length> final_wpt_coords(x2, y2);
   pair<Units::Length, Units::Length> merge_coords(xMerge, yMerge);
   pair<Units::Length, Units::Length> current_coords(x3, y3);

   Units::MetersLength merge_to_lastwpt_distance =
         CoreUtils::CalculateEuclideanDistance(merge_coords, final_wpt_coords);
   Units::MetersLength current_to_lastwpt_distance =
         CoreUtils::CalculateEuclideanDistance(current_coords, final_wpt_coords);
   Units::MetersLength current_to_merge_distance =
         CoreUtils::CalculateEuclideanDistance(current_coords, merge_coords);

   LOG4CPLUS_DEBUG(logger, "current = (" <<
                                         Units::MetersLength(current_coords.first) << "," <<
                                         Units::MetersLength(current_coords.second) << ")");
   LOG4CPLUS_DEBUG(logger, "merge = (" <<
                                       Units::MetersLength(merge_coords.first) << "," <<
                                       Units::MetersLength(merge_coords.second) << ")");
   LOG4CPLUS_DEBUG(logger, "last wpt = (" <<
                                          Units::MetersLength(final_wpt_coords.first) << "," <<
                                          Units::MetersLength(final_wpt_coords.second) << ")");
   LOG4CPLUS_DEBUG(logger, "Current to merge = " << current_to_merge_distance);
   LOG4CPLUS_DEBUG(logger, "Current to lastwpt = " << current_to_lastwpt_distance);
   LOG4CPLUS_DEBUG(logger, "Merge to lastwpt = " << merge_to_lastwpt_distance);

   // convert states to waypoints, using ownship's sensed winds
   own_intent.InsertWaypointAtIndex(
         MakeWaypointFromState(owntruthstate, Units::MetersPerSecondSpeed(owntruthstate.m_Vwx),
                               Units::MetersPerSecondSpeed(owntruthstate.m_Vwy)), 0);

   // Get predicted winds at target aircraft's current altitude & create waypoint
   Units::MetersPerSecondSpeed target_wind_x, target_wind_y;
   Units::HertzFrequency dVwx_dh, dVwy_dh;
   m_weather_prediction.getAtmosphere()->CalculateWindGradientAtAltitude(
         Units::FeetLength(targettruthstate.m_z), m_weather_prediction.east_west, target_wind_x, dVwx_dh);
   m_weather_prediction.getAtmosphere()->CalculateWindGradientAtAltitude(
         Units::FeetLength(targettruthstate.m_z), m_weather_prediction.north_south, target_wind_y, dVwy_dh);
   target_intent.InsertWaypointAtIndex(MakeWaypointFromState(targettruthstate, target_wind_x, target_wind_y), 0);

   const Units::DegreesAngle tolerance(10.0);
   const Units::NauticalMilesLength too_close(.83); // see AAES-939
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

Waypoint IMKinematicAchieve::MakeWaypointFromState(const AircraftState aircraft_state,
                                                   Units::Speed wind_x,
                                                   Units::Speed wind_y) const {
   // calculate fields needed for a Waypoint object

   // lat, lon
   EarthModel::LocalPositionEnu local_pos;
   local_pos.x = Units::FeetLength(aircraft_state.m_x);
   local_pos.y = Units::FeetLength(aircraft_state.m_y);
   local_pos.z = Units::FeetLength(aircraft_state.m_z);

   EarthModel::GeodeticPosition geo_pos;
   m_tangent_plane_sequence->convertLocalToGeodetic(local_pos, geo_pos);

   // ias
   Units::Speed groundspeed_x = Units::FeetPerSecondSpeed(aircraft_state.m_xd)
                                - wind_x;
   Units::Speed groundspeed_y = Units::FeetPerSecondSpeed(aircraft_state.m_yd)
                                - wind_y;
   Units::KnotsSpeed tas_no_wind = sqrt(
         groundspeed_x * groundspeed_x +
         groundspeed_y * groundspeed_y);
   Units::KnotsSpeed cas;
   cas = m_weather_prediction.TAS2CAS(tas_no_wind, local_pos.z);

   return Waypoint("state",
                   geo_pos.latitude, geo_pos.longitude,
                   local_pos.z,
                   local_pos.z,
                   cas,
                   local_pos.z, cas);
}

const Units::SignedAngle
IMKinematicAchieve::CalculateTargetTrackAngle(const vector<AircraftState> &target_adsb_history) {
   vector<AircraftState>::const_reverse_iterator critr = target_adsb_history.rbegin();
   double current_time = (*critr).m_time;
   double lower = current_time - (Units::SecondsTime(TRACK_ANGLE_TAU)).value();

   // Use RadiansAngle here so that circular wrapping does not happen while summing below
   Units::RadiansAngle sum = Units::SignedRadiansAngle(atan2((*critr).m_yd, (*critr).m_xd));
   bool too_close_to_boundary = abs(sum) > Units::SignedDegreesAngle(170.0) ||
                                abs(sum) < Units::SignedDegreesAngle(10);
   if (too_close_to_boundary) {
      sum = Units::UnsignedRadiansAngle(sum);
   }

   unsigned long cnt = 1;
   for (++critr; critr != target_adsb_history.rend(); ++critr) {
      if ((*critr).m_time < lower) {
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
