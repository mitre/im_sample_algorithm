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
#include <utility>
#include "math/CustomMath.h"
#include "public/CoreUtils.h"
#include "public/AircraftCalculations.h"
#include "imalgs/IMAlgorithm.h"
#include "imalgs/InternalObserver.h"

using namespace aaesim::open_source;
using namespace interval_management::open_source;

log4cplus::Logger IMAlgorithm::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("IMAlgorithm"));

const std::string IMAlgorithm::RESET_MSG(".\nUse CUSTOM clearance type to enable this parameter.");
const double IMAlgorithm::UNDEFINED_INTERVAL(-INFINITY);

IMAlgorithm::IMAlgorithm()
   : m_target_aircraft_intent(),
     m_stage_of_im_operation(),
     m_im_clearance(),
     m_pilot_delay(),
     m_weather_prediction(),
     m_speed_limiter(),
     m_previous_reference_im_speed_command_tas(Units::ZERO_SPEED),
     m_previous_im_speed_command_ias(Units::ZERO_SPEED),
     m_error_distance(interval_management::open_source::FIMConfiguration::ERROR_DISTANCE_DEFAULT),
     m_slope(interval_management::open_source::FIMConfiguration::SLOPE_DEFAULT),
     m_initiate_signal_receipt_time(Units::Infinity()),
     m_ownship_ttg_to_abp(Units::SecondsTime(-1.0)),
     m_ownship_reference_ttg_to_ptp(Units::zero()),
     m_target_ttg_to_trp(Units::zero()),
     m_target_trp_crossing_time(Units::zero()),
     m_target_ttg_to_end_of_route(Units::zero()),
     m_ownship_kinematic_dtg_to_abp(Units::Infinity()),
     m_ownship_kinematic_dtg_to_ptp(Units::Infinity()),
     m_target_kinematic_dtg_to_last_waypoint(Units::Infinity()),
     m_target_kinematic_dtg_to_trp(Units::Infinity()),
     m_unmodified_im_speed_command_ias(Units::MetersPerSecondSpeed(-1.0)),
     m_im_speed_command_ias(Units::MetersPerSecondSpeed(-1.0)),
     m_im_speed_command_with_pilot_delay(Units::MetersPerSecondSpeed(-1.0)),
     m_ownship_reference_cas(Units::ZERO_SPEED),
     m_ownship_reference_gs(Units::negInfinity()),
     m_ownship_reference_altitude(Units::zero()),
     m_target_reference_altitude(Units::zero()),
     m_target_reference_ias(Units::negInfinity()),
     m_target_reference_gs(Units::negInfinity()),
     m_total_number_of_im_speed_changes(0),
     m_target_reference_lookup_index(0),
     m_ownship_reference_lookup_index(0),
     m_reference_precalc_index(0),
     m_previous_reference_im_speed_command_mach(0.0),
     m_loaded(false),
     m_im_operation_is_complete(false),
     m_has_maintain_stage(false),
     m_loaded_middle_to_final_quantize_transition_distance(IMUtils::DIST_QUANTIZE_1_DEFAULT),
     m_loaded_first_to_middle_quantize_transition_distance(IMUtils::DIST_QUANTIZE_2_DEFAULT),
     m_loaded_speed_quantize_final_phase(IMUtils::SPEED_QUANTIZE_1_DEFAULT_1_KNOT),
     m_loaded_speed_quantize_middle_phase(IMUtils::SPEED_QUANTIZE_2_DEFAULT),
     m_loaded_speed_quantize_first_phase(IMUtils::SPEED_QUANTIZE_3_DEFAULT),
     m_loaded_use_speed_limiting(IMUtils::LIMIT_FLAG_DEFAULT),
     m_loaded_use_speed_quantization(IMUtils::QUANTIZE_FLAG_DEFAULT) {
   IterClearIMAlg();
}

IMAlgorithm::IMAlgorithm(const IMAlgorithm &obj) : m_initiate_signal_receipt_time(Units::Infinity()) {
   m_total_number_of_im_speed_changes = 0;
   m_target_reference_lookup_index = 0;
   m_ownship_reference_lookup_index = 0;
   m_reference_precalc_index = 0;
   m_previous_reference_im_speed_command_mach = 0.0;
   m_loaded = false;
   m_im_operation_is_complete = false;
   m_ownship_kinematic_dtg_to_ptp = Units::Infinity();
   Copy(obj);
}

IMAlgorithm &IMAlgorithm::operator=(const IMAlgorithm &obj) {
   if (this != &obj) {
      Copy(obj);
   }

   return *this;
}

void IMAlgorithm::Copy(const IMAlgorithm &obj) {
   m_total_number_of_im_speed_changes = obj.m_total_number_of_im_speed_changes;

   m_pilot_delay = obj.m_pilot_delay;

   m_slope = obj.m_slope;
   m_error_distance = obj.m_error_distance;

   m_previous_reference_im_speed_command_tas = obj.m_previous_reference_im_speed_command_tas;
   m_previous_im_speed_command_ias = obj.m_previous_im_speed_command_ias;
   m_previous_reference_im_speed_command_mach = obj.m_previous_reference_im_speed_command_mach;

   m_target_reference_lookup_index = obj.m_target_reference_lookup_index;
   m_ownship_reference_lookup_index = obj.m_ownship_reference_lookup_index;
   m_reference_precalc_index = obj.m_reference_precalc_index;

   m_target_aircraft_intent = obj.m_target_aircraft_intent;
   m_weather_prediction = obj.m_weather_prediction;
   m_stage_of_im_operation = obj.m_stage_of_im_operation;
   m_initiate_signal_receipt_time = obj.m_initiate_signal_receipt_time;
   m_target_trp_crossing_time = obj.m_target_trp_crossing_time;
   m_target_ttg_to_trp = obj.m_target_ttg_to_trp;
   m_target_ttg_to_end_of_route = obj.m_target_ttg_to_end_of_route;

   m_ownship_reference_ttg_to_ptp = obj.m_ownship_reference_ttg_to_ptp;
   m_ownship_ttg_to_abp = obj.m_ownship_ttg_to_abp;
   m_ownship_kinematic_dtg_to_abp = obj.m_ownship_kinematic_dtg_to_abp;

   m_unmodified_im_speed_command_ias = obj.m_unmodified_im_speed_command_ias;
   m_im_speed_command_ias = obj.m_im_speed_command_ias;
   m_im_speed_command_with_pilot_delay = obj.m_im_speed_command_with_pilot_delay;

   m_loaded = obj.m_loaded;
   m_im_operation_is_complete = obj.m_im_operation_is_complete;
   m_im_clearance = obj.m_im_clearance;

   m_target_kinematic_dtg_to_trp = obj.m_target_kinematic_dtg_to_trp;
   m_target_kinematic_dtg_to_last_waypoint = obj.m_target_kinematic_dtg_to_last_waypoint;
   m_ownship_kinematic_dtg_to_ptp = obj.m_ownship_kinematic_dtg_to_ptp;

   m_ownship_reference_gs = obj.m_ownship_reference_gs;
   m_target_reference_ias = obj.m_target_reference_ias;
   m_target_reference_gs = obj.m_target_reference_gs;

   m_loaded_middle_to_final_quantize_transition_distance = obj.m_loaded_middle_to_final_quantize_transition_distance;
   m_loaded_first_to_middle_quantize_transition_distance = obj.m_loaded_first_to_middle_quantize_transition_distance;
   m_loaded_speed_quantize_final_phase = obj.m_loaded_speed_quantize_final_phase;
   m_loaded_speed_quantize_middle_phase = obj.m_loaded_speed_quantize_middle_phase;
   m_loaded_speed_quantize_first_phase = obj.m_loaded_speed_quantize_first_phase;
   m_loaded_use_speed_limiting = obj.m_loaded_use_speed_limiting;
   m_loaded_use_speed_quantization = obj.m_loaded_use_speed_quantization;
}

void IMAlgorithm::CopyParametersFromConfiguration() {
   m_achieve_control_gain = m_configuration.m_achieve_control_gain;
   m_maintain_control_gain = m_configuration.m_maintain_control_gain;
   m_time_threshold = m_configuration.m_time_threshold;
   m_slope = m_configuration.m_slope;
   m_error_distance = m_configuration.m_error_distance;
   m_loaded_use_speed_limiting = m_configuration.m_use_speed_limiting;
   m_threshold_flag = m_configuration.m_threshold_flag;
   m_loaded_use_speed_quantization = m_configuration.m_use_speed_quantization;
   m_loaded_middle_to_final_quantize_transition_distance =
         m_configuration.m_loaded_middle_to_final_quantize_transition_distance;
   m_loaded_first_to_middle_quantize_transition_distance =
         m_configuration.m_loaded_first_to_middle_quantize_transition_distance;
   m_loaded_speed_quantize_final_phase = m_configuration.m_loaded_speed_quantize_final_phase;
   m_loaded_speed_quantize_middle_phase = m_configuration.m_loaded_speed_quantize_middle_phase;
   m_loaded_speed_quantize_first_phase = m_configuration.m_loaded_speed_quantize_first_phase;
}

bool IMAlgorithm::ValidateClearance(const AircraftIntent &ownship_aircraft_intent,
                                    const IMUtils::IMAlgorithmTypes im_algorithm_type) {
   bool result = m_im_clearance.Validate(ownship_aircraft_intent, im_algorithm_type);
   return result;
}

void IMAlgorithm::Initialize(const OwnshipPredictionParameters &ownship_prediction_parameters,
                             const AircraftIntent &ownship_aircraft_intent, WeatherPrediction &weather_prediction) {
   m_im_operation_is_complete = false;
   m_ownship_kinematic_dtg_to_ptp = Units::Infinity();
   SetAssignedSpacingGoal(m_im_clearance);
   // Check whether m_target_aircraft_intent was initialized by a subclass
   if (!m_target_aircraft_intent.IsLoaded()) {
      m_target_aircraft_intent = m_im_clearance.GetTargetAircraftIntent();
   }
   SetWeatherPrediction(weather_prediction);

   m_speed_limiter.Initialize(m_pilot_delay.IsPilotDelayOn());
   m_has_maintain_stage = !m_im_clearance.AbpAndPtpAreColocated();
}

void IMAlgorithm::ResetDefaults() {
   // TODO remove this function
}

aaesim::open_source::Guidance IMAlgorithm::Update(
      const aaesim::open_source::Guidance &prevguidance, const aaesim::open_source::DynamicsState &dynamicsstate,
      const interval_management::open_source::AircraftState &owntruthstate,
      const interval_management::open_source::AircraftState &targettruthstate,
      const std::vector<interval_management::open_source::AircraftState> &targethistory) {

   return prevguidance;
}

void IMAlgorithm::SetPilotDelay(const bool pilot_delay_on, const Units::Time pilot_delay_mean,
                                const Units::Time pilot_delay_standard_deviation) {
   m_pilot_delay.SetUsePilotDelay(pilot_delay_on);
   m_pilot_delay.SetPilotDelayParameters(pilot_delay_mean, pilot_delay_standard_deviation);
}

void IMAlgorithm::IterClearIMAlg() {
   m_pilot_delay.IterationReset();
   m_total_number_of_im_speed_changes = 0;

   m_previous_reference_im_speed_command_tas = Units::ZERO_SPEED;
   m_previous_im_speed_command_ias = Units::ZERO_SPEED;
   m_previous_reference_im_speed_command_mach = 0;

   m_target_reference_lookup_index = -1;
   m_ownship_reference_lookup_index = -1;
   m_reference_precalc_index = -1;
   m_im_operation_is_complete = false;
   m_stage_of_im_operation = UNSET;

   m_ownship_ttg_to_abp = Units::SecondsTime(-1.0);
   m_ownship_reference_ttg_to_ptp = Units::zero();
   m_ownship_reference_cas = Units::ZERO_SPEED;
   m_ownship_reference_altitude = Units::zero();

   m_target_ttg_to_end_of_route = Units::zero();
   m_target_ttg_to_trp = Units::zero();
   m_target_trp_crossing_time = Units::zero();

   m_ownship_kinematic_dtg_to_abp = Units::Infinity();
   m_ownship_kinematic_dtg_to_ptp = Units::Infinity();

   m_target_kinematic_dtg_to_last_waypoint = Units::Infinity();
   m_target_kinematic_dtg_to_trp = Units::Infinity();
   m_target_reference_altitude = Units::zero();
   m_unmodified_im_speed_command_ias = Units::MetersPerSecondSpeed(-1.0);
   m_im_speed_command_ias = Units::MetersPerSecondSpeed(-1.0);
   m_im_speed_command_with_pilot_delay = Units::MetersPerSecondSpeed(-1.0);

   m_target_reference_gs = Units::negInfinity();
   m_target_reference_ias = Units::negInfinity();
   m_ownship_reference_gs = Units::negInfinity();
}

void IMAlgorithm::IterationReset() { IterClearIMAlg(); }

void IMAlgorithm::UpdatePositionMetrics(const interval_management::open_source::AircraftState &ownship_aircraft_state,
                                        const interval_management::open_source::AircraftState &target_aircraft_state) {
   if (target_aircraft_state.GetId() != IMUtils::UNINITIALIZED_AIRCRAFT_ID) {
      MergePointMetric &merge_point_metric =
            InternalObserver::getInstance()->GetMergePointMetric(ownship_aircraft_state.GetId());
      if (merge_point_metric.mergePointFound()) {
         merge_point_metric.update(ownship_aircraft_state.m_x, ownship_aircraft_state.m_y, target_aircraft_state.m_x,
                                   target_aircraft_state.m_y);
      }
      InternalObserver::getInstance()
            ->GetClosestPointMetric(ownship_aircraft_state.GetId())
            .update(ownship_aircraft_state.m_x, ownship_aircraft_state.m_y, target_aircraft_state.m_x,
                    target_aircraft_state.m_y);
   }
}

void IMAlgorithm::DumpParameters(const std::string &parameters_to_print) {
   LOG4CPLUS_DEBUG(IMAlgorithm::m_logger, "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl);

   LOG4CPLUS_DEBUG(IMAlgorithm::m_logger, "IMAlgorithm parms for " << parameters_to_print.c_str() << std::endl
                                                                   << std::endl);

   LOG4CPLUS_DEBUG(IMAlgorithm::m_logger, "mLoaded " << m_loaded << std::endl);

   LOG4CPLUS_DEBUG(IMAlgorithm::m_logger, "mSpeedChangeCount " << m_total_number_of_im_speed_changes << std::endl);

   LOG4CPLUS_DEBUG(IMAlgorithm::m_logger,
                   "slope " << Units::SecondsPerNauticalMileInvertedSpeed(m_slope).value() << std::endl);
   LOG4CPLUS_DEBUG(IMAlgorithm::m_logger,
                   "error distance " << Units::NauticalMilesLength(m_error_distance).value() << std::endl);

   m_pilot_delay.DumpParameters("mPilotDelay");

   LOG4CPLUS_DEBUG(IMAlgorithm::m_logger, "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl);
}
