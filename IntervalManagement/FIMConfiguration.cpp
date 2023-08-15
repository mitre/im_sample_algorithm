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

#include "imalgs/FIMConfiguration.h"
#include "imalgs/IMUtils.h"

using namespace interval_management::open_source;

log4cplus::Logger FIMConfiguration::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("FIMConfiguration"));
const FIMConfiguration DEFAULT_CONFIGURATION();

// Do not change these values without discussing with Lesley Weitz.
const Units::HertzFrequency FIMConfiguration::ACHIEVE_CONTROL_GAIN_DEFAULT(0.008);
const Units::HertzFrequency FIMConfiguration::MAINTAIN_CONTROL_GAIN_DEFAULT(0.008);
const Units::SecondsTime FIMConfiguration::TIME_THRESHOLD_DEFAULT(0);
const Units::SecondsPerNauticalMileInvertedSpeed FIMConfiguration::SLOPE_DEFAULT(0.25);
const Units::NauticalMilesLength FIMConfiguration::ERROR_DISTANCE_DEFAULT(0);
const bool FIMConfiguration::THRESHOLD_FLAG_DEFAULT(true);

FIMConfiguration::FIMConfiguration()
   : m_achieve_control_gain(ACHIEVE_CONTROL_GAIN_DEFAULT),
     m_maintain_control_gain(MAINTAIN_CONTROL_GAIN_DEFAULT),
     m_time_threshold(TIME_THRESHOLD_DEFAULT),
     m_slope(SLOPE_DEFAULT),
     m_error_distance(ERROR_DISTANCE_DEFAULT),

     m_use_speed_limiting(IMUtils::LIMIT_FLAG_DEFAULT),
     m_threshold_flag(THRESHOLD_FLAG_DEFAULT),
     m_use_speed_quantization(IMUtils::QUANTIZE_FLAG_DEFAULT),

     m_loaded_middle_to_final_quantize_transition_distance(IMUtils::DIST_QUANTIZE_1_DEFAULT),
     m_loaded_first_to_middle_quantize_transition_distance(IMUtils::DIST_QUANTIZE_2_DEFAULT),
     m_loaded_speed_quantize_final_phase(IMUtils::SPEED_QUANTIZE_1_DEFAULT_1_KNOT),
     m_loaded_speed_quantize_middle_phase(IMUtils::SPEED_QUANTIZE_2_DEFAULT),
     m_loaded_speed_quantize_first_phase(IMUtils::SPEED_QUANTIZE_3_DEFAULT) {}

FIMConfiguration::~FIMConfiguration() {}

bool FIMConfiguration::load(DecodedStream *input) {
   set_stream(input);

   register_var("k_achieve", &m_achieve_control_gain);
   register_var("k_maintain", &m_maintain_control_gain);

   register_var("limit", &m_use_speed_limiting);

   // speed quantization values (have defaults if not loaded)
   register_var("dist_quantize_1", &m_loaded_middle_to_final_quantize_transition_distance);
   register_var("dist_quantize_2", &m_loaded_first_to_middle_quantize_transition_distance);
   register_var("quant1", &m_loaded_speed_quantize_final_phase);
   register_var("quant2", &m_loaded_speed_quantize_middle_phase);
   register_var("quant3", &m_loaded_speed_quantize_first_phase);

   // error threshold values (have defaults if not loaded)
   register_var("error_distance", &m_error_distance);
   register_var("threshold1", &m_time_threshold);
   register_var("threshold2", &m_slope);

   register_var("use_error_threshold", &m_threshold_flag);
   register_var("use_speed_quantize", &m_use_speed_quantization);

   m_loaded = complete();

   if (m_achieve_control_gain == Units::ZERO_FREQUENCY) {
      std::string msg = "0 achieve control gain encountered-check scenario";
      LOG4CPLUS_FATAL(m_logger, msg);
      throw std::logic_error(msg);
   }

   if (m_maintain_control_gain == Units::ZERO_FREQUENCY) {
      std::string msg = "load: 0 maintain control gain encountered-check scenario";
      LOG4CPLUS_FATAL(m_logger, msg);
      throw std::logic_error(msg);
   }

   return m_loaded;
}