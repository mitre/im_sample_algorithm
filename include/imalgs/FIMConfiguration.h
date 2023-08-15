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

#pragma once

#include "loader/DecodedStream.h"
#include "loader/Loadable.h"
#include "public/Logging.h"

namespace interval_management {
namespace open_source {
class FIMConfiguration : public Loadable {
  public:
   FIMConfiguration();
   virtual ~FIMConfiguration();

   bool load(DecodedStream *input);

   static const FIMConfiguration &GetDefaultConfiguration() { return DEFAULT_CONFIGURATION; }

  private:
   static log4cplus::Logger m_logger;
   static const FIMConfiguration DEFAULT_CONFIGURATION;

  public:
   static const Units::HertzFrequency ACHIEVE_CONTROL_GAIN_DEFAULT;
   static const Units::HertzFrequency MAINTAIN_CONTROL_GAIN_DEFAULT;
   static const Units::SecondsTime TIME_THRESHOLD_DEFAULT;
   static const bool THRESHOLD_FLAG_DEFAULT;
   static const Units::NauticalMilesLength ERROR_DISTANCE_DEFAULT;
   static const Units::SecondsPerNauticalMileInvertedSpeed SLOPE_DEFAULT;

   Units::HertzFrequency m_achieve_control_gain;
   Units::HertzFrequency m_maintain_control_gain;
   Units::SecondsTime m_time_threshold;
   Units::SecondsPerNauticalMileInvertedSpeed m_slope;
   Units::NauticalMilesLength m_error_distance;

   bool m_use_speed_limiting;
   bool m_threshold_flag;
   bool m_use_speed_quantization;

   Units::NauticalMilesLength m_loaded_middle_to_final_quantize_transition_distance;
   Units::NauticalMilesLength m_loaded_first_to_middle_quantize_transition_distance;
   Units::KnotsSpeed m_loaded_speed_quantize_final_phase;
   Units::KnotsSpeed m_loaded_speed_quantize_middle_phase;
   Units::KnotsSpeed m_loaded_speed_quantize_first_phase;

   bool m_loaded;
};

} /* namespace open_source */
} /* namespace interval_management */
