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
// (c) 2026 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <loader/DecodedStream.h>
#include <loader/Loadable.h>
#include <log4cplus/logger.h>

#include "imalgs/IMUtils.h"

namespace interval_management {
namespace open_source {

using namespace mitre::oss::simcore;

class FIMConfiguration final : public Loadable {
  public:
   FIMConfiguration() = default;
   virtual ~FIMConfiguration() = default;

   bool load(DecodedStream *input);

   static const FIMConfiguration &GetDefaultConfiguration();

  private:
   inline static FIMConfiguration DEFAULT_CONFIGURATION();
   static log4cplus::Logger m_logger;

  public:
   // Do not change these values without discussing with Lesley Weitz.
   inline static const Units::HertzFrequency ACHIEVE_CONTROL_GAIN_DEFAULT{0.008};
   inline static const Units::HertzFrequency MAINTAIN_CONTROL_GAIN_DEFAULT{0.008};
   inline static const Units::SecondsTime TIME_THRESHOLD_DEFAULT{0};
   inline static const bool THRESHOLD_FLAG_DEFAULT{true};
   inline static const Units::NauticalMilesLength ERROR_DISTANCE_DEFAULT{0};
   inline static const Units::SecondsPerNauticalMileInvertedSpeed SLOPE_DEFAULT{0.25};

   Units::HertzFrequency m_achieve_control_gain{ACHIEVE_CONTROL_GAIN_DEFAULT};
   Units::HertzFrequency m_maintain_control_gain{MAINTAIN_CONTROL_GAIN_DEFAULT};
   Units::SecondsTime m_time_threshold{TIME_THRESHOLD_DEFAULT};
   Units::SecondsPerNauticalMileInvertedSpeed m_slope{SLOPE_DEFAULT};
   Units::NauticalMilesLength m_error_distance{ERROR_DISTANCE_DEFAULT};
   bool m_use_speed_limiting{IMUtils::LIMIT_FLAG_DEFAULT};
   bool m_use_wind_blending{IMUtils::WIND_BLENDING_FLAG_DEFAULT};
   bool m_threshold_flag{THRESHOLD_FLAG_DEFAULT};
   bool m_use_speed_quantization{IMUtils::QUANTIZE_FLAG_DEFAULT};
   Units::NauticalMilesLength m_loaded_middle_to_final_quantize_transition_distance{IMUtils::DIST_QUANTIZE_1_DEFAULT};
   Units::NauticalMilesLength m_loaded_first_to_middle_quantize_transition_distance{IMUtils::DIST_QUANTIZE_2_DEFAULT};
   Units::KnotsSpeed m_loaded_speed_quantize_final_phase{IMUtils::SPEED_QUANTIZE_1_DEFAULT_1_KNOT};
   Units::KnotsSpeed m_loaded_speed_quantize_middle_phase{IMUtils::SPEED_QUANTIZE_2_DEFAULT};
   Units::KnotsSpeed m_loaded_speed_quantize_first_phase{IMUtils::SPEED_QUANTIZE_3_DEFAULT};

   bool m_loaded{false};
};

inline const FIMConfiguration &interval_management::open_source::FIMConfiguration::GetDefaultConfiguration() {
   static FIMConfiguration fim_configuration{};
   return fim_configuration;
}

} /* namespace open_source */
} /* namespace interval_management */
