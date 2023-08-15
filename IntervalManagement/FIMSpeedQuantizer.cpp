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

#include "imalgs/FIMSpeedQuantizer.h"
#include "utility/CustomUnits.h"
#include "public/CustomMath.h"
#include "imalgs/IMUtils.h"

using namespace interval_management::open_source;

log4cplus::Logger FIMSpeedQuantizer::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("FIMSpeedQuantizer"));

double FIMSpeedQuantizer::FIM_MACH_QUANTIZE_VALUE(0.02);
double FIMSpeedQuantizer::FIM_HYSTERESIS_PERCENTAGE(0.75);

FIMSpeedQuantizer::FIMSpeedQuantizer()
   : m_middle_to_final_quantize_transition_distance(IMUtils::DIST_QUANTIZE_1_DEFAULT),
     m_first_to_middle_quantize_transition_distance(IMUtils::DIST_QUANTIZE_2_DEFAULT),
     m_speed_quantize_final_phase(IMUtils::SPEED_QUANTIZE_1_DEFAULT_1_KNOT),
     m_speed_quantize_middle_phase(IMUtils::SPEED_QUANTIZE_2_DEFAULT),
     m_speed_quantize_first_phase(IMUtils::SPEED_QUANTIZE_3_DEFAULT){};

FIMSpeedQuantizer::FIMSpeedQuantizer(Units::Length middle_to_final_quantize_transition_distance,
                                     Units::Length first_to_middle_quantize_transition_distance,
                                     Units::Speed speed_quantize_final_phase, Units::Speed speed_quantize_middle_phase,
                                     Units::Speed speed_quantize_first_phase) {
   m_middle_to_final_quantize_transition_distance = middle_to_final_quantize_transition_distance;
   m_first_to_middle_quantize_transition_distance = first_to_middle_quantize_transition_distance;
   m_speed_quantize_final_phase = speed_quantize_final_phase;
   m_speed_quantize_middle_phase = speed_quantize_middle_phase;
   m_speed_quantize_first_phase = speed_quantize_first_phase;
}

Units::Speed FIMSpeedQuantizer::QuantizeForDistanceToGo(const Units::Length dtg_to_abp_in,
                                                        const Units::Speed computed_ias_command,
                                                        const Units::Speed previous_ias_command,
                                                        Units::Speed &threshold) const {
   Units::Length dtg_to_abp = dtg_to_abp_in;

   if (dtg_to_abp < Units::ZERO_LENGTH) {
      dtg_to_abp = Units::ZERO_LENGTH;
   }

   Units::Speed quantized_ias_command = computed_ias_command;
   Units::Speed quantizethreshold = Units::MetersPerSecondSpeed(0.0);

   if (dtg_to_abp < m_middle_to_final_quantize_transition_distance) {
      quantizethreshold = m_speed_quantize_final_phase;
   } else if (dtg_to_abp < m_first_to_middle_quantize_transition_distance) {
      quantizethreshold = m_speed_quantize_middle_phase;
   } else {
      quantizethreshold = m_speed_quantize_first_phase;
   }

   if (abs(computed_ias_command - previous_ias_command) < (quantizethreshold * FIM_HYSTERESIS_PERCENTAGE)) {
      quantized_ias_command = previous_ias_command;
   }

   quantized_ias_command = quantize(quantized_ias_command, quantizethreshold);
   threshold = quantizethreshold;

   return quantized_ias_command;
}

BoundedValue<double, 0, 2> FIMSpeedQuantizer::QuantizeMach(BoundedValue<double, 0, 2> reference_mach,
                                                           BoundedValue<double, 0, 2> command_mach) const {
   return BoundedValue<double, 0, 2>(quantize(MachHysteresis(command_mach, reference_mach), FIM_MACH_QUANTIZE_VALUE));
}

BoundedValue<double, 0, 2> FIMSpeedQuantizer::MachHysteresis(const BoundedValue<double, 0, 2> &newmach,
                                                             const BoundedValue<double, 0, 2> &oldmach) const {
   if (fabs(newmach - oldmach) < FIM_MACH_QUANTIZE_VALUE * FIM_HYSTERESIS_PERCENTAGE) {
      return oldmach;
   }
   return newmach;
}
