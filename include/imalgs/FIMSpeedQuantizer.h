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

#include "utility/BoundedValue.h"
#include "public/Logging.h"
#include "scalar/Speed.h"
#include "scalar/Length.h"

namespace interval_management {
namespace open_source {

class FIMSpeedQuantizer {
  public:
   FIMSpeedQuantizer();

   FIMSpeedQuantizer(Units::Length middle_to_final_quantize_transition_distance,
                     Units::Length first_to_middle_quantize_transition_distance,
                     Units::Speed speed_quantize_final_phase, Units::Speed speed_quantize_middle_phase,
                     Units::Speed speed_quantize_first_phase);

   ~FIMSpeedQuantizer() = default;

   Units::Speed QuantizeForDistanceToGo(const Units::Length dtg_to_abp_in, const Units::Speed computed_ias_command,
                                        const Units::Speed old_ias_command, Units::Speed &threshold) const;

   BoundedValue<double, 0, 2> QuantizeMach(BoundedValue<double, 0, 2> reference_mach,
                                           BoundedValue<double, 0, 2> command_mach) const;

   Units::Length GetFirstToMiddleQuantizationTransitionDistance() const;

   Units::Length GetMiddleToFinalQuantizationTransitionDistance() const;

   Units::Speed GetFinalPhaseSpeedQuantizationValue() const;

   Units::Speed GetMiddlePhaseSpeedQuantizationValue() const;

   Units::Speed GetFirstPhaseSpeedQuantizationValue() const;

   void SetFinalPhaseSpeedQuantizationValue(const Units::Speed new_value);

  private:
   static log4cplus::Logger m_logger;
   static double FIM_MACH_QUANTIZE_VALUE;
   static double FIM_HYSTERESIS_PERCENTAGE;

   BoundedValue<double, 0, 2> MachHysteresis(const BoundedValue<double, 0, 2> &newmach,
                                             const BoundedValue<double, 0, 2> &oldmach) const;

   Units::Length m_middle_to_final_quantize_transition_distance;
   Units::Length m_first_to_middle_quantize_transition_distance;
   Units::Speed m_speed_quantize_final_phase;
   Units::Speed m_speed_quantize_middle_phase;
   Units::Speed m_speed_quantize_first_phase;
};

inline void FIMSpeedQuantizer::SetFinalPhaseSpeedQuantizationValue(const Units::Speed new_value) {
   m_speed_quantize_final_phase = new_value;
}

inline Units::Speed FIMSpeedQuantizer::GetFinalPhaseSpeedQuantizationValue() const {
   return m_speed_quantize_final_phase;
}

inline Units::Speed FIMSpeedQuantizer::GetMiddlePhaseSpeedQuantizationValue() const {
   return m_speed_quantize_middle_phase;
}

inline Units::Speed FIMSpeedQuantizer::GetFirstPhaseSpeedQuantizationValue() const {
   return m_speed_quantize_first_phase;
}

inline Units::Length FIMSpeedQuantizer::GetMiddleToFinalQuantizationTransitionDistance() const {
   return m_middle_to_final_quantize_transition_distance;
}

inline Units::Length FIMSpeedQuantizer::GetFirstToMiddleQuantizationTransitionDistance() const {
   return m_first_to_middle_quantize_transition_distance;
}

}  // namespace open_source
}  // namespace interval_management