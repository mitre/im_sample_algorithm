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
#include "scalar/Speed.h"
#include "scalar/Length.h"
#include "public/WeatherPrediction.h"
#include "imalgs/FIMSpeedQuantizer.h"
#include "public/BadaUtils.h"
#include "public/SpeedCommandLimiter.h"

namespace interval_management {
namespace open_source {
class FIMSpeedLimiter : public aaesim::open_source::SpeedCommandLimiter {
  public:
   struct RfLegLimit {
      RfLegLimit(Units::Length distance_to_go, Units::Speed upper_ias_limit)
         : distance_to_go(distance_to_go), upper_ias_limit(upper_ias_limit) {}

      Units::Length distance_to_go;
      Units::Speed upper_ias_limit;
   };

   // Values used in speed command limiting calculations.
   // Added and subtracted to 1 to compute the low and high
   // speed command limits.
   static const BoundedValue<double, 0, 100> DEFAULT_SPEED_DEVIATION_PERCENTAGE;

   FIMSpeedLimiter();

   FIMSpeedLimiter(bool use_speed_quantization, bool use_speed_limiting,
                   aaesim::open_source::bada_utils::FlapSpeeds flap_speeds,
                   aaesim::open_source::bada_utils::FlightEnvelope flight_envelope,
                   aaesim::open_source::bada_utils::Mass mass_data,
                   aaesim::open_source::bada_utils::Aerodynamics aerodynamics,
                   const FIMSpeedQuantizer &speed_quantizer);

   ~FIMSpeedLimiter();

   void Initialize(const bool pilot_delay_enabled);

   // If limiting in effect, this method will limit the command speed based on high and low limit.  The high and low
   // limits are computed from a trajectory speed and a fixed limiting factor.
   // Changes to these flags should be reflected also online:
   // https://huddle.mitre.org/pages/viewpage.action?pageId=29263788#TrajectoryFileFields-CommandSpeedLimitFlag
   Units::Speed LimitSpeedCommand(const Units::Speed previous_ias_speed_command,
                                  const Units::Speed current_ias_speed_command,
                                  const Units::Speed reference_velocity_mps, const Units::Length distance_to_go_to_abp,
                                  const Units::Length distance_to_go_to_ptp, const Units::Length ownship_altitude,
                                  const aaesim::open_source::bada_utils::FlapConfiguration flap_configuration) override;

   BoundedValue<double, 0, 2> LimitMachCommand(
         const BoundedValue<double, 0, 2> &previous_reference_speed_command_mach,
         const BoundedValue<double, 0, 2> &current_mach_command, const BoundedValue<double, 0, 2> &nominal_mach,
         const Units::Mass &current_mass, const Units::Length &current_ownship_altitude,
         const aaesim::open_source::WeatherPrediction &weather_prediction) override;

   const interval_management::open_source::FIMSpeedQuantizer &GetSpeedQuantizer() const;

   void SetLimitFlag(bool limit_flag);

   void SetQuantizeFlag(bool quantize_flag);

   unsigned long int GetActiveFilter() const;

   bool GetLimitFlag() const;

   bool GetQuantizeFlag() const;

   void SetActiveFilter(unsigned long flag);

   void SetMaxSpeedDeviationPercentage(const BoundedValue<double, 0, 100> max_speed_deviation_factor);

   const Units::Speed GetRFLegSpeedLimit(const Units::Length dtg_to_ptp) const;

   void AddRfLegSpeedLimit(const Units::Length distance_to_go, const Units::Speed upper_ias_limit);

   const std::vector<RfLegLimit> &GetRfLegSpeedLimits() const;

   void ClearRfLegSpeedLimits();

   template <class X>
   X LowLimit(const X val) const {
      return (val * m_low_speed_coef);
   };

   template <class X>
   X HighLimit(const X val) const {
      return (val * m_high_speed_coef);
   };

  private:
   static log4cplus::Logger m_logger;

   std::vector<RfLegLimit> m_rf_leg_limits;

   unsigned long int m_active_filter_flag;
   bool m_limit_flag;
   bool m_quantize_flag;
   double m_low_speed_coef;
   double m_high_speed_coef;
   FIMSpeedQuantizer m_fim_quantizer;
   aaesim::open_source::bada_utils::FlapSpeeds m_flap_speeds;
   aaesim::open_source::bada_utils::FlightEnvelope m_aircraft_flight_envelope;
   aaesim::open_source::bada_utils::Mass m_mass_data;
   aaesim::open_source::bada_utils::Aerodynamics m_aerodynamics;
};

inline const interval_management::open_source::FIMSpeedQuantizer &FIMSpeedLimiter::GetSpeedQuantizer() const {
   return m_fim_quantizer;
}

inline bool FIMSpeedLimiter::GetLimitFlag() const { return m_limit_flag; }

inline bool FIMSpeedLimiter::GetQuantizeFlag() const { return m_quantize_flag; }

inline unsigned long int FIMSpeedLimiter::GetActiveFilter() const { return m_active_filter_flag; }

inline void FIMSpeedLimiter::SetActiveFilter(unsigned long flag) { m_active_filter_flag = flag; }

inline void FIMSpeedLimiter::SetQuantizeFlag(bool quantize_flag) { m_quantize_flag = quantize_flag; }

inline void FIMSpeedLimiter::SetLimitFlag(bool limit_flag) { m_limit_flag = limit_flag; }

inline void FIMSpeedLimiter::SetMaxSpeedDeviationPercentage(
      const BoundedValue<double, 0, 100> max_speed_deviation_percentage) {
   m_low_speed_coef = 1 - max_speed_deviation_percentage * .01;
   m_high_speed_coef = 1 + max_speed_deviation_percentage * .01;
}

inline const std::vector<FIMSpeedLimiter::RfLegLimit> &FIMSpeedLimiter::GetRfLegSpeedLimits() const {
   return m_rf_leg_limits;
}

inline void FIMSpeedLimiter::ClearRfLegSpeedLimits() { m_rf_leg_limits.clear(); }

}  // namespace open_source
}  // namespace interval_management