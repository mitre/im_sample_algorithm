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

#include "imalgs/FIMSpeedLimiter.h"
#include "public/CustomMath.h"
#include "imalgs/IMUtils.h"

using namespace interval_management::open_source;

log4cplus::Logger FIMSpeedLimiter::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("FIMSpeedLimiter"));

const BoundedValue<double, 0, 100> FIMSpeedLimiter::DEFAULT_SPEED_DEVIATION_PERCENTAGE(10);

FIMSpeedLimiter::FIMSpeedLimiter(bool use_speed_quantization, bool use_speed_limiting,
                                 aaesim::open_source::bada_utils::FlapSpeeds flap_speeds,
                                 aaesim::open_source::bada_utils::FlightEnvelope flight_envelope,
                                 aaesim::open_source::bada_utils::Mass mass_data,
                                 aaesim::open_source::bada_utils::Aerodynamics aerodynamics,
                                 const FIMSpeedQuantizer &speed_quantizer)
   : m_rf_leg_limits(), m_active_filter_flag(0L), m_low_speed_coef(0), m_high_speed_coef(0) {
   m_limit_flag = use_speed_limiting;
   m_quantize_flag = use_speed_quantization;
   m_fim_quantizer = speed_quantizer;
   m_flap_speeds = flap_speeds;
   m_aircraft_flight_envelope = flight_envelope;
   m_aerodynamics = aerodynamics;
   m_mass_data = mass_data;
   SetMaxSpeedDeviationPercentage(DEFAULT_SPEED_DEVIATION_PERCENTAGE);
}

FIMSpeedLimiter::FIMSpeedLimiter() { SetMaxSpeedDeviationPercentage(DEFAULT_SPEED_DEVIATION_PERCENTAGE); }

FIMSpeedLimiter::~FIMSpeedLimiter() = default;

void FIMSpeedLimiter::Initialize(const bool pilot_delay_enabled) {
   m_fim_quantizer.SetFinalPhaseSpeedQuantizationValue(pilot_delay_enabled ? IMUtils::SPEED_QUANTIZE_1_DEFAULT_5_KNOTS
                                                                           : IMUtils::SPEED_QUANTIZE_1_DEFAULT_1_KNOT);
}

Units::Speed FIMSpeedLimiter::LimitSpeedCommand(
      const Units::Speed previous_ias_speed_command, const Units::Speed current_ias_speed_command,
      const Units::Speed nominal_reference_speed, const Units::Length distance_to_go_to_abp,
      const Units::Length distance_to_end_of_route, const Units::Length ownship_altitude,
      const aaesim::open_source::bada_utils::FlapConfiguration flap_configuration) {

   Units::Speed limitedspeed = current_ias_speed_command;
   Units::Speed llim = Units::ZERO_SPEED;
   Units::Speed hlim = Units::ZERO_SPEED;
   m_active_filter_flag = 0L;

   if (current_ias_speed_command < Units::MetersPerSecondSpeed(0.0000001)) {
      m_active_filter_flag = 1;
   }

   if (m_limit_flag) {
      llim = Units::MetersPerSecondSpeed(LowLimit(nominal_reference_speed));
      hlim = Units::MetersPerSecondSpeed(HighLimit(nominal_reference_speed));

      if (limitedspeed > hlim) {
         limitedspeed = hlim;
         m_active_filter_flag |= 4;
      } else if (limitedspeed < llim) {
         limitedspeed = llim;
         m_active_filter_flag |= 2;
      }

      Units::Speed rf_leg_upper_limit = GetRFLegSpeedLimit(distance_to_end_of_route);
      if (rf_leg_upper_limit > Units::zero()) {
         hlim = rf_leg_upper_limit;
         if (limitedspeed > hlim) {
            limitedspeed = hlim;
            m_active_filter_flag |= 1024;
         }
      }
   }

   Units::Speed resolved_threshold = Units::ZERO_SPEED;
   if (m_quantize_flag) {
      Units::Speed quantspeed = m_fim_quantizer.QuantizeForDistanceToGo(distance_to_go_to_abp, limitedspeed,
                                                                        previous_ias_speed_command, resolved_threshold);

      if (m_limit_flag) {
         if (quantspeed > hlim) {
            while (quantspeed > hlim) {
               quantspeed -= resolved_threshold;
            }
            m_active_filter_flag |= 16;
         } else if (quantspeed < llim) {
            while (quantspeed < llim) {
               quantspeed += resolved_threshold;
            }
            m_active_filter_flag |= 8;
         }

         if (quantspeed > hlim || quantspeed < llim) {
            m_active_filter_flag |= 32;
         }
      }

      limitedspeed = quantspeed;
   }

   if (limitedspeed > m_aircraft_flight_envelope.V_mo) {
      limitedspeed = m_aircraft_flight_envelope.V_mo;
      if (m_quantize_flag) {
         int ilimitspeed = Units::KnotsSpeed(limitedspeed).value();
         int iqthreshold = Units::KnotsSpeed(resolved_threshold).value();
         limitedspeed = Units::KnotsSpeed(ilimitspeed - (ilimitspeed % iqthreshold));
      }
      m_active_filter_flag |= 64;
   }

   if (limitedspeed < Units::KnotsSpeed(150.0)) {
      limitedspeed = Units::KnotsSpeed(150.0);
      m_active_filter_flag |= 128;
      if (m_quantize_flag) {
         int ilimitspeed = Units::KnotsSpeed(limitedspeed).value();
         int iqthreshold = Units::KnotsSpeed(resolved_threshold).value();
         if (ilimitspeed % iqthreshold != 0) {
            limitedspeed = Units::KnotsSpeed(ilimitspeed - (ilimitspeed % iqthreshold) + iqthreshold);
         }
      }
   }

   if (ownship_altitude <= Units::FeetLength(10000) && limitedspeed > Units::KnotsSpeed(250.0)) {
      limitedspeed = Units::KnotsSpeed(250.0);
      if (m_quantize_flag) {
         int ilimitspeed = Units::KnotsSpeed(limitedspeed).value();
         int iqthreshold = Units::KnotsSpeed(resolved_threshold).value();
         limitedspeed = Units::KnotsSpeed(ilimitspeed - (ilimitspeed % iqthreshold));
      }
      m_active_filter_flag |= 256;
   }

   if (flap_configuration > aaesim::open_source::bada_utils::FlapConfiguration::CRUISE) {
      if (flap_configuration == aaesim::open_source::bada_utils::FlapConfiguration::APPROACH &&
          limitedspeed > m_flap_speeds.cas_approach_maximum) {
         limitedspeed = m_flap_speeds.cas_approach_maximum;
         m_active_filter_flag |= 512;
      } else if (flap_configuration == aaesim::open_source::bada_utils::FlapConfiguration::LANDING &&
                 limitedspeed > m_flap_speeds.cas_landing_maximum) {
         limitedspeed = m_flap_speeds.cas_landing_maximum;
         m_active_filter_flag |= 512;
      } else if (flap_configuration == aaesim::open_source::bada_utils::FlapConfiguration::GEAR_DOWN &&
                 limitedspeed > m_flap_speeds.cas_gear_out_maximum) {
         limitedspeed = m_flap_speeds.cas_gear_out_maximum;
         m_active_filter_flag |= 512;
      }

      if (m_quantize_flag) {
         // Conversion to Knots from Meters Per Second can cause a value that had previously
         // been quantized to return a lower, non-quantized value.  See issue AAES-852.
         int ilimitspeed = Units::KnotsSpeed(limitedspeed).value() + 1E-10;
         int iqthreshold = Units::KnotsSpeed(resolved_threshold).value();
         limitedspeed = Units::KnotsSpeed(ilimitspeed - (ilimitspeed % iqthreshold));
      }
   }

   return limitedspeed;
}

BoundedValue<double, 0, 2> FIMSpeedLimiter::LimitMachCommand(
      const BoundedValue<double, 0, 2> &previous_reference_speed_command_mach,
      const BoundedValue<double, 0, 2> &current_mach_command, const BoundedValue<double, 0, 2> &nominal_mach,
      const Units::Mass &current_mass, const Units::Length &current_ownship_altitude,
      const aaesim::open_source::WeatherPrediction &weather_prediction) {

   BoundedValue<double, 0, 2> limited_mach = current_mach_command;
   BoundedValue<double, 0, 2> maximum_mach(m_aircraft_flight_envelope.M_mo);
   m_active_filter_flag = 0L;

   if (current_mach_command > maximum_mach) {
      limited_mach = maximum_mach;
      m_active_filter_flag |= 64;
   }

   const double mass_fraction = current_mass / m_mass_data.m_ref;
   const Units::Speed casmin = 1.3 * m_aerodynamics.cruise.V_stall * sqrt(mass_fraction);
   const Units::Speed tasmin = weather_prediction.getAtmosphere()->CAS2TAS(casmin, current_ownship_altitude);
   const BoundedValue<double, 0, 2> minimum_mach = BoundedValue<double, 0, 2>(
         Units::MetersPerSecondSpeed(tasmin).value() /
         sqrt(GAMMA * R.value() *
              weather_prediction.getAtmosphere()->GetTemperature(current_ownship_altitude).value()));

   double mach(nominal_mach);
   BoundedValue<double, 0, 2> llim(LowLimit(mach));
   BoundedValue<double, 0, 2> hlim(HighLimit(mach));

   if (m_limit_flag) {
      if (limited_mach < std::max(minimum_mach, llim)) {
         limited_mach = std::max(minimum_mach, llim);
         m_active_filter_flag |= 2;
      } else if (limited_mach > hlim) {
         limited_mach = hlim;
         m_active_filter_flag |= 4;
      }
   }

   if (m_quantize_flag) {
      limited_mach = m_fim_quantizer.QuantizeMach(previous_reference_speed_command_mach, limited_mach);
   }

   return limited_mach;
}

const Units::Speed FIMSpeedLimiter::GetRFLegSpeedLimit(const Units::Length dtg_to_ptp) const {
   if (m_rf_leg_limits.size() <= 1) {
      return Units::zero();
   }

   for (auto i = 1; i < m_rf_leg_limits.size(); ++i) {
      if (dtg_to_ptp < m_rf_leg_limits[i].distance_to_go) {
         if (m_rf_leg_limits[i].upper_ias_limit < Units::MetersPerSecondSpeed(1)) {
            return Units::zero();
         }

         return m_rf_leg_limits[i - 1].upper_ias_limit +
                ((dtg_to_ptp - m_rf_leg_limits[i - 1].distance_to_go) /
                 (m_rf_leg_limits[i].distance_to_go - m_rf_leg_limits[i - 1].distance_to_go)) *
                      (m_rf_leg_limits[i].upper_ias_limit - m_rf_leg_limits[i - 1].upper_ias_limit);
      }
   }
   return Units::zero();
}

void FIMSpeedLimiter::AddRfLegSpeedLimit(const Units::Length distance_to_go, const Units::Speed upper_ias_limit) {
   RfLegLimit rf_leg_limit(distance_to_go, upper_ias_limit);
   m_rf_leg_limits.push_back(rf_leg_limit);
}