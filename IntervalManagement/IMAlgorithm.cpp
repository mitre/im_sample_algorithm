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
#include "imalgs/IMAlgorithm.h"
#include "math/CustomMath.h"
#include "public/AircraftCalculations.h"

log4cplus::Logger IMAlgorithm::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("IMAlgorithm"));

const double IMAlgorithm::DEFAULT_SPEED_DEVIATION_PERCENTAGE = 10;

const Units::SecondsTime IMAlgorithm::ASSIGNED_SPACING_GOAL_DEFAULT(-INFINITY);
const Units::NauticalMilesLength IMAlgorithm::ERROR_DISTANCE_DEFAULT(0);
const Units::NauticalMilesLength IMAlgorithm::DIST_QUANTIZE_1_DEFAULT(0);
const Units::NauticalMilesLength IMAlgorithm::DIST_QUANTIZE_2_DEFAULT(10);
Units::KnotsSpeed IMAlgorithm::SPEED_QUANTIZE_1_DEFAULT(1);
const Units::KnotsSpeed IMAlgorithm::SPEED_QUANTIZE_2_DEFAULT(5);
const Units::KnotsSpeed IMAlgorithm::SPEED_QUANTIZE_3_DEFAULT(10);
const bool IMAlgorithm::LIMIT_FLAG_DEFAULT(true);
const bool IMAlgorithm::QUANTIZE_FLAG_DEFAULT(true);

const Units::SecondsPerNauticalMileInvertedSpeed IMAlgorithm::SLOPE_DEFAULT(0.25);
const std::string IMAlgorithm::RESET_MSG(".\nUse CUSTOM clearance type to enable this parameter.");
const double IMAlgorithm::UNDEFINED_INTERVAL(-INFINITY);

IMAlgorithm::IMAlgorithm()
      : m_target_kinetic_trajectory_predictor(nullptr),
        m_ownship_kinetic_trajectory_predictor(nullptr),
        m_im_clearance() {
   m_middle_to_final_quantize_transition_distance = DIST_QUANTIZE_1_DEFAULT;
   m_first_to_middle_quantize_transition_distance = DIST_QUANTIZE_2_DEFAULT;

   m_speed_quantize_final_phase = SPEED_QUANTIZE_1_DEFAULT;
   m_speed_quantize_middle_phase = SPEED_QUANTIZE_2_DEFAULT;
   m_speed_quantize_first_phase = SPEED_QUANTIZE_3_DEFAULT;

   m_error_distance = ERROR_DISTANCE_DEFAULT;
   m_slope = SLOPE_DEFAULT;

   m_active_filter_flag = 0L;

   m_limit_flag = LIMIT_FLAG_DEFAULT;
   m_quantize_flag = QUANTIZE_FLAG_DEFAULT;

   m_loaded = false;
   m_is_guidance_valid = false;

   m_ownship_kinematic_dtg_to_ptp = Units::Infinity();
   m_ownship_kinetic_dtg_to_ptp = Units::Infinity();

   SetMaxSpeedDeviationPercentage(DEFAULT_SPEED_DEVIATION_PERCENTAGE);
   IterClearIMAlg();
}

IMAlgorithm::IMAlgorithm(const IMAlgorithm &obj) {
   m_target_kinetic_trajectory_predictor = NULL;
   m_ownship_kinetic_trajectory_predictor = NULL;
   m_total_number_of_im_speed_changes = 0;
   m_limit_flag = false;
   m_quantize_flag = false;
   m_target_reference_lookup_index = 0;
   m_ownship_reference_lookup_index = 0;
   m_reference_precalc_index = 0;
   m_previous_reference_im_speed_command_mach = 0.0;
   m_loaded = false;
   m_im_operation_is_complete = false;
   m_is_guidance_valid = false;

   m_ownship_kinematic_dtg_to_ptp = Units::Infinity();
   m_ownship_kinetic_dtg_to_ptp = Units::Infinity();

   Copy(obj);
}

IMAlgorithm::~IMAlgorithm() = default;

IMAlgorithm &IMAlgorithm::operator=(const IMAlgorithm &obj) {
   if (this != &obj) {
      Copy(obj);
   }

   return *this;
}

void IMAlgorithm::Copy(const IMAlgorithm &obj) {
   m_total_number_of_im_speed_changes = obj.m_total_number_of_im_speed_changes;

   m_pilot_delay = obj.m_pilot_delay;
   m_tangent_plane_sequence = obj.m_tangent_plane_sequence;

   m_middle_to_final_quantize_transition_distance = obj.m_middle_to_final_quantize_transition_distance;
   m_first_to_middle_quantize_transition_distance = obj.m_first_to_middle_quantize_transition_distance;

   m_speed_quantize_final_phase = obj.m_speed_quantize_final_phase;
   m_speed_quantize_middle_phase = obj.m_speed_quantize_middle_phase;
   m_speed_quantize_first_phase = obj.m_speed_quantize_first_phase;

   m_slope = obj.m_slope;
   m_error_distance = obj.m_error_distance;

   m_limit_flag = obj.m_limit_flag;
   m_quantize_flag = obj.m_quantize_flag;

   m_previous_reference_im_speed_command_tas = obj.m_previous_reference_im_speed_command_tas;
   m_previous_im_speed_command_ias = obj.m_previous_im_speed_command_ias;
   m_previous_reference_im_speed_command_mach = obj.m_previous_reference_im_speed_command_mach;

   m_target_reference_lookup_index = obj.m_target_reference_lookup_index;
   m_ownship_reference_lookup_index = obj.m_ownship_reference_lookup_index;
   m_reference_precalc_index = obj.m_reference_precalc_index;

   m_ownship_kinetic_trajectory_predictor = obj.m_ownship_kinetic_trajectory_predictor;
   m_target_kinetic_trajectory_predictor = obj.m_target_kinetic_trajectory_predictor;

   m_target_aircraft_intent = obj.m_target_aircraft_intent;

   m_achieve_by_point = obj.m_achieve_by_point;

   m_weather_prediction = obj.m_weather_prediction;

   m_stage_of_im_operation = obj.m_stage_of_im_operation;

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

   m_rfleg_limits = obj.m_rfleg_limits;
   m_has_rf_leg = obj.m_has_rf_leg;

   m_is_guidance_valid = obj.m_is_guidance_valid;
   m_active_filter_flag = obj.m_active_filter_flag;
   m_target_kinetic_dtg_to_trp = obj.m_target_kinetic_dtg_to_trp;
   m_target_kinetic_dtg_to_last_waypoint = obj.m_target_kinetic_dtg_to_last_waypoint;
   m_target_kinematic_dtg_to_trp = obj.m_target_kinematic_dtg_to_trp;
   m_target_kinematic_dtg_to_last_waypoint = obj.m_target_kinematic_dtg_to_last_waypoint;
   m_ownship_kinetic_dtg_to_abp = obj.m_ownship_kinetic_dtg_to_abp;
   m_ownship_kinetic_dtg_to_ptp = obj.m_ownship_kinetic_dtg_to_ptp;
   m_ownship_kinematic_dtg_to_ptp = obj.m_ownship_kinematic_dtg_to_ptp;

   m_high_speed_coef = obj.m_high_speed_coef;
   m_low_speed_coef = obj.m_low_speed_coef;
}

void IMAlgorithm::Initialize(const KineticTrajectoryPredictor &ownship_kinetic_trajectory_predictor,
                             const KineticTrajectoryPredictor &target_kinetic_trajectory_predictor,
                             std::shared_ptr<TangentPlaneSequence> tangent_plane_sequence,
                             AircraftIntent &target_aircraft_intent,
                             const IMClearance &im_clearance,
                             const std::string &achieve_by_point,
                             WeatherPrediction &weather_prediction) {
   m_im_operation_is_complete = false;
   m_ownship_kinematic_dtg_to_ptp = Units::Infinity();

   SetAssignedSpacingGoal(im_clearance);
   m_im_clearance = im_clearance;
   m_target_aircraft_intent = target_aircraft_intent;
   m_achieve_by_point.assign(achieve_by_point);
   SetKineticTrajectoryPredictors(ownship_kinetic_trajectory_predictor, target_kinetic_trajectory_predictor);
   SetTangentPlaneSequence(tangent_plane_sequence);
   SetWeatherPrediction(weather_prediction);
   if (im_clearance.GetClearanceType() != IMClearance::CUSTOM) {
      ResetDefaults();
   }
   m_has_maintain_stage = !im_clearance.AbpAndPtpAreColocated();

}

void IMAlgorithm::ResetDefaults() {

   if (m_limit_flag != LIMIT_FLAG_DEFAULT) {
      LOG4CPLUS_WARN(m_logger, "mLimitFlag reset to " << LIMIT_FLAG_DEFAULT << RESET_MSG);
      m_limit_flag = LIMIT_FLAG_DEFAULT;
   }

   if (m_middle_to_final_quantize_transition_distance != DIST_QUANTIZE_1_DEFAULT) {
      LOG4CPLUS_WARN(m_logger, "mDistQuantize1 reset to " << DIST_QUANTIZE_1_DEFAULT << RESET_MSG);
      m_middle_to_final_quantize_transition_distance = DIST_QUANTIZE_1_DEFAULT;
   }

   if (m_first_to_middle_quantize_transition_distance != DIST_QUANTIZE_2_DEFAULT) {
      LOG4CPLUS_WARN(m_logger, "mDistQuantize2 reset to " << DIST_QUANTIZE_2_DEFAULT << RESET_MSG);
      m_first_to_middle_quantize_transition_distance = DIST_QUANTIZE_2_DEFAULT;
   }

   if (m_pilot_delay.IsPilotDelayOn()) {
      SPEED_QUANTIZE_1_DEFAULT = Units::KnotsSpeed(5);
   } else {
      SPEED_QUANTIZE_1_DEFAULT = Units::KnotsSpeed(1);
   }

   if (m_speed_quantize_final_phase != SPEED_QUANTIZE_1_DEFAULT) {
      LOG4CPLUS_WARN(m_logger, "mSpeedQuantize1 reset to " << SPEED_QUANTIZE_1_DEFAULT << RESET_MSG);
      m_speed_quantize_final_phase = SPEED_QUANTIZE_1_DEFAULT;
   }

   if (m_speed_quantize_middle_phase != SPEED_QUANTIZE_2_DEFAULT) {
      LOG4CPLUS_WARN(m_logger, "mSpeedQuantize2 reset to " << SPEED_QUANTIZE_2_DEFAULT << RESET_MSG);
      m_speed_quantize_middle_phase = SPEED_QUANTIZE_2_DEFAULT;
   }

   if (m_speed_quantize_first_phase != SPEED_QUANTIZE_3_DEFAULT) {
      LOG4CPLUS_WARN(m_logger, "mSpeedQuantize3 reset to " << SPEED_QUANTIZE_3_DEFAULT << RESET_MSG);
      m_speed_quantize_first_phase = SPEED_QUANTIZE_3_DEFAULT;
   }

   if (m_error_distance != ERROR_DISTANCE_DEFAULT) {
      LOG4CPLUS_WARN(m_logger, "mErrorDistance reset to " << ERROR_DISTANCE_DEFAULT << RESET_MSG);
      m_error_distance = ERROR_DISTANCE_DEFAULT;
   }

   if (m_slope != SLOPE_DEFAULT) {
      LOG4CPLUS_WARN(m_logger, "mSlope reset to " << SLOPE_DEFAULT << RESET_MSG);
      m_slope = SLOPE_DEFAULT;
   }

   if (m_quantize_flag != QUANTIZE_FLAG_DEFAULT) {
      LOG4CPLUS_WARN(m_logger, "mQuantizeFlag reset to " << QUANTIZE_FLAG_DEFAULT << RESET_MSG);
      m_quantize_flag = QUANTIZE_FLAG_DEFAULT;
   }

   if (m_high_speed_coef != 1 + DEFAULT_SPEED_DEVIATION_PERCENTAGE * 0.01) {
      LOG4CPLUS_WARN(m_logger, "max_speed_deviation_percentage reset to " << DEFAULT_SPEED_DEVIATION_PERCENTAGE << RESET_MSG);
      SetMaxSpeedDeviationPercentage(DEFAULT_SPEED_DEVIATION_PERCENTAGE);
   }
}

Guidance IMAlgorithm::Update(const Guidance &prevguidance,
                             const DynamicsState &dynamicsstate,
                             const AircraftState &owntruthstate,
                             const AircraftState &targettruthstate,
                             const vector<AircraftState> &targethistory) {
   throw std::runtime_error("Implementation error. You should not be here!");

   return prevguidance;
}

void IMAlgorithm::SetPilotDelay(const bool pilot_delay_on,
                                const Units::Time pilot_delay_mean,
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
   m_ownship_kinetic_dtg_to_ptp = Units::Infinity();
   m_ownship_kinetic_dtg_to_abp = Units::Infinity();

   m_target_kinematic_dtg_to_last_waypoint = Units::Infinity();
   m_target_kinematic_dtg_to_trp = Units::Infinity();
   m_target_kinetic_dtg_to_last_waypoint = Units::Infinity();
   m_target_kinetic_dtg_to_trp = Units::Infinity();
   m_target_reference_altitude = Units::zero();

   m_unmodified_im_speed_command_ias = Units::MetersPerSecondSpeed(-1.0);
   m_im_speed_command_ias = Units::MetersPerSecondSpeed(-1.0);
   m_im_speed_command_with_pilot_delay = Units::MetersPerSecondSpeed(-1.0);

   m_im_clearance = IMClearance();
}

void IMAlgorithm::IterationReset() {
   IterClearIMAlg();
}

void IMAlgorithm::UpdatePositionMetrics(const AircraftState &ownship_aircraft_state,
                                        const AircraftState &target_aircraft_state) {
   if (target_aircraft_state.m_id != IMUtils::UNINITIALIZED_AIRCRAFT_ID) {
      MergePointMetric &merge_point_metric =
            InternalObserver::getInstance()->GetMergePointMetric(ownship_aircraft_state.m_id);
      if (merge_point_metric.mergePointFound()) {
         merge_point_metric.update(
               ownship_aircraft_state.m_x, ownship_aircraft_state.m_y, target_aircraft_state.m_x,
               target_aircraft_state.m_y);
      }
      InternalObserver::getInstance()->GetClosestPointMetric(ownship_aircraft_state.m_id).update(
            ownship_aircraft_state.m_x, ownship_aircraft_state.m_y, target_aircraft_state.m_x,
            target_aircraft_state.m_y);
   }
}

Units::Speed IMAlgorithm::QuantizeThreshold(const Units::Length dtg_to_abp_in,
                                            const Units::Speed computed_ias_command,
                                            const Units::Speed old_ias_command,
                                            Units::Speed &threshold) {
   Units::Length dtg_to_abp = dtg_to_abp_in;

   if (dtg_to_abp < Units::ZERO_LENGTH) {
      dtg_to_abp = Units::ZERO_LENGTH;
   }

   Units::Speed result = computed_ias_command;
   Units::Speed quantizethreshold = Units::MetersPerSecondSpeed(0.0);

   if (dtg_to_abp < m_middle_to_final_quantize_transition_distance) {
      quantizethreshold = m_speed_quantize_final_phase;
   } else if (dtg_to_abp < m_first_to_middle_quantize_transition_distance) {
      quantizethreshold = m_speed_quantize_middle_phase;
   } else {
      quantizethreshold = m_speed_quantize_first_phase;
   }

   if (abs(computed_ias_command - old_ias_command) < (quantizethreshold * 0.75)) {
      result = old_ias_command;
   }

   result = quantize(result, quantizethreshold);
   threshold = quantizethreshold;

   return result;
}

Units::Speed IMAlgorithm::QuantizeThreshold(const Units::Length dtg_to_abp_in,
                                            const Units::Speed computed_ias_command,
                                            const Units::Speed old_ias_command) {
   Units::Length dtg_to_abp = dtg_to_abp_in;
   if (dtg_to_abp < Units::ZERO_LENGTH) {
      dtg_to_abp = Units::ZERO_LENGTH;
   }

   Units::Speed result = computed_ias_command;
   Units::Speed quantizethreshold = Units::MetersPerSecondSpeed(0.0);

   if (dtg_to_abp < m_middle_to_final_quantize_transition_distance) {
      quantizethreshold = m_speed_quantize_final_phase;
   } else if (dtg_to_abp < m_first_to_middle_quantize_transition_distance) {
      quantizethreshold = m_speed_quantize_middle_phase;
   } else {
      quantizethreshold = m_speed_quantize_first_phase;
   }

   if (abs(computed_ias_command - old_ias_command) < (quantizethreshold * 0.75)) {
      result = old_ias_command;
   }

   result = quantize(result, quantizethreshold);

   return result;
}

Units::Speed IMAlgorithm::LimitImSpeedCommand(const Units::Speed im_speed_command_ias,
                                              const double reference_velocity_mps,
                                              const Units::Length distance_to_go_to_abp,
                                              const BadaWithCalc &bada_with_calc,
                                              const Units::Length ownship_altitude,
                                              const int flap_configuration,
                                              const Units::Speed rf_upper_limit) {
   Units::Speed limitedspeed = im_speed_command_ias;
   Units::Speed quantspeed;
   Units::Speed llim = Units::ZERO_SPEED;
   Units::Speed hlim = Units::ZERO_SPEED;
   const Units::MetersPerSecondSpeed near_zero(0.0000001);

   m_active_filter_flag = 0L;

   if (im_speed_command_ias < near_zero) {
      m_active_filter_flag = 1;
   }

   if (m_limit_flag) {
      llim = Units::MetersPerSecondSpeed(LowLimit(reference_velocity_mps));
      hlim = Units::MetersPerSecondSpeed(HighLimit(reference_velocity_mps));

      if (limitedspeed > hlim) {
         limitedspeed = hlim;
         m_active_filter_flag |= 4;
      } else if (limitedspeed < llim) {
         limitedspeed = llim;
         m_active_filter_flag |= 2;
      }

      if (rf_upper_limit > Units::zero()) {
         hlim = rf_upper_limit;
         if (limitedspeed > hlim) {
            limitedspeed = hlim;
            m_active_filter_flag |= 1024;
         }
      }
   }

   Units::Speed qThreshold = Units::ZERO_SPEED;
   if (m_quantize_flag) {
      quantspeed = QuantizeThreshold(distance_to_go_to_abp, limitedspeed, m_previous_im_speed_command_ias, qThreshold);

      if (m_limit_flag) {
         if (quantspeed > hlim) {
            while (quantspeed > hlim) {
               quantspeed -= qThreshold;
            }
            m_active_filter_flag |= 16;
         } else if (quantspeed < llim) {
            while (quantspeed < llim) {
               quantspeed += qThreshold;
            }
            m_active_filter_flag |= 8;
         }

         if (quantspeed > hlim || quantspeed < llim) {
            m_active_filter_flag |= 32;
            LOG4CPLUS_WARN(m_logger, "quantized speed out of limits:\n"
                  << "  QuantSpeed: " << Units::KnotsSpeed(quantspeed).value() << " llim: "
                  << Units::KnotsSpeed(llim).value() << ", hlim: " << Units::KnotsSpeed(hlim).value());
         }
      }

      limitedspeed = quantspeed;
   }

   if (limitedspeed > bada_with_calc.flight_envelope.V_mo) {
      limitedspeed = bada_with_calc.flight_envelope.V_mo;
      if (m_quantize_flag) {
         int ilimitspeed = Units::KnotsSpeed(limitedspeed).value();
         int iqthreshold = Units::KnotsSpeed(qThreshold).value();
         limitedspeed = Units::KnotsSpeed(ilimitspeed - (ilimitspeed % iqthreshold));
      }
      m_active_filter_flag |= 64;
   }

   if (limitedspeed < Units::KnotsSpeed(150.0)) {
      limitedspeed = Units::KnotsSpeed(150.0);
      m_active_filter_flag |= 128;
      if (m_quantize_flag) {
         int ilimitspeed = Units::KnotsSpeed(limitedspeed).value();
         int iqthreshold = Units::KnotsSpeed(qThreshold).value();
         if (ilimitspeed % iqthreshold != 0) {
            limitedspeed = Units::KnotsSpeed(ilimitspeed - (ilimitspeed % iqthreshold) + iqthreshold);
         }
      }
   }

   if (ownship_altitude <= Units::FeetLength(10000) && limitedspeed > Units::KnotsSpeed(250.0)) {
      limitedspeed = Units::KnotsSpeed(250.0);
      if (m_quantize_flag) {
         int ilimitspeed = Units::KnotsSpeed(limitedspeed).value();
         int iqthreshold = Units::KnotsSpeed(qThreshold).value();
         limitedspeed = Units::KnotsSpeed(ilimitspeed - (ilimitspeed % iqthreshold));
      }
      m_active_filter_flag |= 256;
   }

   if (flap_configuration > 0) {
      if (flap_configuration == 1 && limitedspeed > bada_with_calc.mFlapSpeeds.VappMax) {
         limitedspeed = bada_with_calc.mFlapSpeeds.VappMax;
         m_active_filter_flag |= 512;
      } else if (flap_configuration == 2 && limitedspeed > bada_with_calc.mFlapSpeeds.VlndMax) {
         limitedspeed = bada_with_calc.mFlapSpeeds.VlndMax;
         m_active_filter_flag |= 512;
      } else if (flap_configuration == 3 && limitedspeed > bada_with_calc.mFlapSpeeds.VgearMax) {
         limitedspeed = bada_with_calc.mFlapSpeeds.VgearMax;
         m_active_filter_flag |= 512;
      }

      if (m_quantize_flag) {
         // Conversion to Knots from Meters Per Second can cause a value that had previously
         // been quantized to return a lower, non-quantized value.  See issue AAES-852.
         int ilimitspeed = Units::KnotsSpeed(limitedspeed).value() + 1E-10;
         int iqthreshold = Units::KnotsSpeed(qThreshold).value();
         limitedspeed = Units::KnotsSpeed(ilimitspeed - (ilimitspeed % iqthreshold));
      }
   }

   LOG4CPLUS_TRACE(m_logger,
                   "Quantized speed is " << std::setprecision(12) << Units::KnotsSpeed(quantspeed) << " and limiter is " << m_active_filter_flag);

   return limitedspeed;
}

double IMAlgorithm::LimitImMachCommand(double estimated_mach, const double nominal_mach,
      const BadaWithCalc &bada_calculator, const Units::Length current_ownship_altitude) {

   m_active_filter_flag = 0L;

   if (estimated_mach > bada_calculator.flight_envelope.M_mo) {
      estimated_mach = bada_calculator.flight_envelope.M_mo;
      m_active_filter_flag |= 64;
   }

   const double massterm = bada_calculator.mAircraftMass / bada_calculator.mass.m_ref;
   const Units::Speed casmin = 1.3 * bada_calculator.aerodynamics.cruise.V_stall * sqrt(massterm);
   const Units::Speed tasmin =
         m_weather_prediction.getAtmosphere()->CAS2TAS(casmin, current_ownship_altitude);
   const double minimum_mach = Units::MetersPerSecondSpeed(tasmin).value() /
                               sqrt(GAMMA * R.value() * m_weather_prediction.getAtmosphere()->GetTemperature(
                                     current_ownship_altitude).value());

   double llim = LowLimit(nominal_mach);
   double hlim = HighLimit(nominal_mach);

   if (m_limit_flag) {
      if (estimated_mach < std::max(minimum_mach, llim)) {
         estimated_mach = std::max(minimum_mach, llim);
         m_active_filter_flag |= 2;
      } else if (estimated_mach > hlim) {
         estimated_mach = hlim;
         m_active_filter_flag |= 4;
      }
   }
   if (m_quantize_flag) {
      estimated_mach = MachHysteresis(estimated_mach, m_previous_reference_im_speed_command_mach);
      estimated_mach = quantize(estimated_mach, 0.02);
   }

   return estimated_mach;
}

const Units::Speed IMAlgorithm::GetRFLegSpeedLimit(Units::Length dtg_to_ptp) const
{
   if (m_rfleg_limits.size() <= 1) {
      return Units::zero();
   }

   for (unsigned int i = 1; i < m_rfleg_limits.size(); i++) {
      if (dtg_to_ptp < m_rfleg_limits[i].first) {
         if (m_rfleg_limits[i].second < Units::MetersPerSecondSpeed(1)) {
            return Units::zero();
         }
         return m_rfleg_limits[i-1].second +
                ((dtg_to_ptp - m_rfleg_limits[i-1].first) / (m_rfleg_limits[i].first - m_rfleg_limits[i-1].first)) *
                (m_rfleg_limits[i].second - m_rfleg_limits[i-1].second);
      }
   }
   return Units::zero();
}

void IMAlgorithm::DumpParameters(const std::string &parameters_to_print) {
   LOG4CPLUS_DEBUG(IMAlgorithm::m_logger, "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl);

   LOG4CPLUS_DEBUG(IMAlgorithm::m_logger,
                   "IMAlgorithm parms for " << parameters_to_print.c_str() << std::endl << std::endl);

   LOG4CPLUS_DEBUG(IMAlgorithm::m_logger, "mLoaded " << m_loaded << std::endl);

   LOG4CPLUS_DEBUG(IMAlgorithm::m_logger, "mSpeedChangeCount " << m_total_number_of_im_speed_changes << std::endl);

   LOG4CPLUS_DEBUG(IMAlgorithm::m_logger, "limit flag " << m_limit_flag << std::endl);
   LOG4CPLUS_DEBUG(IMAlgorithm::m_logger, "quantize flag " << m_quantize_flag << std::endl);

   LOG4CPLUS_DEBUG(IMAlgorithm::m_logger,
                   "mDistQuantize1   "
                         << Units::NauticalMilesLength(m_middle_to_final_quantize_transition_distance).value()
                         << std::endl);
   LOG4CPLUS_DEBUG(IMAlgorithm::m_logger,
                   "mDistQuantize2   "
                         << Units::NauticalMilesLength(m_first_to_middle_quantize_transition_distance).value()
                         << std::endl);

   LOG4CPLUS_DEBUG(IMAlgorithm::m_logger,
                   "mSpeedQuantize1  " << Units::KnotsSpeed(m_speed_quantize_final_phase).value() << std::endl);
   LOG4CPLUS_DEBUG(IMAlgorithm::m_logger,
                   "mSpeedQuantize2  " << Units::KnotsSpeed(m_speed_quantize_middle_phase).value() << std::endl);
   LOG4CPLUS_DEBUG(IMAlgorithm::m_logger,
                   "mSpeedQuantize3  " << Units::KnotsSpeed(m_speed_quantize_first_phase).value() << std::endl);

   LOG4CPLUS_DEBUG(IMAlgorithm::m_logger,
                   "slope " << Units::SecondsPerNauticalMileInvertedSpeed(m_slope).value() << std::endl);
   LOG4CPLUS_DEBUG(IMAlgorithm::m_logger,
                   "error distance " << Units::NauticalMilesLength(m_error_distance).value() << std::endl);

   LOG4CPLUS_DEBUG(IMAlgorithm::m_logger,
                   "ownship kinetic achieve waypoint " << m_achieve_by_point.c_str() << std::endl);

   m_ownship_kinetic_trajectory_predictor->GetAircraftIntent().DumpParms("mOwnIntent");
   m_target_kinetic_trajectory_predictor->GetAircraftIntent().DumpParms("mTargetIntent");

   m_pilot_delay.DumpParameters("mPilotDelay");

   LOG4CPLUS_DEBUG(IMAlgorithm::m_logger, "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl);
}


void IMAlgorithm::SetMaxSpeedDeviationPercentage(const double max_speed_deviation_percentage) {
   m_low_speed_coef = 1 - max_speed_deviation_percentage * .01;
   m_high_speed_coef = 1 + max_speed_deviation_percentage * .01;
}
