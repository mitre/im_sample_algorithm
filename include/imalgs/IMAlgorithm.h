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

#pragma once

#include "imalgs/FIMConfiguration.h"
#include "public/Guidance.h"
#include "public/AircraftIntent.h"
#include "imalgs/AircraftState.h"
#include "utility/Logging.h"
#include "public/PilotDelay.h"
#include "public/ThreeDOFDynamics.h"
#include "public/TangentPlaneSequence.h"
#include <scalar/Time.h>
#include <scalar/Speed.h>
#include <scalar/Length.h>
#include <scalar/Frequency.h>
#include "public/BadaUtils.h"
#include "imalgs/IMClearance.h"
#include "imalgs/FIMSpeedLimiter.h"

namespace interval_management {
namespace open_source {

class IMAlgorithm {
  public:
   static const std::string RESET_MSG;
   static const double UNDEFINED_INTERVAL;

   enum FlightStage { UNSET = -1, NONE = 0, ACHIEVE = 1, MAINTAIN = 2 };

   struct OwnshipPredictionParameters {
      Units::Angle maximum_allowable_bank_angle;
      Units::Speed transition_ias;
      double transition_mach;
      Units::Length transition_altitude;
      Units::Length expected_cruise_altitude;
      aaesim::open_source::bada_utils::FlapSpeeds flap_speeds;
      aaesim::open_source::bada_utils::FlightEnvelope flight_envelope;
      aaesim::open_source::bada_utils::Mass mass_data;
      aaesim::open_source::bada_utils::Aerodynamics aerodynamics;
   };

   IMAlgorithm();

   IMAlgorithm(const IMAlgorithm &obj);

   virtual ~IMAlgorithm() = default;

   IMAlgorithm &operator=(const IMAlgorithm &obj);

   virtual void Initialize(const OwnshipPredictionParameters &ownship_prediction_parameters,
                           const AircraftIntent &ownship_aircraft_intent, WeatherPrediction &weather_prediction);

   // Called during initialization whenever the clearance type is not CUSTOM.
   virtual void ResetDefaults();

   virtual void IterationReset();

   virtual aaesim::open_source::Guidance Update(
         const aaesim::open_source::Guidance &prevguidance, const aaesim::open_source::DynamicsState &dynamicsstate,
         const interval_management::open_source::AircraftState &owntruthstate,
         const interval_management::open_source::AircraftState &targettruthstate,
         const std::vector<interval_management::open_source::AircraftState> &targethistory);

   virtual const double GetSpacingError() const = 0;  // Returned value has units stripped off.

   virtual const bool IsImOperationComplete() const;

   virtual const bool IsOwnshipPassedPtp() const;

   void UpdatePositionMetrics(const interval_management::open_source::AircraftState &ownship_aircraft_state,
                              const interval_management::open_source::AircraftState &target_aircraft_state);

   void SetPilotDelay(const bool pilot_delay_on, const Units::Time pilot_delay_mean,
                      const Units::Time pilot_delay_standard_deviation);

   void DisablePilotDelayModel();

   void EnableDefaultPilotDelayModel();

   void SetErrorDistance(Units::Length error_distance);

   void SetSlope(Units::InvertedSpeed slope);

   void SetLimitFlag(bool limit_flag);

   void SetQuantizeFlag(bool quantize_flag);

   void SetMaxSpeedDeviationPercentage(const BoundedValue<double, 0, 100> max_speed_deviation_factor);

   const Units::Speed GetFinalPhaseSpeedQuantizationValue() const;

   const Units::Speed GetMiddlePhaseSpeedQuantizationValue() const;

   const Units::Speed GetFirstPhaseSpeedQuantizationValue() const;

   const bool GetLimitFlag() const;

   const bool GetQuantizeFlag() const;

   const bool IsLoaded() const;

   const FlightStage GetFlightStage() const;

   const Units::Length GetOwnshipDtgtoAbp() const;

   const Units::Time GetOwnshipTtgtoAbp() const;

   const Units::Time GetOwnshipTtgtoPtp() const;

   const Units::Time GetTargetTtgToTrp() const;

   const Units::Time GetTargetTtgToEndOfRoute() const;

   const Units::Time GetTargetTrpCrossingTime() const;

   virtual void DumpParameters(const std::string &parameters_to_print);

   virtual int GetSpeedChangeCount() const;

   virtual const Units::Speed GetImSpeedCommandIas() const;

   virtual const Units::Speed GetDelayedImSpeedCommandIas() const;

   virtual const double GetPsi() const;

   virtual const double GetMsi() const;

   virtual const double GetAssignedSpacingGoal() const;

   virtual const Units::Length GetTargetDtgToLastWaypoint() const;

   virtual const double GetSpacingInterval() const;

   const Units::Speed GetUnmodifiedImSpeedCommandIas() const;

   const IMClearance &GetClearance() const;

   const unsigned long int GetActiveFilter() const;

   virtual bool IsBlendWind() const;

   virtual void SetBlendWind(bool wind_blending_enabled) = 0;

   Units::Length GetMiddleToFinalQuantizationTransitionDistance() const;

   Units::Length GetFirstToMiddleQuantizationTransitionDistance() const;

   const Units::Length GetOwnshipDtgToPtp() const;

   const Units::Speed GetPreviousSpeedCommandIas() const;

   const Units::Speed GetOwnshipReferenceIas() const;

   const Units::Speed GetTargetReferenceIas() const;

   const Units::Speed GetOwnshipReferenceGroundspeed() const;

   const Units::Speed GetTargetReferenceGroundspeed() const;

   const Units::Length GetOwnshipReferenceAltitude() const;

   const Units::Length GetTargetReferenceAltitude() const;

   const Units::Length GetTargetKinematicDtgToTrp() const;

   const interval_management::open_source::FIMConfiguration &GetConfiguration() const { return m_configuration; }

   void SetConfiguration(const interval_management::open_source::FIMConfiguration &configuration) {
      m_configuration = configuration;
      CopyParametersFromConfiguration();
   }

   void InitializeProgrammatically(const IMClearance &im_clearance,
                                   const interval_management::open_source::FIMConfiguration &configuration) {
      SetClearance(im_clearance);
      SetConfiguration(configuration);
   }

   bool ValidateClearance(const AircraftIntent &ownship_aircraft_intent,
                          const IMUtils::IMAlgorithmTypes im_algorithm_type);

   void SetClearance(const IMClearance &im_clearance) { m_im_clearance = im_clearance; }

  protected:
   void Copy(const IMAlgorithm &obj);

   virtual void CopyParametersFromConfiguration();

   virtual void SetAssignedSpacingGoal(const IMClearance &clearance) = 0;

   void SetActiveFilter(unsigned long flag);

   AircraftIntent m_target_aircraft_intent;
   FlightStage m_stage_of_im_operation;
   IMClearance m_im_clearance;
   PilotDelay m_pilot_delay;
   WeatherPrediction m_weather_prediction;
   interval_management::open_source::FIMSpeedLimiter m_speed_limiter;

   Units::Speed m_previous_reference_im_speed_command_tas;
   Units::Speed m_previous_im_speed_command_ias;

   interval_management::open_source::FIMConfiguration m_configuration;
   Units::Length m_error_distance;
   Units::InvertedSpeed m_slope;

   Units::Time m_initiate_signal_receipt_time;
   Units::Time m_ownship_ttg_to_abp;
   Units::Time m_ownship_reference_ttg_to_ptp;

   Units::Time m_target_ttg_to_trp;
   Units::Time m_target_trp_crossing_time;
   Units::Time m_target_ttg_to_end_of_route;

   Units::Length m_ownship_kinematic_dtg_to_abp;
   Units::Length m_ownship_kinematic_dtg_to_ptp;

   Units::Length m_target_kinematic_dtg_to_last_waypoint;
   Units::Length m_target_kinematic_dtg_to_trp;

   Units::Speed m_unmodified_im_speed_command_ias;
   Units::Speed m_im_speed_command_ias;
   Units::Speed m_im_speed_command_with_pilot_delay;

   Units::KnotsSpeed m_ownship_reference_cas;
   Units::Speed m_ownship_reference_gs;
   Units::Length m_ownship_reference_altitude;

   Units::Length m_target_reference_altitude;
   Units::Speed m_target_reference_ias;
   Units::Speed m_target_reference_gs;

   int m_total_number_of_im_speed_changes;
   int m_target_reference_lookup_index;
   int m_ownship_reference_lookup_index;
   int m_reference_precalc_index;
   double m_previous_reference_im_speed_command_mach;

   bool m_loaded;
   bool m_im_operation_is_complete;
   bool m_has_maintain_stage;

   Units::NauticalMilesLength m_loaded_middle_to_final_quantize_transition_distance;
   Units::NauticalMilesLength m_loaded_first_to_middle_quantize_transition_distance;
   Units::KnotsSpeed m_loaded_speed_quantize_final_phase;
   Units::KnotsSpeed m_loaded_speed_quantize_middle_phase;
   Units::KnotsSpeed m_loaded_speed_quantize_first_phase;
   bool m_loaded_use_speed_limiting;
   bool m_loaded_use_speed_quantization;

   Units::Frequency m_achieve_control_gain;
   Units::Frequency m_maintain_control_gain;
   Units::Time m_time_threshold;
   bool m_threshold_flag;

  private:
   void IterClearIMAlg();
   void SetWeatherPrediction(const WeatherPrediction &weather_prediction);

   static log4cplus::Logger m_logger;
};

inline const bool IMAlgorithm::IsImOperationComplete() const {
   // This should only be called by non-IM aircraft, to keep regression the same. Non-IM aircraft should probably not
   // report out IM speeds to output files. AAES-799
   return m_ownship_kinematic_dtg_to_ptp <= Units::zero();
}

inline const bool IMAlgorithm::IsOwnshipPassedPtp() const { return false; }

inline const Units::Speed IMAlgorithm::GetPreviousSpeedCommandIas() const { return m_previous_im_speed_command_ias; }

inline const Units::Length IMAlgorithm::GetOwnshipDtgToPtp() const { return m_ownship_kinematic_dtg_to_ptp; }

inline const Units::Length IMAlgorithm::GetTargetDtgToLastWaypoint() const { return Units::Infinity(); }

inline void IMAlgorithm::DisablePilotDelayModel() { m_pilot_delay.SetUsePilotDelay(false); }

inline void IMAlgorithm::EnableDefaultPilotDelayModel() { m_pilot_delay.SetUsePilotDelay(true); }

inline void IMAlgorithm::SetLimitFlag(bool limit_flag) { m_speed_limiter.SetLimitFlag(limit_flag); }

inline void IMAlgorithm::SetErrorDistance(Units::Length error_distance) { m_error_distance = error_distance; }

inline void IMAlgorithm::SetSlope(Units::InvertedSpeed slope) { m_slope = slope; }

inline void IMAlgorithm::SetQuantizeFlag(bool quantize_flag) { m_speed_limiter.SetQuantizeFlag(quantize_flag); }

inline void IMAlgorithm::SetMaxSpeedDeviationPercentage(
      const BoundedValue<double, 0, 100> max_speed_deviation_percentage) {
   m_speed_limiter.SetMaxSpeedDeviationPercentage(max_speed_deviation_percentage);
}

inline const Units::Speed IMAlgorithm::GetFinalPhaseSpeedQuantizationValue() const {
   return m_speed_limiter.GetSpeedQuantizer().GetFinalPhaseSpeedQuantizationValue();
}

inline const Units::Speed IMAlgorithm::GetMiddlePhaseSpeedQuantizationValue() const {
   return m_speed_limiter.GetSpeedQuantizer().GetMiddlePhaseSpeedQuantizationValue();
}

inline const Units::Speed IMAlgorithm::GetFirstPhaseSpeedQuantizationValue() const {
   return m_speed_limiter.GetSpeedQuantizer().GetFirstPhaseSpeedQuantizationValue();
}

inline const bool IMAlgorithm::GetLimitFlag() const { return m_speed_limiter.GetLimitFlag(); }

inline const bool IMAlgorithm::GetQuantizeFlag() const { return m_speed_limiter.GetQuantizeFlag(); }

inline const IMAlgorithm::FlightStage IMAlgorithm::GetFlightStage() const { return m_stage_of_im_operation; }

inline const Units::Time IMAlgorithm::GetOwnshipTtgtoAbp() const { return m_ownship_ttg_to_abp; }

inline const Units::Time IMAlgorithm::GetOwnshipTtgtoPtp() const { return m_ownship_reference_ttg_to_ptp; }

inline const Units::Length IMAlgorithm::GetOwnshipDtgtoAbp() const { return m_ownship_kinematic_dtg_to_abp; }

inline const Units::Time IMAlgorithm::GetTargetTtgToTrp() const { return m_target_ttg_to_trp; }

inline const Units::Time IMAlgorithm::GetTargetTtgToEndOfRoute() const { return m_target_ttg_to_end_of_route; }

inline const Units::Time IMAlgorithm::GetTargetTrpCrossingTime() const { return m_target_trp_crossing_time; }

inline int IMAlgorithm::GetSpeedChangeCount() const { return m_total_number_of_im_speed_changes; }

inline const bool IMAlgorithm::IsLoaded() const { return m_loaded; }

inline bool IMAlgorithm::IsBlendWind() const { return false; }

inline void IMAlgorithm::SetWeatherPrediction(const WeatherPrediction &weather_prediction) {
   m_weather_prediction = weather_prediction;
   m_pilot_delay.SetAtmosphere(weather_prediction.getAtmosphere());
}

inline Units::Length IMAlgorithm::GetMiddleToFinalQuantizationTransitionDistance() const {
   return m_speed_limiter.GetSpeedQuantizer().GetMiddleToFinalQuantizationTransitionDistance();
}

inline Units::Length IMAlgorithm::GetFirstToMiddleQuantizationTransitionDistance() const {
   return m_speed_limiter.GetSpeedQuantizer().GetFirstToMiddleQuantizationTransitionDistance();
}

inline const IMClearance &IMAlgorithm::GetClearance() const { return m_im_clearance; }

inline const Units::Speed IMAlgorithm::GetUnmodifiedImSpeedCommandIas() const {
   return m_unmodified_im_speed_command_ias;
}

inline const Units::Speed IMAlgorithm::GetImSpeedCommandIas() const { return m_im_speed_command_ias; }

inline const Units::Speed IMAlgorithm::GetDelayedImSpeedCommandIas() const {
   return m_im_speed_command_with_pilot_delay;
}

inline const double IMAlgorithm::GetPsi() const { return UNDEFINED_INTERVAL; }

inline const double IMAlgorithm::GetMsi() const { return UNDEFINED_INTERVAL; }

inline const double IMAlgorithm::GetAssignedSpacingGoal() const { return UNDEFINED_INTERVAL; }

inline const unsigned long int IMAlgorithm::GetActiveFilter() const { return m_speed_limiter.GetActiveFilter(); }

inline void IMAlgorithm::SetActiveFilter(unsigned long flag) { m_speed_limiter.SetActiveFilter(flag); }

inline const double IMAlgorithm::GetSpacingInterval() const { return UNDEFINED_INTERVAL; }

inline const Units::Speed IMAlgorithm::GetOwnshipReferenceIas() const { return m_ownship_reference_cas; }

inline const Units::Speed IMAlgorithm::GetTargetReferenceIas() const { return m_target_reference_ias; }

inline const Units::Speed IMAlgorithm::GetTargetReferenceGroundspeed() const { return m_target_reference_gs; }

inline const Units::Speed IMAlgorithm::GetOwnshipReferenceGroundspeed() const { return m_ownship_reference_gs; }

inline const Units::Length IMAlgorithm::GetOwnshipReferenceAltitude() const { return m_ownship_reference_altitude; }

inline const Units::Length IMAlgorithm::GetTargetReferenceAltitude() const { return m_target_reference_altitude; }

inline const Units::Length IMAlgorithm::GetTargetKinematicDtgToTrp() const { return m_target_kinematic_dtg_to_trp; }
}  // namespace open_source
}  // namespace interval_management