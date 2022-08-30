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

#include "public/Guidance.h"
#include "public/AircraftIntent.h"
#include "imalgs/AircraftState.h"
#include "utility/Logging.h"
#include "public/PilotDelay.h"
#include "aaesim/Wgs84KineticDescentPredictor.h"
#include "public/ThreeDOFDynamics.h"
#include "public/TangentPlaneSequence.h"
#include <scalar/Time.h>
#include <scalar/Speed.h>
#include <scalar/Length.h>
#include <scalar/Frequency.h>
#include "imalgs/IMClearance.h"

class IMAlgorithm
{
   friend class IMAlgorithm_MaxSpeedDeviation_Test;
public:
   static const Units::SecondsTime ASSIGNED_SPACING_GOAL_DEFAULT;
   static const bool LIMIT_FLAG_DEFAULT;
   static const Units::NauticalMilesLength DIST_QUANTIZE_1_DEFAULT;
   static const Units::NauticalMilesLength DIST_QUANTIZE_2_DEFAULT;
   static Units::KnotsSpeed SPEED_QUANTIZE_1_DEFAULT;  // not const because it needs to change based on pilot delay flag
   static const Units::KnotsSpeed SPEED_QUANTIZE_2_DEFAULT;
   static const Units::KnotsSpeed SPEED_QUANTIZE_3_DEFAULT;
   static const Units::NauticalMilesLength ERROR_DISTANCE_DEFAULT;
   static const bool QUANTIZE_FLAG_DEFAULT;
   static const Units::SecondsPerNauticalMileInvertedSpeed SLOPE_DEFAULT;
   static const std::string RESET_MSG;
   static const double UNDEFINED_INTERVAL;

   enum FlightStage
   {
      UNSET = -1,
      NONE = 0,
      ACHIEVE = 1,
      MAINTAIN = 2
   };

   struct OwnshipPredictionParameters {
      Units::Angle maximum_allowable_bank_angle;
      Units::Speed transition_ias;
      double transition_mach;
      Units::Length transition_altitude;
      Units::Length expected_cruise_altitude;
   };

   IMAlgorithm();

   IMAlgorithm(const IMAlgorithm &obj);

   virtual ~IMAlgorithm();

   IMAlgorithm &operator=(const IMAlgorithm &obj);

   virtual void Initialize(std::shared_ptr<const aaesim::BadaPerformanceCalculator> aircraft_performance_calculator,
                           OwnshipPredictionParameters ownship_prediction_parameters,
                           const AircraftIntent &ownship_aircraft_intent,
                           const AircraftIntent &target_aircraft_intent,
                           const IMClearance &im_clearance,
                           WeatherPrediction &weather_prediction);

   // Called during initialization whenever the clearance type is not CUSTOM.
   virtual void ResetDefaults();

   virtual void IterationReset();

   virtual aaesim::open_source::Guidance Update(const aaesim::open_source::Guidance &prevguidance,
                           const aaesim::open_source::DynamicsState &dynamicsstate,
                           const interval_management::AircraftState &owntruthstate,
                           const interval_management::AircraftState &targettruthstate,
                           const vector<interval_management::AircraftState> &targethistory);

   virtual const double GetSpacingError() const = 0;  // Returned value has units stripped off.

   virtual const bool IsImOperationComplete() const;

   virtual const bool IsOwnshipPassedPtp() const;

   void UpdatePositionMetrics(const interval_management::AircraftState &ownship_aircraft_state,
                              const interval_management::AircraftState &target_aircraft_state);

   void SetPilotDelay(const bool pilot_delay_on,
                      const Units::Time pilot_delay_mean,
                      const Units::Time pilot_delay_standard_deviation);

   void DisablePilotDelayModel();

   void EnableDefaultPilotDelayModel();

   void SetMiddleToFinalQuantizeTransitionDistance(const Units::Length middle_to_final_quantize_transition_distance);

   void SetFirstToMiddleQuantizeTransitionDistance(const Units::Length first_to_middle_quantize_transition_distance);

   void SetFinalPhaseSpeedQuantizeValue(const Units::KnotsSpeed final_phase_speed_quantize_value);

   void SetMiddlePhaseSpeedQuantizationValue(const Units::KnotsSpeed middle_phase_speed_quantization_value);

   void SetFirstPhaseSpeedQuantizationValue(const Units::KnotsSpeed first_phase_speed_quantization_value);

   void SetErrorDistance(Units::Length error_distance);

   void SetSlope(Units::InvertedSpeed slope);

   void SetLimitFlag(bool limit_flag);

   void SetQuantizeFlag(bool quantize_flag);

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

   const Wgs84KineticDescentPredictor *GetOwnshipKineticTrajectoryPredictor() const;

   const Wgs84KineticDescentPredictor *GetTargetKineticTrajectoryPredictor() const;

   virtual void InitializeFmsPredictors(const Wgs84KineticDescentPredictor &ownship_kinetic_trajectory_predictor,
                                        const Wgs84KineticDescentPredictor &target_kinetic_trajectory_predictor);

   // Values used in speed command limiting calculations.
   // Added and subtracted to 1 to compute the low and high
   // speed command limits.
   static const double DEFAULT_SPEED_DEVIATION_PERCENTAGE;

 protected:
   const Wgs84KineticDescentPredictor *m_target_kinetic_trajectory_predictor;
   const Wgs84KineticDescentPredictor *m_ownship_kinetic_trajectory_predictor;

   bool HasKineticPredictors() const;

   void Copy(const IMAlgorithm &obj);

   double MachHysteresis(double newmach,
                         double oldmach);

   Units::Speed QuantizeThreshold(const Units::Length dtg_to_abp_in,
                                  const Units::Speed computed_ias_command,
                                  const Units::Speed old_ias_command,
                                  Units::Speed &threshold);

   Units::Speed QuantizeThreshold(const Units::Length dtg_to_abp_in,
                                  const Units::Speed computed_ias_command,
                                  const Units::Speed old_ias_command);

   void SetWeatherPrediction(const WeatherPrediction &weather_prediction);

   // If limiting in effect, this method will limit the command speed based on high and low limit.  The high and low
   // limits are computed from a trajectory speed and a fixed limiting factor.
   // Changes to these flags should be reflected also online:
   // https://huddle.mitre.org/pages/viewpage.action?pageId=29263788#TrajectoryFileFields-CommandSpeedLimitFlag
   Units::Speed LimitImSpeedCommand(const Units::Speed im_speed_command_ias,
                                    const double reference_velocity_mps,
                                    const Units::Length distance_to_go_to_abp,
                                    std::shared_ptr<const aaesim::BadaPerformanceCalculator> bada_with_calc,
                                    const Units::Length ownship_altitude,
                                    const aaesim::open_source::bada_utils::FlapConfiguration flap_configuration,
                                    const Units::Speed rf_upper_limit);

   double LimitImMachCommand(
         double estimated_mach,
         const double nominal_mach,
         std::shared_ptr<const aaesim::BadaPerformanceCalculator> bada_calculator,
         const Units::Length current_ownship_altitude);


   virtual void SetMaxSpeedDeviationPercentage(const double max_speed_deviation_factor);

   template<class X>
   const X LowLimit(const X val) {
      return (val * m_low_speed_coef);
   };

   template<class X>
   const X HighLimit(const X val) {
      return (val * m_high_speed_coef);
   };

   virtual void SetAssignedSpacingGoal(const IMClearance &clearance) = 0;

   void SetActiveFilter(unsigned long flag);

   const Units::Speed GetRFLegSpeedLimit(Units::Length dtg_to_ptp) const;

   std::vector<std::pair<Units::Length, Units::Speed>> m_rfleg_limits; // (dist_to_go, upper_ias_limit)


   AircraftIntent m_target_aircraft_intent;
   FlightStage m_stage_of_im_operation;
   IMClearance m_im_clearance;
   PilotDelay m_pilot_delay;
   WeatherPrediction m_weather_prediction;

   Units::Length m_middle_to_final_quantize_transition_distance;
   Units::Length m_first_to_middle_quantize_transition_distance;

   Units::Speed m_speed_quantize_final_phase;
   Units::Speed m_speed_quantize_middle_phase;
   Units::Speed m_speed_quantize_first_phase;

   Units::Speed m_previous_reference_im_speed_command_tas;
   Units::Speed m_previous_im_speed_command_ias;

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
   Units::Length m_ownship_kinetic_dtg_to_ptp;
   Units::Length m_ownship_kinetic_dtg_to_abp;

   Units::Length m_target_kinematic_dtg_to_last_waypoint;
   Units::Length m_target_kinematic_dtg_to_trp;
   Units::Length m_target_kinetic_dtg_to_last_waypoint;
   Units::Length m_target_kinetic_dtg_to_trp;

   Units::Speed m_unmodified_im_speed_command_ias;
   Units::Speed m_im_speed_command_ias;
   Units::Speed m_im_speed_command_with_pilot_delay;

   // To trigger recalculation with blended wind AAES-933
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

   unsigned long int m_active_filter_flag;

   bool m_loaded;
   bool m_limit_flag;
   bool m_quantize_flag;
   bool m_im_operation_is_complete;
   bool m_has_rf_leg;
   bool m_has_maintain_stage;

private:
   double m_low_speed_coef;
   double m_high_speed_coef;
   bool m_has_kinetic_predictors;
   void IterClearIMAlg();

   static log4cplus::Logger m_logger;
};

inline const bool IMAlgorithm::IsImOperationComplete() const {
   // This should only be called by non-IM aircraft, to keep regression the same. Non-IM aircraft should probably not
   // report out IM speeds to output files. AAES-799
   return m_ownship_kinematic_dtg_to_ptp <= Units::zero();
}

inline const bool IMAlgorithm::IsOwnshipPassedPtp() const {
   return false;
}

inline const Units::Speed IMAlgorithm::GetPreviousSpeedCommandIas() const {
   return m_previous_im_speed_command_ias;
}

inline const Units::Length IMAlgorithm::GetOwnshipDtgToPtp() const {
   return m_ownship_kinematic_dtg_to_ptp;
}

inline const Units::Length IMAlgorithm::GetTargetDtgToLastWaypoint() const {
   return Units::Infinity();
}

inline void IMAlgorithm::DisablePilotDelayModel() {
   m_pilot_delay.SetUsePilotDelay(false);
}

inline void IMAlgorithm::EnableDefaultPilotDelayModel() {
   m_pilot_delay.SetUsePilotDelay(true);
}

inline void IMAlgorithm::
SetMiddleToFinalQuantizeTransitionDistance(const Units::Length middle_to_final_quantize_transition_distance) {
   m_middle_to_final_quantize_transition_distance = middle_to_final_quantize_transition_distance;
}

inline void IMAlgorithm::
SetFirstToMiddleQuantizeTransitionDistance(const Units::Length first_to_middle_quantize_transition_distance) {
   m_first_to_middle_quantize_transition_distance = first_to_middle_quantize_transition_distance;
}

inline void IMAlgorithm::SetFinalPhaseSpeedQuantizeValue(const Units::KnotsSpeed final_phase_speed_quantize_value) {
   m_speed_quantize_final_phase = final_phase_speed_quantize_value;
}

inline void
IMAlgorithm::SetMiddlePhaseSpeedQuantizationValue(const Units::KnotsSpeed middle_phase_speed_quantization_value) {
   m_speed_quantize_middle_phase = middle_phase_speed_quantization_value;
}

inline void
IMAlgorithm::SetFirstPhaseSpeedQuantizationValue(const Units::KnotsSpeed first_phase_speed_quantization_value) {
   m_speed_quantize_first_phase = first_phase_speed_quantization_value;
}

inline void IMAlgorithm::SetLimitFlag(bool limit_flag) {
   m_limit_flag = limit_flag;
}

inline void IMAlgorithm::SetErrorDistance(Units::Length error_distance) {
   m_error_distance = error_distance;
}

inline void IMAlgorithm::SetSlope(Units::InvertedSpeed slope) {
   m_slope = slope;
}

inline void IMAlgorithm::SetQuantizeFlag(bool quantize_flag) {
   m_quantize_flag = quantize_flag;
}

inline const Units::Speed IMAlgorithm::GetFinalPhaseSpeedQuantizationValue() const {
   return m_speed_quantize_final_phase;
}

inline const Units::Speed IMAlgorithm::GetMiddlePhaseSpeedQuantizationValue() const {
   return m_speed_quantize_middle_phase;
}

inline const Units::Speed IMAlgorithm::GetFirstPhaseSpeedQuantizationValue() const {
   return m_speed_quantize_first_phase;
}

inline const bool IMAlgorithm::GetLimitFlag() const {
   return m_limit_flag;
}

inline const bool IMAlgorithm::GetQuantizeFlag() const {
   return m_quantize_flag;
}

inline const IMAlgorithm::FlightStage IMAlgorithm::GetFlightStage() const {
   return m_stage_of_im_operation;
}

inline const Units::Time IMAlgorithm::GetOwnshipTtgtoAbp() const {
   return m_ownship_ttg_to_abp;
}

inline const Units::Time IMAlgorithm::GetOwnshipTtgtoPtp() const {
   return m_ownship_reference_ttg_to_ptp;
}

inline const Units::Length IMAlgorithm::GetOwnshipDtgtoAbp() const {
   return m_ownship_kinematic_dtg_to_abp;
}

inline const Units::Time IMAlgorithm::GetTargetTtgToTrp() const {
   return m_target_ttg_to_trp;
}

inline const Units::Time IMAlgorithm::GetTargetTtgToEndOfRoute() const {
   return m_target_ttg_to_end_of_route;
}

inline const Units::Time IMAlgorithm::GetTargetTrpCrossingTime() const {
   return m_target_trp_crossing_time;
}

inline int IMAlgorithm::GetSpeedChangeCount() const {
   return m_total_number_of_im_speed_changes;
}

inline const bool IMAlgorithm::IsLoaded() const {
   return m_loaded;
}

inline bool IMAlgorithm::IsBlendWind() const {
   return false;
}

inline double IMAlgorithm::MachHysteresis(double newmach,
                                          double oldmach) {
   if (fabs(newmach - oldmach) < 0.02 * 0.75) {
      return oldmach;
   }
   return newmach;
}

inline void IMAlgorithm::SetWeatherPrediction(const WeatherPrediction &weather_prediction) {
   m_weather_prediction = weather_prediction;
   m_pilot_delay.SetAtmosphere(weather_prediction.getAtmosphere());
}

inline Units::Length IMAlgorithm::GetMiddleToFinalQuantizationTransitionDistance() const {
   return m_middle_to_final_quantize_transition_distance;
}

inline Units::Length IMAlgorithm::GetFirstToMiddleQuantizationTransitionDistance() const {
   return m_first_to_middle_quantize_transition_distance;
}

inline const IMClearance &IMAlgorithm::GetClearance() const {
   return m_im_clearance;
}

inline const Units::Speed IMAlgorithm::GetUnmodifiedImSpeedCommandIas() const {
   return m_unmodified_im_speed_command_ias;
}

inline const Units::Speed IMAlgorithm::GetImSpeedCommandIas() const {
   return m_im_speed_command_ias;
}

inline const Units::Speed IMAlgorithm::GetDelayedImSpeedCommandIas() const {
   return m_im_speed_command_with_pilot_delay;
}

inline const double IMAlgorithm::GetPsi() const {
   return UNDEFINED_INTERVAL;
}

inline const double IMAlgorithm::GetMsi() const {
   return UNDEFINED_INTERVAL;
}

inline const double IMAlgorithm::GetAssignedSpacingGoal() const {
   return UNDEFINED_INTERVAL;
}

inline const unsigned long int IMAlgorithm::GetActiveFilter() const {
   return m_active_filter_flag;
}

inline void IMAlgorithm::SetActiveFilter(unsigned long flag) {
   m_active_filter_flag = flag;
}

inline const double IMAlgorithm::GetSpacingInterval() const {
   return UNDEFINED_INTERVAL;
}

inline const Units::Speed IMAlgorithm::GetOwnshipReferenceIas() const {
   return m_ownship_reference_cas;
}

inline const Units::Speed IMAlgorithm::GetTargetReferenceIas() const {
   return m_target_reference_ias;
}

inline const Units::Speed IMAlgorithm::GetTargetReferenceGroundspeed() const {
   return m_target_reference_gs;
}

inline const Units::Speed IMAlgorithm::GetOwnshipReferenceGroundspeed() const {
   return m_ownship_reference_gs;
}

inline const Units::Length IMAlgorithm::GetOwnshipReferenceAltitude() const {
   return m_ownship_reference_altitude;
}

inline const Units::Length IMAlgorithm::GetTargetReferenceAltitude() const {
   return m_target_reference_altitude;
}

inline const Units::Length IMAlgorithm::GetTargetKinematicDtgToTrp() const {
   return m_target_kinematic_dtg_to_trp;
}

inline const Wgs84KineticDescentPredictor *IMAlgorithm::GetTargetKineticTrajectoryPredictor() const {
   return m_target_kinetic_trajectory_predictor;
}

inline const Wgs84KineticDescentPredictor *IMAlgorithm::GetOwnshipKineticTrajectoryPredictor() const {
   return m_ownship_kinetic_trajectory_predictor;
}

inline bool IMAlgorithm::HasKineticPredictors() const {
   return m_has_kinetic_predictors;
}