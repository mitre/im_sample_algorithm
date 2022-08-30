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

#include "imalgs/IMAchieve.h"
#include "imalgs/IMKinematicTimeBasedMaintain.h"
#include "imalgs/KinematicTrajectoryPredictor.h"
#include "imalgs/AchievePointCalcs.h"

class IMKinematicAchieve : public IMAchieve, public Loadable
{

public:

   enum RFLegPhase
   {
      NON_RF_LEG,
      ON_RF_LEG,
      PRE_RF_LEG
   };

   static const int MINIMUM_FAS_TRACK_COUNT;
   static const Units::SecondsTime TRACK_ANGLE_TAU;

   IMKinematicAchieve();

   virtual ~IMKinematicAchieve() = default;

   void IterationReset() override;

   void Initialize(std::shared_ptr<const aaesim::BadaPerformanceCalculator> aircraft_performance_calculator,
                  OwnshipPredictionParameters ownship_prediction_parameters,
                  const AircraftIntent &ownship_aircraft_intent,
                  const AircraftIntent& target_aircraft_intent,
                  const IMClearance& im_clearance,
                  WeatherPrediction& weather_prediction) override;

   aaesim::open_source::Guidance Update(const aaesim::open_source::Guidance& prevguidance,
                  const aaesim::open_source::DynamicsState& dynamicsstate,
                  const interval_management::AircraftState& owntruthstate,
                  const interval_management::AircraftState& targettruthstate,
                  const vector<interval_management::AircraftState>& targethistory) override;

   void ResetDefaults() override;

   const bool IsOwnshipPassedPtp() const override;

   const Units::Length GetTargetDtgToLastWaypoint() const override;

   virtual const interval_management::AircraftState GetTargetStateProjectedAsgAdjusted() const = 0;

   bool IsBlendWind() const override;

   void SetBlendWind(bool wind_blending_enabled) override;

   /*
    * API
    */
   void SetRecordMaintainMetrics(bool new_value);

   /*
    * API
    */
   const bool GetRecordMaintainMetrics() const;

   /*
    * API
    * Return the ownship kinematic vertical predictor if a new trajectory was generated this frame.
    */
   std::shared_ptr<VerticalPredictor> GetOwnshipVerticalPredictor() const;

   /*
    * API
    * Return the target kinematic vertical predictor if a new trajectory was generated this frame.
    */
   std::shared_ptr<VerticalPredictor> GetTargetVerticalPredictor() const;

   /*
    * API
    * Return a boolean indicating if the most recent call to Update() caused a new
    * trajectory prediction to occur for either ownship or target.
    */
   bool IsNewTrajectoryPredictionAvailable() const;

   const KinematicTrajectoryPredictor& GetOwnshipKinematicPredictor() const;

   const KinematicTrajectoryPredictor& GetTargetKinematicPredictor() const;

   bool load(DecodedStream* input);

   const bool IsTargetAligned() const;

   const Units::SignedAngle CalculateTargetTrackAngle(const vector<interval_management::AircraftState> &target_adsb_history);
   const Waypoint& GetTrafficReferencePoint() const;

protected:

   virtual const bool IsOwnshipBelowTransitionAltitude(Units::Length current_ownship_altitude);

   Waypoint MakeWaypointFromState(const interval_management::AircraftState aircraft_state,
                                  Units::Speed wind_x,
                                  Units::Speed wind_y) const;

   bool CalculateRFLegPhase(); // Returns true if there is an RF Leg on this route

   void ComputeFASTrajectories(
         const interval_management::AircraftState& owntruthstate,
         const interval_management::AircraftState& targettruthstate);

   void CheckPredictionAccuracy(
         const interval_management::AircraftState& owntruthstate,
         const interval_management::AircraftState& targettruthstate);

   void SetTrafficReferencePointConstraints(
         const interval_management::AircraftState& owntruthstate,
         const interval_management::AircraftState& targetsyncstate);

   KinematicTrajectoryPredictor m_ownship_kinematic_trajectory_predictor;
   KinematicTrajectoryPredictor m_target_kinematic_trajectory_predictor;

   interval_management::AchievePointCalcs m_ownship_kinematic_achieve_by_calcs;
   Waypoint m_traffic_reference_point;    // can be specified or generated
   interval_management::AchievePointCalcs m_target_kinematic_traffic_reference_point_calcs;

   AircraftIntent m_ownship_aircraft_intent;

   AlongPathDistanceCalculator m_ownship_distance_calculator,
                               m_target_distance_calculator,
                               m_im_ownship_distance_calculator;

   std::list<Units::Angle> m_ownship_track_angle_history;
   std::list<Units::Angle> m_target_track_angle_history;

   double m_assigned_spacing_goal_from_input_file;

   int m_target_altitude_failure_count;

   bool m_fas_intent_valid;
   bool m_compute_ownship_kinematic_trajectory;
   bool m_compute_target_kinematic_trajectory;
   bool m_target_aircraft_exists;
   bool m_target_history_exists;
   bool m_is_target_aligned;
   bool m_new_trajectory_prediction_available;

   static const Units::FeetLength TARGET_ALTITUDE_TOLERANCE;

private:
   void IterClearIMKinAch();

   void HandleTrajectoryPrediction(const interval_management::AircraftState& owntruthstate,
                                   const interval_management::AircraftState& targetsyncstate,
                                   const vector<interval_management::AircraftState>& target_adsb_history);

   void CalculateOwnshipDtgToPlannedTerminationPoint(const interval_management::AircraftState& current_ownship_state);

   void CalculateOwnshipDtgToAchieveByPoint();

   void CalculateTargetDtgToImPoints(const interval_management::AircraftState& current_lead_state);

   void TrimAircraftIntentAfterWaypoint(AircraftIntent& aircraft_intent,
                                        const std::string& waypoint_name);

   void SetTangentPlaneSequence(std::shared_ptr<TangentPlaneSequence> tangent_plane_sequence);

   static log4cplus::Logger logger;

   bool m_blend_wind;
   Wgs84AlongPathDistanceCalculator m_ownship_kinetic_distance_calculator;

   std::shared_ptr<TangentPlaneSequence> m_tangent_plane_sequence;

};

inline bool IMKinematicAchieve::IsNewTrajectoryPredictionAvailable() const {
   return m_new_trajectory_prediction_available;
}

inline const Waypoint& IMKinematicAchieve::GetTrafficReferencePoint() const {
   return m_traffic_reference_point;
}

inline void IMKinematicAchieve::CalculateOwnshipDtgToAchieveByPoint() {
   m_ownship_kinematic_dtg_to_abp =
         m_ownship_kinematic_dtg_to_ptp - m_ownship_kinematic_achieve_by_calcs.GetDistanceFromWaypoint();
}

inline const Units::Length IMKinematicAchieve::GetTargetDtgToLastWaypoint() const {
   return m_target_kinematic_dtg_to_last_waypoint;
}

inline const bool IMKinematicAchieve::IsOwnshipPassedPtp() const {
   return m_ownship_kinematic_dtg_to_ptp <= Units::zero();
}

inline std::shared_ptr<VerticalPredictor> IMKinematicAchieve::GetOwnshipVerticalPredictor() const {
   if (m_new_trajectory_prediction_available) {
      return m_ownship_kinematic_trajectory_predictor.GetVerticalPredictor();
   }
   return nullptr;
}

inline std::shared_ptr<VerticalPredictor> IMKinematicAchieve::GetTargetVerticalPredictor() const {
   if (m_new_trajectory_prediction_available) {
      return m_target_kinematic_trajectory_predictor.GetVerticalPredictor();
   }
   return nullptr;
}

inline const KinematicTrajectoryPredictor& IMKinematicAchieve::GetOwnshipKinematicPredictor() const {
   return m_ownship_kinematic_trajectory_predictor;
}

inline const KinematicTrajectoryPredictor& IMKinematicAchieve::GetTargetKinematicPredictor() const {
   return m_target_kinematic_trajectory_predictor;
}

inline bool IMKinematicAchieve::IsBlendWind() const {
   return m_blend_wind;
}

inline void IMKinematicAchieve::SetBlendWind(bool wind_blending_enabled) {
   m_blend_wind = wind_blending_enabled;
}

inline void IMKinematicAchieve::SetRecordMaintainMetrics(bool new_value) {
   InternalObserver::getInstance()->SetRecordMaintainMetrics(new_value);
}

inline const bool IMKinematicAchieve::GetRecordMaintainMetrics() const {
   return InternalObserver::getInstance()->GetRecordMaintainMetrics();
}

inline const bool IMKinematicAchieve::IsOwnshipBelowTransitionAltitude(Units::Length current_ownship_altitude) {
   return current_ownship_altitude <
          m_ownship_kinematic_trajectory_predictor.GetKinematicDescent4dPredictor()->GetTransitionAltitude();
}

inline const bool IMKinematicAchieve::IsTargetAligned() const {
   return m_is_target_aligned;
}

inline void IMKinematicAchieve::SetTangentPlaneSequence(std::shared_ptr<TangentPlaneSequence> tangent_plane_sequence) {
   m_tangent_plane_sequence = tangent_plane_sequence;
}
