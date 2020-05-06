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

   virtual ~IMKinematicAchieve();

   virtual void IterationReset();

   virtual void Initialize(const KineticTrajectoryPredictor& ownship_kinetic_trajectory_predictor,
                           const KineticTrajectoryPredictor& target_kinetic_trajectory_predictor,
                           std::shared_ptr<TangentPlaneSequence> tangent_plane_sequence,
                           AircraftIntent& target_aircraft_intent,
                           const IMClearance& im_clearance,
                           const std::string& achieve_by_point,
                           WeatherPrediction& weather_prediction);

   virtual Guidance Update(const Guidance& prevguidance,
                           const DynamicsState& dynamicsstate,
                           const AircraftState& owntruthstate,
                           const AircraftState& targettruthstate,
                           const vector<AircraftState>& targethistory);

   virtual void ResetDefaults();

   virtual const bool IsOwnshipPassedPtp() const;

   virtual const Units::Length GetTargetDtgToLastWaypoint() const;

   //FIXME aaes-820 shadows name in IMAlgorithm, but is not virtual
   virtual bool IsBlendWind() const;

   //FIXME aaes-820 shadows name in IMAlgorithm, but is not virtual
   void SetBlendWind(bool wind_blending_enabled);

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

   const KinematicTrajectoryPredictor& GetOwnshipKinematicPredictor() const;

   const KinematicTrajectoryPredictor& GetTargetKinematicPredictor() const;

   bool load(DecodedStream* input);

   const bool IsTargetAligned() const;

   const Units::SignedAngle CalculateTargetTrackAngle(const vector<AircraftState> &target_adsb_history);


protected:

   virtual const bool IsOwnshipBelowTransitionAltitude(Units::Length current_ownship_altitude);

   Waypoint MakeWaypointFromState(const AircraftState aircraft_state,
                                  Units::Speed wind_x,
                                  Units::Speed wind_y) const;

   bool CalculateRFLegPhase(); // Returns true if there is an RF Leg on this route

   void ComputeFASTrajectories(
         const AircraftState& owntruthstate,
         const AircraftState& targettruthstate);

   void CheckPredictionAccuracy(
         const AircraftState& owntruthstate,
         const AircraftState& targettruthstate);

   KinematicTrajectoryPredictor m_ownship_kinematic_trajectory_predictor;
   KinematicTrajectoryPredictor m_target_kinematic_trajectory_predictor;

   AchievePointCalcs m_ownship_kinematic_achieve_by_calcs;
   AchievePointCalcs m_target_kinematic_traffic_reference_point_calcs;

   AircraftIntent m_ownship_aircraft_intent;

   AlongPathDistanceCalculator m_ownship_distance_calculator, m_target_distance_calculator,
                               m_ownship_kinetic_distance_calculator, m_im_ownship_distance_calculator;

   std::list<Units::Angle> m_ownship_track_angle_history;
   std::list<Units::Angle> m_target_track_angle_history;

   double m_assigned_spacing_goal_from_input_file{};


   bool m_fas_intent_valid;
   bool m_compute_ownship_kinematic_trajectory;
   bool m_compute_target_kinematic_trajectory;
   bool m_ownship_kinematic_trajectory_dumped;
   bool m_target_kinematic_trajectory_dumped;
   bool m_target_aircraft_exists;
   bool m_target_history_exists;
   int m_target_altitude_failure_count;
   bool m_is_target_aligned;

   static const Units::FeetLength TARGET_ALTITUDE_TOLERANCE;

private:
   void IterClearIMKinAch();

   void HandleTrajectoryPrediction(const AircraftState& owntruthstate,
                                   const AircraftState& targetsyncstate,
                                   const vector<AircraftState>& target_adsb_history);

   void CalculateOwnshipDtgToPlannedTerminationPoint(const AircraftState& current_ownship_state);

   void CalculateOwnshipDtgToAchieveByPoint();

   void CalculateTargetDtgToImPoints(const AircraftState& current_lead_state);

   void TrimAircraftIntentAfterWaypoint(AircraftIntent& aircraft_intent,
                                        const std::string& waypoint_name);

   static log4cplus::Logger logger;

   bool m_blend_wind;
   bool m_new_trajectory_prediction_available;
};

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
