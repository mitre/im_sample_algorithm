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

#include <list>
#include <memory>
#include <vector>

#include "imalgs/AchievePointCalcs.h"
#include "imalgs/IMAchieve.h"
#include "imalgs/IMKinematicTimeBasedMaintain.h"
#include "imalgs/InternalObserver.h"
#include "imalgs/FIMAircraftIntent.h"
#include "public/BlendWindsVerticallyByAltitude.h"
#include "public/KinematicTrajectoryPredictor.h"
#include "public/WindBlendingAlgorithm.h"

namespace interval_management {
namespace open_source {

using namespace mitre::oss::simcore;


class IMKinematicAchieve : public IMAchieve, public Loadable {
  public:
   enum RFLegPhase { NON_RF_LEG, ON_RF_LEG, PRE_RF_LEG };

   static const int MINIMUM_FAS_TRACK_COUNT;
   static const Units::SecondsTime TRACK_ANGLE_TAU;
   static const bool BLEND_WIND_DEFAULT;

   IMKinematicAchieve();

   virtual ~IMKinematicAchieve() = default;

   void IterationReset() override;

   void Initialize(const OwnshipPredictionParameters &ownship_prediction_parameters,
                   const AircraftIntent &ownship_aircraft_intent,
                   mitre::oss::simcore::WeatherPrediction &weather_prediction) override;

   virtual void Initialize(const OwnshipPredictionParameters &ownship_prediction_parameters,
                           const AircraftIntent &ownship_aircraft_intent,
                           mitre::oss::simcore::WeatherPrediction &weather_prediction,
                           std::shared_ptr<TangentPlaneSequence> &position_converter) = 0;

   mitre::oss::simcore::Guidance Update(
         const mitre::oss::simcore::Guidance &prevguidance, const mitre::oss::simcore::DynamicsState &dynamicsstate,
         const interval_management::open_source::AircraftState &owntruthstate,
         const interval_management::open_source::AircraftState &targettruthstate,
         const std::vector<interval_management::open_source::AircraftState> &targethistory) override;

   void ResetDefaults() override;

   const bool IsOwnshipPassedPtp() const override;

   const Units::Length GetTargetDtgToLastWaypoint() const override;

   virtual const interval_management::open_source::AircraftState GetTargetStateProjectedAsgAdjusted() const = 0;

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

   const mitre::oss::simcore::KinematicTrajectoryPredictor &GetOwnshipKinematicPredictor() const;

   const mitre::oss::simcore::KinematicTrajectoryPredictor &GetTargetKinematicPredictor() const;

   bool load(DecodedStream *input) override;

   const bool IsTargetAligned() const;

   const Units::SignedAngle CalculateTargetTrackAngle(
         const std::vector<interval_management::open_source::AircraftState> &target_adsb_history);
   const Waypoint &GetTrafficReferencePoint() const;

  protected:
   virtual const bool IsOwnshipBelowTransitionAltitude(Units::Length current_ownship_altitude) override;

   Waypoint MakeWaypointFromState(const interval_management::open_source::AircraftState &aircraft_state,
                                  Units::Speed wind_x, Units::Speed wind_y) const;

   void CalculateRFLegPhase(const std::vector<PrecalcWaypoint> &waypoints,
                            const Units::Acceleration deceleration_rate_flight_path_angle,
                            const VerticalPath &vertical_path,
                            const std::vector<mitre::oss::simcore::HorizontalPath> &horizontal_trajectory);

   void ComputeFASTrajectories(const interval_management::open_source::AircraftState &owntruthstate,
                               const interval_management::open_source::AircraftState &targettruthstate);

   void CheckPredictionAccuracy(const interval_management::open_source::AircraftState &owntruthstate,
                                const interval_management::open_source::AircraftState &targettruthstate);

   void SetTrafficReferencePointConstraints(const interval_management::open_source::AircraftState &owntruthstate,
                                            const interval_management::open_source::AircraftState &targetsyncstate);

   mitre::oss::simcore::KinematicTrajectoryPredictor m_ownship_kinematic_trajectory_predictor;
   mitre::oss::simcore::KinematicTrajectoryPredictor m_target_kinematic_trajectory_predictor;

   interval_management::open_source::AchievePointCalcs m_ownship_kinematic_achieve_by_calcs;
   Waypoint m_traffic_reference_point;
   interval_management::open_source::AchievePointCalcs m_target_kinematic_traffic_reference_point_calcs;

   FIMAircraftIntent m_ownship_aircraft_intent;

   mitre::oss::simcore::AlongPathDistanceCalculator m_ownship_distance_calculator;
   mitre::oss::simcore::AlongPathDistanceCalculator m_target_distance_calculator;
   mitre::oss::simcore::AlongPathDistanceCalculator m_im_ownship_distance_calculator;

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

   std::shared_ptr<mitre::oss::simcore::WindBlendingAlgorithm> m_wind_blender{
         std::make_shared<mitre::oss::simcore::BlendWindsVerticallyByAltitude>()};

   static const Units::FeetLength TARGET_ALTITUDE_TOLERANCE;

  protected:
   void SetTangentPlaneSequence(std::shared_ptr<TangentPlaneSequence> tangent_plane_sequence);

  private:
   void IterClearIMKinAch();

   void HandleTrajectoryPrediction(
         const interval_management::open_source::AircraftState &owntruthstate,
         const interval_management::open_source::AircraftState &targetsyncstate,
         const std::vector<interval_management::open_source::AircraftState> &target_adsb_history);

   void CalculateOwnshipDtgToPlannedTerminationPoint(
         const interval_management::open_source::AircraftState &current_ownship_state);

   void CalculateOwnshipDtgToAchieveByPoint();

   void CalculateTargetDtgToImPoints(const interval_management::open_source::AircraftState &current_lead_state);

   static log4cplus::Logger logger;

   std::shared_ptr<TangentPlaneSequence> m_tangent_plane_sequence;
};

inline bool IMKinematicAchieve::IsNewTrajectoryPredictionAvailable() const {
   return m_new_trajectory_prediction_available;
}

inline const Waypoint &IMKinematicAchieve::GetTrafficReferencePoint() const { return m_traffic_reference_point; }

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

inline const mitre::oss::simcore::KinematicTrajectoryPredictor &IMKinematicAchieve::GetOwnshipKinematicPredictor()
      const {
   return m_ownship_kinematic_trajectory_predictor;
}

inline const mitre::oss::simcore::KinematicTrajectoryPredictor &IMKinematicAchieve::GetTargetKinematicPredictor()
      const {
   return m_target_kinematic_trajectory_predictor;
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

inline const bool IMKinematicAchieve::IsTargetAligned() const { return m_is_target_aligned; }

inline void IMKinematicAchieve::SetTangentPlaneSequence(std::shared_ptr<TangentPlaneSequence> tangent_plane_sequence) {
   m_tangent_plane_sequence = tangent_plane_sequence;
}
}  // namespace open_source
}  // namespace interval_management
