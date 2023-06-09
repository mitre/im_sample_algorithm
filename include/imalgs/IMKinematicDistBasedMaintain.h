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

#include "imalgs/IMMaintain.h"
#include "public/KinematicTrajectoryPredictor.h"
#include "imalgs/TrueDistances.h"
#include "imalgs/AchievePointCalcs.h"

namespace interval_management {
namespace open_source {
class IMKinematicDistBasedMaintain : public IMMaintain {
  public:
   IMKinematicDistBasedMaintain();

   virtual ~IMKinematicDistBasedMaintain();

   virtual void IterationReset();

   // Does NOT inherit from IMAlgorithm::Update()
   aaesim::open_source::Guidance Update(
         const aaesim::open_source::DynamicsState &dynamics_state,
         const interval_management::open_source::AircraftState &ownship_aircraft_state,
         const interval_management::open_source::AircraftState
               &target_state_projected_on_ownships_path_at_adjusted_distance,
         const Units::Length target_dtg_along_ownships_path_at_adjusted_distance,
         const Units::Length target_dtg_along_ownships_path,
         const aaesim::open_source::KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor,
         const aaesim::open_source::Guidance &guidance_in,
         const std::vector<interval_management::open_source::AircraftState> &target_aircraft_state_history,
         const interval_management::open_source::AchievePointCalcs &ownship_achieve_point_calcs,
         const interval_management::open_source::AchievePointCalcs &traffic_reference_point_calcs,
         PilotDelay &pilot_delay);

   virtual const double GetMsi() const;

   virtual void DumpParameters(const std::string &parameters_to_print);

   const void SetImSpeedCommandIas(Units::Speed im_speed);

  private:
   void CalculateIas(const Units::Length current_ownship_altitude,
                     const Units::Length target_kinematic_dtg_to_end_of_route,
                     const aaesim::open_source::DynamicsState &dynamics_state,
                     const aaesim::open_source::KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor,
                     PilotDelay &pilot_delay);

   void CalculateMach(const Units::Length current_ownship_altitude,
                      const Units::Length target_kinematic_dtg_to_end_of_route,
                      const Units::Speed true_airspeed_command,
                      const aaesim::open_source::KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor,
                      PilotDelay &pilot_delay, const Units::Mass current_mass);

   void RecordInternalObserverData(
         const interval_management::open_source::AircraftState &ownship_aircraft_state,
         const interval_management::open_source::AircraftState &target_aircraft_state,
         const aaesim::open_source::DynamicsState &dynamics_state, const Units::Speed true_airspeed_command,
         const Units::Length target_true_dtg, const Units::Length ownship_true_dtg,
         const std::vector<interval_management::open_source::AircraftState> &target_aircraft_state_history,
         const aaesim::open_source::KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor);

   const bool IsOwnshipBelowTransitionAltitude(
         Units::Length current_ownship_altitude,
         const aaesim::open_source::KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor);

   Units::Length m_measured_spacing_interval;

   static log4cplus::Logger m_logger;
};

inline const bool IMKinematicDistBasedMaintain::IsOwnshipBelowTransitionAltitude(
      Units::Length current_ownship_altitude,
      const aaesim::open_source::KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor) {
   return current_ownship_altitude <
          ownship_kinematic_trajectory_predictor.GetKinematicDescent4dPredictor()->GetTransitionAltitude();
}

inline const double IMKinematicDistBasedMaintain::GetMsi() const {
   return Units::NauticalMilesLength(m_measured_spacing_interval).value();
}

inline const void IMKinematicDistBasedMaintain::SetImSpeedCommandIas(Units::Speed im_speed) {
   m_im_speed_command_ias = im_speed;
}
}  // namespace open_source
}  // namespace interval_management