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
#include "imalgs/KinematicTrajectoryPredictor.h"
#include "imalgs/TrueDistances.h"
#include "imalgs/AchievePointCalcs.h"

class IMKinematicTimeBasedMaintain : public IMMaintain
{
public:

   IMKinematicTimeBasedMaintain();

   virtual ~IMKinematicTimeBasedMaintain();

   virtual void IterationReset();

   // Does NOT inherit from IMAlgorithm::Update()
   aaesim::open_source::Guidance Update(const aaesim::open_source::DynamicsState &dynamics_state,
                   const interval_management::AircraftState &ownship_aircraft_state,
                   const interval_management::AircraftState &target_aircraft_state_projected_asg_adjusted,
                   const KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor,
                   const aaesim::open_source::Guidance &guidance_in,
                   const vector<interval_management::AircraftState> &target_aircraft_state_history,
                   const interval_management::AchievePointCalcs &ownship_achieve_point_calcs,
                   const interval_management::AchievePointCalcs &traffic_reference_point_calcs,
                   PilotDelay &pilot_delay_model,
                   const Units::Length &target_kinematic_dtg_to_end_of_route);

   virtual const double GetMsi() const;

   virtual void DumpParameters(const std::string &parameters_to_print);

private:
   void CalculateIas(const Units::Length current_ownship_altitude,
                     const aaesim::open_source::DynamicsState &dynamics_state,
                     const KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor,
                     PilotDelay &pilot_delay);

   void CalculateMach(const Units::Length current_ownship_altitude,
                      const Units::Speed true_airspeed_command,
                      const KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor,
                      PilotDelay &pilot_delay);

   const bool IsOwnshipBelowTransitionAltitude(Units::Length current_ownship_altitude,
                                               const KinematicTrajectoryPredictor& ownship_kinematic_trajectory_predictor);

   Units::Time m_measured_spacing_interval;

   static log4cplus::Logger m_logger;
};

inline const bool IMKinematicTimeBasedMaintain::
IsOwnshipBelowTransitionAltitude(Units::Length current_ownship_altitude,
                                 const KinematicTrajectoryPredictor& ownship_kinematic_trajectory_predictor) {
   return current_ownship_altitude <
          ownship_kinematic_trajectory_predictor.GetKinematicDescent4dPredictor()->GetTransitionAltitude();
}

inline const double IMKinematicTimeBasedMaintain::GetMsi() const {
   return Units::SecondsTime(m_measured_spacing_interval).value();
}