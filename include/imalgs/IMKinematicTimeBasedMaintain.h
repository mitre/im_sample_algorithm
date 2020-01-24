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

#include "imalgs/IMMaintain.h"
#include "imalgs/KinematicTrajectoryPredictor.h"
#include "aaesim/TrueDistances.h"
#include "aaesim/AchievePointCalcs.h"

class IMKinematicTimeBasedMaintain : public IMMaintain
{
public:

   IMKinematicTimeBasedMaintain();

   virtual ~IMKinematicTimeBasedMaintain();

   virtual void IterationReset();

   // Does NOT inherit from IMAlgorithm::Update()
   Guidance Update(const DynamicsState &dynamics_state,
                   const AircraftState &ownship_aircraft_state,
                   const AircraftState &target_aircraft_state_projected_asg_adjusted,
                   const KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor,
                   const Guidance &guidance_in,
                   const vector<AircraftState> &target_aircraft_state_history,
                   const AchievePointCalcs &ownship_achieve_point_calcs,
                   const AchievePointCalcs &traffic_reference_point_calcs,
                   PilotDelay &pilot_delay_model,
                   const Units::Length &target_kinematic_dtg_to_end_of_route);

   virtual const double GetMsi() const;

   virtual void DumpParameters(const std::string &parameters_to_print);

private:
   void CalculateIas(const Units::Length current_ownship_altitude,
                     const DynamicsState &dynamics_state,
                     const KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor,
                     PilotDelay &pilot_delay);

   void CalculateMach(const Units::Length current_ownship_altitude,
                      const Units::Speed true_airspeed_command,
                      const KinematicTrajectoryPredictor &ownship_kinematic_trajectory_predictor,
                      PilotDelay &pilot_delay);

   const bool IsOwnshipBelowTransitionAltitude(Units::Length current_ownship_altitude,
                                               const KinematicTrajectoryPredictor& ownship_kinematic_trajectory_predictor);

   Units::Time m_measured_spacing_interval;

   //FIXME aaes-820 m_logger shadows name in IMAlgorithm
   static log4cplus::Logger logger;
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