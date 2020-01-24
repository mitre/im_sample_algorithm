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

#pragma  once


#include <public/StandardAtmosphere.h>
#include "IMAlgorithm.h"
#include <deque>


class TestVectorSpeedControl : public IMAlgorithm
{
public:
   TestVectorSpeedControl();
   virtual ~TestVectorSpeedControl();

   virtual void Initialize(const KineticTrajectoryPredictor &ownship_kinetic_trajectory_predictor,
                           const KineticTrajectoryPredictor &target_kinetic_trajectory_predictor,
                           std::shared_ptr<TangentPlaneSequence> tangent_plane_sequence,
                           AircraftIntent &target_aircraft_intent,
                           const IMClearance &im_clearance,
                           const std::string &achieve_by_point,
                           WeatherPrediction &weather_prediction);

   virtual Guidance Update(const Guidance &prevguidance,
                           const DynamicsState &dynamicsstate,
                           const AircraftState &owntruthstate,
                           const AircraftState &targettruthstate,
                           const vector<AircraftState> &targethistory);

protected:

   virtual void SetAssignedSpacingGoal(const IMClearance &clearance);

   virtual const double GetSpacingError() const;

private:

   AlongPathDistanceCalculator m_distance_calculator;
   Units::KnotsSpeed m_acceleration_phase_target_ias;
   Units::KnotsSpeed m_deceleration_phase_target_ias;
   unsigned long m_acceleration_phase_hold_duration;
   unsigned long m_acceleration_phase_count;
   bool m_acceleration_phase_complete;
   bool m_acceleration_target_achieved;
   std::deque<Units::Speed> m_pilot_delayed_speeds;
};
