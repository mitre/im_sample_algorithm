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


class TestVectorSpeedControl : public IMAlgorithm, Loadable
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

    bool load(DecodedStream *input) override;

   void ResetDefaults() override;

protected:

   virtual void SetAssignedSpacingGoal(const IMClearance &clearance);

   virtual const double GetSpacingError() const;

private:
   static unsigned int DEFAULT_DECELERATION_START_TIME_SEC, DEFAULT_ACCELERATION_START_TIME_SEC;
   static unsigned long DEFAULT_DECELERATION_DELTA_IAS, DEFAULT_ACCELERATION_DELTA_IAS;

   AlongPathDistanceCalculator m_distance_calculator;
   Units::KnotsSpeed m_acceleration_phase_target_ias;
   Units::KnotsSpeed m_deceleration_phase_target_ias;
   unsigned long m_acceleration_phase_hold_duration;
   unsigned long m_acceleration_phase_count;
   Units::KnotsSpeed m_acceleration_phase_delta_ias;
   Units::KnotsSpeed m_deceleration_phase_delta_ias;
   unsigned int m_deceleration_start_time_sec, m_acceleration_start_time_sec;
   bool m_acceleration_phase_complete;
   bool m_acceleration_target_achieved;
   std::deque<Units::Speed> m_pilot_delayed_speeds;
};
