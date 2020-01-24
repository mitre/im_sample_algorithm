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

#include "IMTimeBasedAchieve.h"

/**
 * This class contains overrides that permit a special study for Interval Management.
 *
 * For more information, talk to Stuart Bowman.
 * See also AAES-698.
 */
class IMTimeBasedAchieveMutableASG : public IMTimeBasedAchieve
{
public:

   IMTimeBasedAchieveMutableASG();

   IMTimeBasedAchieveMutableASG(const IMTimeBasedAchieve &obj);

   virtual ~IMTimeBasedAchieveMutableASG();

   IMTimeBasedAchieveMutableASG &operator=(const IMTimeBasedAchieveMutableASG &obj);

   virtual void IterationReset();

   virtual Guidance Update(const Guidance &previous_im_guidance,
                           const DynamicsState &three_dof_dynamics_state,
                           const AircraftState &current_ownship_state,
                           const AircraftState &current_target_state,
                           const vector<AircraftState> &target_adsb_history);

   virtual void DumpParameters(const std::string &parameters_to_print);


   virtual void Initialize(const KineticTrajectoryPredictor &ownship_kinetic_trajectory_predictor,
                           const KineticTrajectoryPredictor &target_kinetic_trajectory_predictor,
                           std::shared_ptr<TangentPlaneSequence> tangent_plane_sequence,
                           AircraftIntent &target_aircraft_intent,
                           const IMClearance &im_clearance,
                           const std::string &achieve_by_point,
                           WeatherPrediction &weather_prediction);

   bool load(DecodedStream *input);

protected:

   void Copy(const IMTimeBasedAchieveMutableASG &obj);

private:
   //FIXME aaes-820 see other notes
   static log4cplus::Logger logger;

   Units::NauticalMilesLength m_dtg_trigger;
   Units::SecondsTime m_next_assigned_spacing_goal;
   Units::SecondsTime m_asg_change_duration;
   Units::SecondsTime m_asg_change_increment;
};
