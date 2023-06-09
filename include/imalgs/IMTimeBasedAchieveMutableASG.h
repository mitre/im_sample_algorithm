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

#include "IMTimeBasedAchieve.h"

/**
 * This class contains overrides that permit a special study for Interval Management.
 *
 * For more information, talk to Stuart Bowman.
 * See also AAES-698.
 */

namespace interval_management {
namespace open_source {

class IMTimeBasedAchieveMutableASG : public IMTimeBasedAchieve {
  public:
   IMTimeBasedAchieveMutableASG();

   IMTimeBasedAchieveMutableASG(const IMTimeBasedAchieve &obj);

   virtual ~IMTimeBasedAchieveMutableASG();

   IMTimeBasedAchieveMutableASG &operator=(const IMTimeBasedAchieveMutableASG &obj);

   virtual void IterationReset();

   virtual aaesim::open_source::Guidance Update(
         const aaesim::open_source::Guidance &previous_im_guidance,
         const aaesim::open_source::DynamicsState &three_dof_dynamics_state,
         const interval_management::open_source::AircraftState &current_ownship_state,
         const interval_management::open_source::AircraftState &current_target_state,
         const std::vector<interval_management::open_source::AircraftState> &target_adsb_history);

   virtual void DumpParameters(const std::string &parameters_to_print);

   virtual void Initialize(const OwnshipPredictionParameters &ownship_prediction_parameters,
                           const AircraftIntent &ownship_aircraft_intent,
                           WeatherPrediction &weather_prediction) override;

   bool load(DecodedStream *input);

  protected:
   void Copy(const IMTimeBasedAchieveMutableASG &obj);

  private:
   static log4cplus::Logger logger;

   Units::NauticalMilesLength m_dtg_trigger;
   Units::SecondsTime m_next_assigned_spacing_goal;
   Units::SecondsTime m_asg_change_duration;
   Units::SecondsTime m_asg_change_increment;
};
}  // namespace open_source
}  // namespace interval_management