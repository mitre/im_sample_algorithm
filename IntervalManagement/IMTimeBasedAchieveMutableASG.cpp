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
// 2023 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include <stdexcept>
#include <imalgs/IMTimeBasedAchieveMutableASG.h>

#include "public/CustomMath.h"
#include "public/AircraftCalculations.h"
#include "public/CoreUtils.h"
#include "public/SimulationTime.h"

using namespace std;
using namespace interval_management::open_source;

log4cplus::Logger IMTimeBasedAchieveMutableASG::logger =
      log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("IMTimeBasedAchieveMutableASG"));

IMTimeBasedAchieveMutableASG::IMTimeBasedAchieveMutableASG()
   : IMTimeBasedAchieve(),
     m_dtg_trigger(Units::ZERO_LENGTH),
     m_next_assigned_spacing_goal(Units::zero()),
     m_asg_change_duration(Units::zero()),
     m_asg_change_increment(Units::zero()) {}

IMTimeBasedAchieveMutableASG::~IMTimeBasedAchieveMutableASG() {}

IMTimeBasedAchieveMutableASG &IMTimeBasedAchieveMutableASG::operator=(const IMTimeBasedAchieveMutableASG &obj) {
   if (this != &obj) {
      Copy(obj);
   }

   return *this;
}

void IMTimeBasedAchieveMutableASG::Copy(const IMTimeBasedAchieveMutableASG &obj) {
   IMTimeBasedAchieve::Copy(obj);
   m_asg_change_increment = obj.m_asg_change_increment;
   m_asg_change_duration = obj.m_asg_change_duration;
   m_next_assigned_spacing_goal = obj.m_next_assigned_spacing_goal;
   m_dtg_trigger = obj.m_dtg_trigger;
}

void IMTimeBasedAchieveMutableASG::IterationReset() { IMTimeBasedAchieve::IterationReset(); }

bool IMTimeBasedAchieveMutableASG::load(DecodedStream *input) {
   register_var("dtg_trigger_nm", &m_dtg_trigger, true);
   register_var("new_asg_sec", &m_next_assigned_spacing_goal, true);
   register_var("asg_change_duration_sec", &m_asg_change_duration, true);
   const bool mLoaded = IMTimeBasedAchieve::load(input);

   return mLoaded;
}

void IMTimeBasedAchieveMutableASG::Initialize(const OwnshipPredictionParameters &ownship_prediction_parameters,
                                              const AircraftIntent &ownship_aircraft_intent,
                                              aaesim::open_source::WeatherPrediction &weather_prediction) {
   IMKinematicAchieve::Initialize(ownship_prediction_parameters, ownship_aircraft_intent, weather_prediction);

   if (m_loaded) {
      // Calculate the change increment relative to the simulation step.
      // Positive means the ASG is increasing.
      if (m_asg_change_duration > Units::zero()) {
         m_asg_change_increment = ((m_next_assigned_spacing_goal - m_assigned_spacing_goal) / m_asg_change_duration) *
                                  aaesim::open_source::SimulationTime::GetSimulationTimeStep();
      } else {
         // make the change immediate
         m_asg_change_increment = (m_next_assigned_spacing_goal - m_assigned_spacing_goal);
      }
   }
}

aaesim::open_source::Guidance IMTimeBasedAchieveMutableASG::Update(
      const aaesim::open_source::Guidance &previous_im_guidance,
      const aaesim::open_source::DynamicsState &three_dof_dynamics_state,
      const interval_management::open_source::AircraftState &current_ownship_state,
      const interval_management::open_source::AircraftState &current_target_state,
      const vector<interval_management::open_source::AircraftState> &target_adsb_history) {
   // This class only does one thing different than the superclass: it modifies
   // the assigned spacing goal as indicated by the input file parameters.
   const Units::FeetLength x(current_ownship_state.m_x), y(current_ownship_state.m_y);
   Units::Length kinematic_dtg;
   m_ownship_distance_calculator.CalculateAlongPathDistanceFromPosition(x, y, kinematic_dtg);
   const bool increment = kinematic_dtg < m_dtg_trigger && m_assigned_spacing_goal != m_next_assigned_spacing_goal;
   if (increment) {
      m_assigned_spacing_goal += m_asg_change_increment;
   }

   aaesim::open_source::Guidance guidanceout =
         IMTimeBasedAchieve::Update(previous_im_guidance, three_dof_dynamics_state, current_ownship_state,
                                    current_target_state, target_adsb_history);

   return guidanceout;
}

void IMTimeBasedAchieveMutableASG::DumpParameters(const string &parameters_to_print) {
   LOG4CPLUS_DEBUG(IMTimeBasedAchieveMutableASG::logger,
                   "--------------------------------------------------------------------" << endl);
   LOG4CPLUS_DEBUG(IMTimeBasedAchieveMutableASG::logger, parameters_to_print.c_str() << endl << endl);
   LOG4CPLUS_DEBUG(logger, "dtg_trigger_nm = " << m_dtg_trigger);
   LOG4CPLUS_DEBUG(logger, "new_asg_sec = " << m_next_assigned_spacing_goal);
   LOG4CPLUS_DEBUG(logger, "asg_change_duration_sec = " << m_asg_change_duration);
   LOG4CPLUS_DEBUG(logger, "calculated asg change increment per simulation step = " << m_asg_change_increment);

   IMTimeBasedAchieve::DumpParameters(parameters_to_print);

   LOG4CPLUS_DEBUG(IMTimeBasedAchieveMutableASG::logger,
                   "--------------------------------------------------------------------" << endl);
}
