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

#include <public/ThreeDOFDynamics.h>
#include <imalgs/TestVectorSpeedControl.h>
#include <public/StandardAtmosphere.h>
#include <imalgs/KinematicTrajectoryPredictor.h>

unsigned int TestVectorSpeedControl::DEFAULT_DECELERATION_START_TIME_SEC = 60;
unsigned int TestVectorSpeedControl::DEFAULT_ACCELERATION_START_TIME_SEC = 240;
unsigned long TestVectorSpeedControl::DEFAULT_DECELERATION_DELTA_IAS = 30.0;
unsigned long TestVectorSpeedControl::DEFAULT_ACCELERATION_DELTA_IAS = 40.0;

TestVectorSpeedControl::TestVectorSpeedControl()
      : m_distance_calculator(),
        m_acceleration_phase_target_ias(),
        m_deceleration_phase_target_ias(),
        m_acceleration_phase_hold_duration(),
        m_acceleration_phase_count(),
        m_acceleration_phase_delta_ias(),
        m_deceleration_phase_delta_ias(),
        m_deceleration_start_time_sec(),
        m_acceleration_start_time_sec(),
        m_acceleration_phase_complete(false),
        m_acceleration_target_achieved(false),
        m_pilot_delayed_speeds() {
}

TestVectorSpeedControl::~TestVectorSpeedControl() = default;

void TestVectorSpeedControl::InitializeFmsPredictors(const Wgs84KineticDescentPredictor &ownship_kinetic_trajectory_predictor,
                                                     const Wgs84KineticDescentPredictor &target_kinetic_trajectory_predictor) {
   IMAlgorithm::InitializeFmsPredictors(ownship_kinetic_trajectory_predictor, target_kinetic_trajectory_predictor);

   if (!m_loaded) {
      ResetDefaults();
      m_loaded = true;
   }

   m_distance_calculator = Wgs84AlongPathDistanceCalculator(ownship_kinetic_trajectory_predictor.GetHorizontalPath(),
                                                       TrajectoryIndexProgressionDirection::DECREMENTING);

   const Units::Speed initial_speed = Units::MetersPerSecondSpeed(
      ownship_kinetic_trajectory_predictor.GetVertPredictor()->GetVerticalPath().cas_mps.back());
   // For developing Wgs84
   //const Units::Speed initial_speed = Units::MetersPerSecondSpeed(170);
   m_deceleration_phase_target_ias = initial_speed - m_deceleration_phase_delta_ias;
   m_acceleration_phase_target_ias = m_deceleration_phase_target_ias + m_acceleration_phase_delta_ias;
   m_acceleration_phase_complete = false;
   m_acceleration_target_achieved = false;
   m_acceleration_phase_hold_duration = 10;
   m_acceleration_phase_count = 0;
   m_pilot_delayed_speeds.clear();
   if (m_pilot_delay.IsPilotDelayOn()) {
      const auto delay_params = m_pilot_delay.GetPilotDelayParameters();
      const auto delay_mean = Units::SecondsTime(delay_params.first).value() - 1.0;
      for (int i = 0; i < delay_mean; ++i) {
         m_pilot_delayed_speeds.push_back(Units::zero());
      }
      m_pilot_delayed_speeds.push_back(initial_speed);
   } else {
      m_pilot_delayed_speeds.push_back(initial_speed);
   }
}

aaesim::open_source::Guidance TestVectorSpeedControl::Update(const aaesim::open_source::Guidance &prevguidance,
                                        const aaesim::open_source::DynamicsState &dynamicsstate,
                                        const interval_management::AircraftState &owntruthstate,
                                        const interval_management::AircraftState &targettruthstate,
                                        const vector<interval_management::AircraftState> &targethistory) {

   aaesim::open_source::Guidance return_guidance = prevguidance;
   const double current_time = owntruthstate.GetTimeStamp().value();
   Units::Speed new_ias_command = prevguidance.m_ias_command;
   Units::Speed new_delayed_ias_command = prevguidance.m_ias_command;

   if (current_time < m_deceleration_start_time_sec) {
      return_guidance.SetValid(false);
   } else if (current_time >= m_deceleration_start_time_sec && current_time <= m_acceleration_start_time_sec) {
      new_ias_command = m_deceleration_phase_target_ias;
      return_guidance.SetValid(true);
   } else if (current_time > 240 and !m_acceleration_phase_complete) {
      new_ias_command = m_acceleration_phase_target_ias;
      if (!m_acceleration_target_achieved) {
         const Units::Speed tolerance = Units::KnotsSpeed(1.0);
         m_acceleration_target_achieved =
               dynamicsstate.v_indicated_airspeed > (m_acceleration_phase_target_ias - tolerance);
      }
      m_acceleration_phase_complete =
            m_acceleration_target_achieved && m_acceleration_phase_count > m_acceleration_phase_hold_duration;
      if (m_acceleration_target_achieved) {
         m_acceleration_phase_count++;
      }
      return_guidance.SetValid(true);
   } else {
      // operation complete
      m_ownship_kinematic_dtg_to_ptp = Units::NegInfinity();
      return_guidance.SetValid(false);
   }

   // Modify pilot delay queue
   m_pilot_delayed_speeds.pop_front();
   m_pilot_delayed_speeds.push_back(new_ias_command);
   new_delayed_ias_command = m_pilot_delayed_speeds.front();

   if (return_guidance.IsValid()) {
      return_guidance.m_ias_command = new_delayed_ias_command;

      // Output only
      {
         StandardAtmosphere m_standard_atmosphere = StandardAtmosphere(Units::CelsiusTemperature(0.));
         Units::Speed tas_command = m_standard_atmosphere.CAS2TAS(new_ias_command,
                                                                  Units::FeetLength(owntruthstate.m_z));
         Units::Length ownship_true_dtg;
         m_distance_calculator.CalculateAlongPathDistanceFromPosition(Units::FeetLength(owntruthstate.m_x),
                                                                      Units::FeetLength(owntruthstate.m_y),
                                                                      ownship_true_dtg);
         InternalObserver::getInstance()->IM_command_output(owntruthstate.GetId(),
                                                            owntruthstate.GetTimeStamp().value(),
                                                            owntruthstate.m_z,
                                                            Units::MetersPerSecondSpeed(
                                                                  dynamicsstate.v_true_airspeed).value(),
                                                            Units::MetersPerSecondSpeed(
                                                                  owntruthstate.GetGroundSpeed()).value(),
                                                            Units::MetersPerSecondSpeed(new_ias_command).value(),
                                                            Units::MetersPerSecondSpeed(
                                                                  new_delayed_ias_command).value(),
                                                            Units::MetersPerSecondSpeed(tas_command).value(),
                                                            0.0,
                                                            0.0,
                                                            Units::MetersLength(-ownship_true_dtg).value(),
                                                            Units::MetersLength(ownship_true_dtg).value());
      }
   }

   return return_guidance;
}

void TestVectorSpeedControl::SetAssignedSpacingGoal(const IMClearance &clearance) {
    // leave blank
}

const double TestVectorSpeedControl::GetSpacingError() const {
   return UNDEFINED_INTERVAL;
}

bool TestVectorSpeedControl::load(DecodedStream *input) {
    set_stream(input);

    ResetDefaults();

    register_var("deceleration_phase_delta_ias", &m_deceleration_phase_delta_ias, false);
    register_var("acceleration_phase_delta_ias", &m_acceleration_phase_delta_ias, false);
    register_var("deceleration_start_time_sec", &m_deceleration_start_time_sec, false);
    register_var("acceleration_start_time_sec", &m_acceleration_start_time_sec, false);

    // Complete parameter loading.
    m_loaded = complete();

    return m_loaded;
}
void TestVectorSpeedControl::ResetDefaults() {
   m_deceleration_start_time_sec = DEFAULT_DECELERATION_START_TIME_SEC;
   m_acceleration_start_time_sec = DEFAULT_ACCELERATION_START_TIME_SEC;
   m_deceleration_phase_delta_ias = Units::KnotsSpeed(DEFAULT_DECELERATION_DELTA_IAS);
   m_acceleration_phase_delta_ias = Units::KnotsSpeed(DEFAULT_ACCELERATION_DELTA_IAS);
}
