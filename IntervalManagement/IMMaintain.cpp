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

#include "imalgs/IMMaintain.h"

using namespace interval_management::open_source;

log4cplus::Logger IMMaintain::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("IMMaintain"));

const Units::HertzFrequency IMMaintain::MAINTAIN_CONTROL_GAIN_DEFAULT(.008);

IMMaintain::IMMaintain() { m_maintain_control_gain = MAINTAIN_CONTROL_GAIN_DEFAULT; }

IMMaintain::IMMaintain(const IMMaintain &obj) { Copy(obj); }

void IMMaintain::ResetDefaults() {
   IMAlgorithm::ResetDefaults();

   if (m_maintain_control_gain != MAINTAIN_CONTROL_GAIN_DEFAULT) {
      LOG4CPLUS_WARN(m_logger, "mMaintainControlGain reset to " << MAINTAIN_CONTROL_GAIN_DEFAULT << RESET_MSG);
      m_maintain_control_gain = MAINTAIN_CONTROL_GAIN_DEFAULT;
   }
}

void IMMaintain::Copy(const IMMaintain &obj) {
   IMAlgorithm::Copy(obj);

   m_maintain_control_gain = obj.m_maintain_control_gain;
   m_ownship_decrementing_distance_calculator = obj.m_ownship_decrementing_distance_calculator;
   m_ownship_distance_calculator = obj.m_ownship_distance_calculator;
}

void IMMaintain::IterationReset() { IMAlgorithm::IterationReset(); }

void IMMaintain::InitializeScenario(IMAchieve *obj, const Units::Frequency maintain_control_gain) {
   m_maintain_control_gain = maintain_control_gain;
}

void IMMaintain::Prepare(Units::Speed previous_im_speed_command, Units::Speed previous_ias_command,
                         interval_management::open_source::FIMSpeedLimiter speed_limiter, double previous_mach_command,
                         const aaesim::open_source::EuclideanTrajectoryPredictor &ownship_trajectory_predictor,
                         const AlongPathDistanceCalculator &im_distance_calculator,
                         const std::vector<interval_management::open_source::AircraftState> &target_adsb_track_history,
                         const IMClearance &im_clearance,
                         const std::vector<interval_management::open_source::FIMSpeedLimiter::RfLegLimit> &rf_limits) {
   m_im_clearance = im_clearance;
   m_previous_reference_im_speed_command_tas = previous_im_speed_command;
   m_previous_im_speed_command_ias = previous_ias_command;
   m_previous_reference_im_speed_command_mach = previous_mach_command;
   m_ownship_decrementing_distance_calculator = AlongPathDistanceCalculator(
         ownship_trajectory_predictor.GetHorizontalPath(), TrajectoryIndexProgressionDirection::DECREMENTING);
   m_ownship_distance_calculator = im_distance_calculator;
   m_speed_limiter = speed_limiter;

   m_speed_limiter.ClearRfLegSpeedLimits();
   for (const auto &rf_limit : rf_limits) {
      m_speed_limiter.AddRfLegSpeedLimit(rf_limit.distance_to_go, rf_limit.upper_ias_limit);
   }
}

void IMMaintain::DumpParameters(const std::string &parameters_to_print) {
   IMAlgorithm::DumpParameters(parameters_to_print);
}

Units::Frequency IMMaintain::GetMaintainControlGain() const { return m_maintain_control_gain; }

void IMMaintain::SetAssignedSpacingGoal(const IMClearance &clearance) {
   // Developers: The class is required to implement this method, but there is nothing to do here. The maintain
   // algorithm doesn't explicitly need the ASG value so long as the update method is called with the correct
   // information (that has taken ASG into account).
   //
   // Don't do anything here.
}

const double IMMaintain::GetSpacingError() const {
   // Developers: The class is required to implement this method, but there is nothing to do here. The maintain
   // algorithm doesn't explicitly know the ASG value so it can't report a spacing error. Use the achieve class
   // for all spacing error calculations.
   return -INFINITY;
}
