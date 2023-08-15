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

#include "imalgs/IMAchieve.h"

#include "public/AircraftCalculations.h"
#include "public/Environment.h"
#include "imalgs/MOPSPredictedWindEvaluatorVersion1.h"
#include "imalgs/MOPSPredictedWindEvaluatorVersion2.h"

using namespace std;
using namespace interval_management::open_source;

log4cplus::Logger IMAchieve::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("IMAchieve"));

const shared_ptr<aaesim::open_source::PredictedWindEvaluator> IMAchieve::m_predicted_wind_evaluator(
      MOPSPredictedWindEvaluatorVersion2::GetInstance());

// Do not change these values without discussing with Lesley Weitz.
const Units::Angle IMAchieve::TOLERANCE_ANGLE(Units::DegreesAngle(15.0));

IMAchieve::IMAchieve() {
   m_transitioned_to_maintain = false;
   m_achieve_by_point.assign("");
   m_within_error_threshold = false;
   IterClearIMAch();
}

IMAchieve::IMAchieve(const IMAchieve &obj) { Copy(obj); }

void IMAchieve::IterClearIMAch() {
   m_transitioned_to_maintain = false;
   m_achieve_by_point.assign("");
}

void IMAchieve::Copy(const IMAchieve &obj) {
   IMAlgorithm::Copy(obj);
   m_transitioned_to_maintain = obj.m_transitioned_to_maintain;
   m_achieve_control_gain = obj.m_achieve_control_gain;
   m_maintain_control_gain = obj.m_maintain_control_gain;
   m_threshold_flag = obj.m_threshold_flag;
   m_time_threshold = obj.m_time_threshold;
   m_received_one_valid_target_state = obj.m_received_one_valid_target_state;
   m_within_error_threshold = obj.m_within_error_threshold;
   m_achieve_by_point = obj.m_achieve_by_point;
}

const bool IMAchieve::WithinErrorThreshold(const Units::Length distance_to_go, const Units::Time ownship_ttg,
                                           const Units::Time reference_ttg) {
   Units::Time ttgdiff = abs(ownship_ttg - reference_ttg);

   m_within_error_threshold = ttgdiff < GetErrorThreshold(distance_to_go);
   return m_within_error_threshold;
}

Units::Time IMAchieve::GetErrorThreshold(Units::Length distance_to_go) {
   Units::Time ethresh = Units::ZERO_TIME;

   if (distance_to_go < m_error_distance) {
      ethresh = m_time_threshold;
   } else {
      ethresh = m_time_threshold + (m_slope * (distance_to_go - m_error_distance));
   }

   return ethresh;
}

void IMAchieve::IterationReset() {
   IMAlgorithm::IterationReset();
   m_received_one_valid_target_state = false;

   IterClearIMAch();
}

aaesim::open_source::Guidance IMAchieve::Update(
      const aaesim::open_source::Guidance &prevguidance, const aaesim::open_source::DynamicsState &dynamicsstate,
      const interval_management::open_source::AircraftState &owntruthstate,
      const interval_management::open_source::AircraftState &targettruthstate,
      const vector<interval_management::open_source::AircraftState> &targethistory) {
   IMAlgorithm::Update(prevguidance, dynamicsstate, owntruthstate, targettruthstate, targethistory);
   if (!m_received_one_valid_target_state) {
      m_received_one_valid_target_state = targettruthstate.GetId() != IMUtils::UNINITIALIZED_AIRCRAFT_ID;
   }

   return prevguidance;
}

void IMAchieve::DumpParameters(const string &parameters_to_print) {
   LOG4CPLUS_DEBUG(IMAchieve::m_logger, "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl);

   LOG4CPLUS_DEBUG(IMAchieve::m_logger, "IMAlgorithm parms for " << parameters_to_print.c_str() << endl << endl);

   IMAlgorithm::DumpParameters(parameters_to_print);

   LOG4CPLUS_DEBUG(IMAchieve::m_logger,
                   "mAchieveControlGain " << Units::HertzFrequency(m_achieve_control_gain).value() << endl);
   LOG4CPLUS_DEBUG(IMAchieve::m_logger,
                   "maintain control gain " << Units::HertzFrequency(m_maintain_control_gain).value() << endl);
   LOG4CPLUS_DEBUG(IMAchieve::m_logger, "mBlendWind " << IsBlendWind() << endl);
   LOG4CPLUS_DEBUG(IMAchieve::m_logger, "threshold flag " << m_threshold_flag << endl);
   LOG4CPLUS_DEBUG(IMAchieve::m_logger, "time threshold " << Units::SecondsTime(m_time_threshold).value() << endl);
   LOG4CPLUS_DEBUG(IMAchieve::m_logger, "ownship achieve-by waypoint " << m_achieve_by_point.c_str() << std::endl);
   LOG4CPLUS_DEBUG(IMAchieve::m_logger, "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl);
}

void IMAchieve::Initialize(const OwnshipPredictionParameters &ownship_prediction_parameters,
                           const AircraftIntent &ownship_aircraft_intent,
                           aaesim::open_source::WeatherPrediction &weather_prediction) {

   IMAlgorithm::Initialize(ownship_prediction_parameters, ownship_aircraft_intent, weather_prediction);

   m_achieve_by_point.assign(m_im_clearance.GetAchieveByPoint());
}
