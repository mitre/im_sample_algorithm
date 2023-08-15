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

#include "imalgs/FIMAlgorithmInitializer.h"

using namespace interval_management::open_source;

interval_management::open_source::FIMAlgorithmInitializer::FIMAlgorithmInitializer(
      const FIMAlgorithmInitializer::Builder *builder) {
   fms_prediction_paramters = builder->GetFmsPredictionParameters();
   performance_parameters = builder->GetPerformanceParameters();
   surveillance_processor = builder->GetSurveillanceProcessor();
}

void interval_management::open_source::FIMAlgorithmInitializer::Initialize(
      interval_management::open_source::FIMAlgorithmAdapter *algorithm) {
   if (auto kinematic_algorithm =
             std::dynamic_pointer_cast<interval_management::open_source::IMTimeBasedAchieveMutableASG>(
                   algorithm->GetImAlgorithm())) {
      Initialize(kinematic_algorithm.get());
      return;
   }
   if (auto time_based_algorithm = std::dynamic_pointer_cast<interval_management::open_source::IMTimeBasedAchieve>(
             algorithm->GetImAlgorithm())) {
      Initialize(time_based_algorithm.get());
      return;
   }
   if (auto dist_based_algorithm = std::dynamic_pointer_cast<interval_management::open_source::IMDistBasedAchieve>(
             algorithm->GetImAlgorithm())) {
      Initialize(dist_based_algorithm.get());
      return;
   }
}

void interval_management::open_source::FIMAlgorithmInitializer::Initialize(
      interval_management::open_source::IMKinematicAchieve *kinematic_algorithm) {
   kinematic_algorithm->Initialize(BuildOwnshipPredictionParameters(), fms_prediction_paramters.ownship_aircraft_intent,
                                   fms_prediction_paramters.weather_prediction);
}

void interval_management::open_source::FIMAlgorithmInitializer::Initialize(
      interval_management::open_source::IMTimeBasedAchieveMutableASG *test_vector_algorithm) {
   test_vector_algorithm->Initialize(BuildOwnshipPredictionParameters(),
                                     fms_prediction_paramters.ownship_aircraft_intent,
                                     fms_prediction_paramters.weather_prediction);
}

void interval_management::open_source::FIMAlgorithmInitializer::Initialize(
      interval_management::open_source::IMTimeBasedAchieve *time_achieve_algorithm) {
   time_achieve_algorithm->Initialize(BuildOwnshipPredictionParameters(),
                                      fms_prediction_paramters.ownship_aircraft_intent,
                                      fms_prediction_paramters.weather_prediction);
}

void interval_management::open_source::FIMAlgorithmInitializer::Initialize(
      interval_management::open_source::IMDistBasedAchieve *dist_achieve_algorithm) {
   dist_achieve_algorithm->Initialize(BuildOwnshipPredictionParameters(),
                                      fms_prediction_paramters.ownship_aircraft_intent,
                                      fms_prediction_paramters.weather_prediction);
}

IMAlgorithm::OwnshipPredictionParameters
      interval_management::open_source::FIMAlgorithmInitializer::BuildOwnshipPredictionParameters() const {
   IMAlgorithm::OwnshipPredictionParameters ownship_prediction_parameters;
   ownship_prediction_parameters.aerodynamics = performance_parameters.aerodynamics;
   ownship_prediction_parameters.flap_speeds = performance_parameters.flap_speeds;
   ownship_prediction_parameters.flight_envelope = performance_parameters.flight_envelope;
   ownship_prediction_parameters.mass_data = performance_parameters.mass_data;
   ownship_prediction_parameters.expected_cruise_altitude = fms_prediction_paramters.expected_cruise_altitude;
   ownship_prediction_parameters.maximum_allowable_bank_angle = fms_prediction_paramters.maximum_allowable_bank_angle;
   ownship_prediction_parameters.transition_altitude = fms_prediction_paramters.transition_altitude;
   ownship_prediction_parameters.transition_ias = fms_prediction_paramters.transition_ias;
   ownship_prediction_parameters.transition_mach = fms_prediction_paramters.transition_mach;
   return ownship_prediction_parameters;
}

const interval_management::open_source::FIMAlgorithmInitializer
      interval_management::open_source::FIMAlgorithmInitializer::Builder::Build() const {
   return interval_management::open_source::FIMAlgorithmInitializer(this);
}

interval_management::open_source::FIMAlgorithmInitializer::Builder *
      interval_management::open_source::FIMAlgorithmInitializer::Builder::AddOwnshipPerformanceParameters(
            aaesim::open_source::OwnshipPerformanceParameters performance_parameters) {
   m_performance_parameters = performance_parameters;
   return this;
}

interval_management::open_source::FIMAlgorithmInitializer::Builder *
      interval_management::open_source::FIMAlgorithmInitializer::Builder::AddOwnshipFmsPredictionParameters(
            aaesim::open_source::OwnshipFmsPredictionParameters prediction_parameters) {
   m_prediction_parameters = prediction_parameters;
   return this;
}

interval_management::open_source::FIMAlgorithmInitializer::Builder *
      interval_management::open_source::FIMAlgorithmInitializer::Builder::AddSurveillanceProcessor(
            std::shared_ptr<const aaesim::open_source::ASSAP> processor) {
   m_surveillance_processor = processor;
   return this;
}
