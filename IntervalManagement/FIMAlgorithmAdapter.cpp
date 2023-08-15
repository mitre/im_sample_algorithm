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

#include "imalgs/FIMAlgorithmAdapter.h"

#include "imalgs/IMUtils.h"
#include "imalgs/FIMAlgorithmInitializer.h"

using namespace interval_management::open_source;

interval_management::open_source::FIMAlgorithmAdapter::FIMAlgorithmAdapter(std::shared_ptr<IMAlgorithm> im_algorithm,
                                                                           IMUtils::IMAlgorithmTypes algorithm_type)
   : m_im_algorithm(nullptr),
     m_im_algorithm_type(IMUtils::IMAlgorithmTypes::NONE),
     m_assap(nullptr),
     m_current_guidance_phase(aaesim::open_source::GuidanceFlightPhase::TAKEOFF_ROLL),
     m_target_history(),
     m_initialized(false) {
   m_im_algorithm = im_algorithm;
   m_im_algorithm_type = algorithm_type;
}

void interval_management::open_source::FIMAlgorithmAdapter::Initialize(
      aaesim::open_source::FlightDeckApplicationInitializer &initializer_visitor) {
   auto ownship_intent_from_clearance = m_im_algorithm->GetClearance().GetOwnshipIntent();
   if (ownship_intent_from_clearance.has_value()) {
      initializer_visitor.fms_prediction_paramters.ownship_aircraft_intent = ownship_intent_from_clearance.value();
   }
   m_im_algorithm->ValidateClearance(initializer_visitor.fms_prediction_paramters.ownship_aircraft_intent,
                                     m_im_algorithm_type);
   try {
      interval_management::open_source::FIMAlgorithmInitializer initializer =
            static_cast<interval_management::open_source::FIMAlgorithmInitializer &>(initializer_visitor);
      initializer.Initialize(this);
      m_assap = initializer.surveillance_processor;
      m_initialized = true;
   } catch (std::exception &e) {
      static const std::string error_message(
            "Developer Error: The wrong concrete instance of FlightDeckApplicationInitializer was provided. Expecting "
            "interval_management::open_source::FIMAlgorithmInitializer");
      throw std::runtime_error(error_message);
   }
}

aaesim::open_source::Guidance interval_management::open_source::FIMAlgorithmAdapter::Update(
      const aaesim::open_source::SimulationTime &simtime, const aaesim::open_source::Guidance &current_guidance,
      const aaesim::open_source::DynamicsState &dynamics_state,
      const aaesim::open_source::AircraftState &own_truth_state) {

   m_current_guidance_phase = current_guidance.m_active_guidance_phase;
   if (m_current_guidance_phase != aaesim::open_source::GuidanceFlightPhase::CRUISE_DESCENT) return current_guidance;

   UpdateTargetHistory(simtime);

   aaesim::open_source::Guidance im_algorithm_guidance = current_guidance;
   im_algorithm_guidance.SetValid(false);

   aaesim::open_source::AircraftState synced_target_state = m_assap->Update(
         own_truth_state, m_assap->GetAdsbReceiver()->GetCurrentADSBReport(GetImClearance().GetTargetId()));

   if (m_initialized && !m_im_algorithm->IsImOperationComplete()) {
      if (im_algorithm_guidance.GetSelectedSpeed().GetSpeedType() == UNSPECIFIED_SPEED) {
         im_algorithm_guidance.SetSelectedSpeed(
               aaesim::open_source::AircraftSpeed::OfIndicatedAirspeed(Units::KnotsSpeed(60)));
      }

      return m_im_algorithm->Update(
            im_algorithm_guidance, dynamics_state, IMUtils::ConvertToIntervalManagementAircraftState(own_truth_state),
            IMUtils::ConvertToIntervalManagementAircraftState(synced_target_state), m_target_history);
   }
   return im_algorithm_guidance;
}

bool interval_management::open_source::FIMAlgorithmAdapter::IsActive() const {
   return !m_im_algorithm->IsImOperationComplete();
}

void interval_management::open_source::FIMAlgorithmAdapter::UpdateTargetHistory(
      const aaesim::open_source::SimulationTime &simtime) {
   std::vector<Sensor::ADSB::ADSBSVReport> recent_reports =
         m_assap->GetAdsbReceiver()->GetReportsReceivedByTime(simtime);
   if (recent_reports.empty()) {
      return;
   }

   for (const auto &adsb_sv_report : recent_reports) {
      if (adsb_sv_report.GetId() == GetImClearance().GetTargetId()) {

         const aaesim::open_source::AircraftState ads_b_state =
               aaesim::open_source::AircraftState::CreateFromADSBReport(adsb_sv_report);
         interval_management::open_source::AircraftState imstate =
               IMUtils::ConvertToIntervalManagementAircraftState(ads_b_state);
         if (imstate.GetId() > IMUtils::UNINITIALIZED_AIRCRAFT_ID && imstate.GetTimeStamp() >= Units::zero()) {
            imstate.m_distance_to_go_meters = Units::MetersLength(m_im_algorithm->GetTargetDtgToLastWaypoint()).value();
            m_target_history.push_back(imstate);
         }
      }
   }
}
