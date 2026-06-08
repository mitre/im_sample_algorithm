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
// (c) 2026 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "imalgs/FIMAlgorithmAdapter.h"

#include <memory>
#include <string>
#include <vector>

#include "imalgs/FIMAlgorithmInitializer.h"
#include "imalgs/IMUtils.h"
#include "nlohmann/json.hpp"
#include "public/CoreUtils.h"
#include "public/SingleTangentPlaneSequence.h"

using namespace interval_management::open_source;
using json = nlohmann::json;

interval_management::open_source::FIMAlgorithmAdapter::FIMAlgorithmAdapter(std::shared_ptr<IMAlgorithm> im_algorithm,
                                                                           IMUtils::IMAlgorithmTypes algorithm_type)
   : m_im_algorithm(im_algorithm), m_im_algorithm_type(algorithm_type) {}

void interval_management::open_source::FIMAlgorithmAdapter::Initialize(
      aaesim::open_source::FlightDeckApplicationInitializer &initializer_visitor) {
   auto ownship_intent_from_clearance = m_im_algorithm->GetClearance().GetOwnshipIntent();
   if (ownship_intent_from_clearance.has_value()) {
      initializer_visitor.fms_prediction_parameters.fms_intent = ownship_intent_from_clearance.value();
   }
   m_im_algorithm->ValidateClearance(initializer_visitor.fms_prediction_parameters.fms_intent, m_im_algorithm_type);
   initializer_visitor.fms_prediction_parameters.fms_intent =
         AircraftIntent::CopyAndTrimAfterNamedWaypoint(initializer_visitor.fms_prediction_parameters.fms_intent,
                                                       m_im_algorithm->GetClearance().GetPlannedTerminationPoint());
   auto waypoints =
         CoreUtils::ShortenLongLegs(initializer_visitor.fms_prediction_parameters.fms_intent.GetWaypointList());
   m_position_converter = std::make_unique<SingleTangentPlaneSequence>(waypoints);
   initializer_visitor.position_converter = m_position_converter;

   try {
      interval_management::open_source::FIMAlgorithmInitializer initializer =
            static_cast<interval_management::open_source::FIMAlgorithmInitializer &>(initializer_visitor);
      initializer.Initialize(this);
      m_assap = initializer.surveillance_processor;
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
   aaesim::open_source::Guidance im_algorithm_guidance = current_guidance;
   im_algorithm_guidance.SetValid(false);
   if (current_guidance.m_active_guidance_phase != aaesim::open_source::GuidanceFlightPhase::CRUISE_DESCENT)
      return im_algorithm_guidance;
   if (m_im_algorithm->IsImOperationComplete()) return im_algorithm_guidance;

   UpdateTargetHistory(simtime);
   aaesim::open_source::AircraftState synced_target_state = m_assap->Update(
         own_truth_state, m_assap->GetAdsbReceiver()->GetCurrentADSBReport(GetImClearance().GetTargetId()));

   if (im_algorithm_guidance.GetSelectedSpeed().GetSpeedType() == UNSPECIFIED_SPEED) {
      im_algorithm_guidance.SetSelectedSpeed(
            aaesim::open_source::AircraftSpeed::OfIndicatedAirspeed(Units::KnotsSpeed(60)));
   }
   auto ownship_im_state = ConvertAircraftState(own_truth_state);
   auto target_im_state = ConvertAircraftState(synced_target_state);
   return m_im_algorithm->Update(im_algorithm_guidance, dynamics_state, ownship_im_state, target_im_state,
                                 m_target_history);
}

interval_management::open_source::AircraftState
      interval_management::open_source::FIMAlgorithmAdapter::ConvertAircraftState(
            const aaesim::open_source::AircraftState &state) const {
   if (state.GetTime().value() < 0 || state.GetUniqueId() == IMUtils::UNINITIALIZED_AIRCRAFT_ID)
      return IMUtils::ConvertToIntervalManagementAircraftState(state);

   EarthModel::LocalPositionEnu enu_position;
   m_position_converter->ConvertGeodeticToLocal(
         EarthModel::GeodeticPosition::Of(state.GetLatitude(), state.GetLongitude()), enu_position);
   auto updated_state =
         aaesim::open_source::AircraftState::Builder(state).Position(enu_position.x, enu_position.y)->Build();
   LogAircraftState(updated_state);
   return IMUtils::ConvertToIntervalManagementAircraftState(updated_state);
}

bool interval_management::open_source::FIMAlgorithmAdapter::IsActive() const {
   return !m_im_algorithm->IsImOperationComplete();
}

void interval_management::open_source::FIMAlgorithmAdapter::UpdateTargetHistory(
      const aaesim::open_source::SimulationTime &simtime) {
   std::vector<aaesim::open_source::ADSBSVReport> recent_reports =
         m_assap->GetAdsbReceiver()->GetReportsReceivedByTime(simtime);
   if (recent_reports.empty()) {
      return;
   }

   for (const auto &adsb_sv_report : recent_reports) {
      if (adsb_sv_report.GetId() == GetImClearance().GetTargetId()) {
         if (adsb_sv_report.GetTime() >= Units::zero()) {
            const aaesim::open_source::AircraftState ads_b_state =
                  aaesim::open_source::AircraftState::FromAdsbReport(adsb_sv_report);
            interval_management::open_source::AircraftState imstate = ConvertAircraftState(ads_b_state);
            imstate.m_distance_to_go_meters = Units::MetersLength(m_im_algorithm->GetTargetDtgToLastWaypoint()).value();
            m_target_history.push_back(imstate);
         }
      }
   }
}

void FIMAlgorithmAdapter::LogAircraftState(const aaesim::open_source::AircraftState &state) {
   if (m_logger.getLogLevel() == log4cplus::TRACE_LOG_LEVEL) {
      json j;
      j["acid"] = state.GetUniqueId();
      j["time_sec"] = state.GetTime().value();
      j["position_enu_x_m"] = Units::MetersLength(state.GetPositionEnuX()).value();
      j["position_enu_y_m"] = Units::MetersLength(state.GetPositionEnuY()).value();
      j["altitude_msl_m"] = Units::MetersLength(state.GetAltitudeMsl()).value();
      j["position_lat_deg"] = Units::DegreesAngle(state.GetLatitude()).value();
      j["position_lon_deg"] = Units::DegreesAngle(state.GetLongitude()).value();
      LOG4CPLUS_TRACE(m_logger, j.dump());
   }
}
