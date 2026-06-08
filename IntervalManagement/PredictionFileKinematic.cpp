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

#include "imalgs/PredictionFileKinematic.h"

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "imalgs/FIMAlgorithmAdapter.h"
#include "imalgs/IMKinematicAchieve.h"
#include "public/CoreUtils.h"

using namespace interval_management::open_source;

log4cplus::Logger PredictionFileKinematic::logger =
      log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("PredictionFileKinematic"));

PredictionFileKinematic::PredictionFileKinematic()
   : OutputHandler("", "_predicted_kinematic_trajectory.csv"),
     algorithm_prediction_ownship(),
     algorithm_prediction_target() {}

void PredictionFileKinematic::Finish() {
   if (!algorithm_prediction_ownship.empty()) {
      os.open(filename.c_str());

      if (!os.is_open()) {
         std::string error_msg = "Cannot open " + filename;
         LOG4CPLUS_FATAL(logger, error_msg);
         return;
      }

      os.set_delimiter(',', ",");

      // Write header.
      os << "iteration";
      os << "acid";
      os << "source";
      os << "time_s";
      os << "altitude_m";
      os << "ias_mps";
      os << "ttg_s";
      os << "dtg_m";
      os << "gs_mps";
      os << "tas_mps";
      os << "vwe_mps";
      os << "vwn_mps";
      os << "algorithm";
      os << "flap_setting";
      os << NEWLINE;

      os.set_precision(10);

      auto data_writer = [this](const std::pair<std::string, std::vector<PredictionFileBase::PredictionData>> &data) {
         for (auto ix = 0; ix < data.second.size(); ++ix) {
            os << data.second[ix].iteration_number;
            os << data.second[ix].acid;
            os << data.second[ix].source;
            os << Units::SecondsTime(data.second[ix].simulation_time).value();
            os << Units::MetersLength(data.second[ix].altitude).value();
            os << Units::MetersPerSecondSpeed(data.second[ix].IAS).value();
            os << Units::SecondsTime(data.second[ix].time_to_go).value();
            os << Units::MetersLength(data.second[ix].distance_to_go).value();
            os << Units::MetersPerSecondSpeed(data.second[ix].GS).value();
            os << Units::MetersPerSecondSpeed(data.second[ix].TAS).value();
            os << data.second[ix].VwePred.value();
            os << data.second[ix].VwnPred.value();
            os << data.second[ix].algorithm;
            os << data.second[ix].flap_setting;
            os << NEWLINE;
         }
      };
      std::for_each(algorithm_prediction_ownship.cbegin(), algorithm_prediction_ownship.cend(), data_writer);
      std::for_each(algorithm_prediction_target.cbegin(), algorithm_prediction_target.cend(), data_writer);

      os.close();
   }
   m_finished = true;

   algorithm_prediction_ownship.clear();
   algorithm_prediction_target.clear();
}

void PredictionFileKinematic::Gather(
      const int iteration, const Units::Time simulation_time, std::string aircraft_id,
      const std::shared_ptr<const aaesim::open_source::FlightDeckApplication> &flightdeck_application) {
   const bool is_im_application =
         CoreUtils::InstanceOf<interval_management::open_source::FIMAlgorithmAdapter>(flightdeck_application.get());
   if (is_im_application) {
      std::shared_ptr<const interval_management::open_source::FIMAlgorithmAdapter> im_algorithm_adapter =
            std::dynamic_pointer_cast<const interval_management::open_source::FIMAlgorithmAdapter>(
                  flightdeck_application);
      std::shared_ptr<IMKinematicAchieve> im_kinematic_achieve =
            std::dynamic_pointer_cast<IMKinematicAchieve>(im_algorithm_adapter->GetImAlgorithm());
      IMUtils::IMAlgorithmTypes im_algorithm_type = im_algorithm_adapter->GetImAlgorithmType();

      const VerticalPath *ownship_vert_path{};
      const VerticalPath *target_vert_path{};

      switch (im_algorithm_type) {
         case IMUtils::IMAlgorithmTypes::TIMEBASEDACHIEVE:
         case IMUtils::IMAlgorithmTypes::TIMEBASEDACHIEVEMUTABLEASG:
         case IMUtils::IMAlgorithmTypes::DISTANCEBASEDACHIEVE:
            ownship_vert_path =
                  &im_kinematic_achieve->GetOwnshipKinematicPredictor().GetVerticalPredictor()->GetVerticalPath();
            target_vert_path =
                  &im_kinematic_achieve->GetTargetKinematicPredictor().GetVerticalPredictor()->GetVerticalPath();
            break;
         case IMUtils::IMAlgorithmTypes::NONE:
         case IMUtils::IMAlgorithmTypes::TESTSPEEDCONTROL:
         case IMUtils::IMAlgorithmTypes::KINETICACHIEVE:
         case IMUtils::IMAlgorithmTypes::KINETICTARGETACHIEVE:
            break;
         default:
            const std::string msg =
                  "Encountered unknown application type: " + IMUtils::algorithm_type_dictionary.at(im_algorithm_type);
            throw std::runtime_error(msg);
      }

      IMAlgorithm::FlightStage flight_stage = im_kinematic_achieve->GetFlightStage();

      bool im_operation_is_active =
            (flight_stage == IMAlgorithm::FlightStage::ACHIEVE || flight_stage == IMAlgorithm::FlightStage::MAINTAIN) &&
            flightdeck_application->IsActive();

      if (im_operation_is_active && im_kinematic_achieve->IsNewTrajectoryPredictionAvailable()) {
         std::vector<PredictionData> &ownship_prediction_data = algorithm_prediction_ownship[aircraft_id];
         if (TrajectoryWasRegenerated(ownship_prediction_data, *ownship_vert_path, iteration,
                                      PredictionData::DataSource::IM_ALGO_OWNSHIP)) {
            auto prediction_data_vector =
                  ExtractPredictionDataFromVerticalPath(iteration, simulation_time, aircraft_id, *ownship_vert_path,
                                                        PredictionData::DataSource::IM_ALGO_OWNSHIP);
            ownship_prediction_data.insert(ownship_prediction_data.end(),
                                           std::make_move_iterator(prediction_data_vector.begin()),
                                           std::make_move_iterator(prediction_data_vector.end()));
         }

         std::vector<PredictionData> &target_prediction_data = algorithm_prediction_target[aircraft_id];
         if (TrajectoryWasRegenerated(target_prediction_data, *target_vert_path, iteration,
                                      PredictionData::DataSource::IM_ALGO_TARGET)) {
            auto prediction_data_vector =
                  ExtractPredictionDataFromVerticalPath(iteration, simulation_time, aircraft_id, *target_vert_path,
                                                        PredictionData::DataSource::IM_ALGO_TARGET);
            target_prediction_data.insert(target_prediction_data.end(),
                                          std::make_move_iterator(prediction_data_vector.begin()),
                                          std::make_move_iterator(prediction_data_vector.end()));
         }
      }
   }
}

const bool PredictionFileKinematic::TrajectoryWasRegenerated(
      const std::vector<PredictionData> &prediction_data_single_acid, const VerticalPath &vertical_path,
      const int iteration, const PredictionData::DataSource source) const {
   if (!prediction_data_single_acid.empty()) {
      bool trajectory_was_regenerated = prediction_data_single_acid.back().iteration_number != iteration;
      trajectory_was_regenerated = trajectory_was_regenerated || prediction_data_single_acid.back().source != source;
      trajectory_was_regenerated =
            trajectory_was_regenerated ||
            prediction_data_single_acid.back().time_to_go != Units::SecondsTime(vertical_path.time_to_go_sec.back());
      trajectory_was_regenerated =
            trajectory_was_regenerated ||
            prediction_data_single_acid.back().altitude != Units::MetersLength(vertical_path.altitude_m.back());
      trajectory_was_regenerated =
            trajectory_was_regenerated ||
            prediction_data_single_acid.back().IAS != Units::MetersPerSecondSpeed(vertical_path.cas_mps.back());

      return trajectory_was_regenerated;
   }
   return true;
}
