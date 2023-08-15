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

#include "imalgs/PredictionFileKinematic.h"

#include "imalgs/IMKinematicAchieve.h"
#include "imalgs/FIMAlgorithmAdapter.h"
#include "public/CoreUtils.h"
#include <cmath>

using namespace interval_management::open_source;

log4cplus::Logger PredictionFileKinematic::logger =
      log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("PredictionFileKinematic"));

PredictionFileKinematic::PredictionFileKinematic()
   : OutputHandler("", "-Predicted_Kinematic_Trajectory.csv"),
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
      os.flush();

      // Important for outputting double data.
      os.set_precision(10);

      for (auto id = algorithm_prediction_ownship.cbegin(); id != algorithm_prediction_ownship.cend(); ++id) {
         if (id->second.size() == 0) {
            LOG4CPLUS_DEBUG(logger,
                            "No predicted data will be reported for acid " << id->first << ". No algorithm available.");
         }
         for (auto ix = 0; ix < id->second.size(); ++ix) {
            os << id->second[ix].iteration_number;
            os << id->second[ix].acid;
            os << id->second[ix].source;
            os << Units::SecondsTime(id->second[ix].simulation_time).value();
            os << Units::MetersLength(id->second[ix].altitude).value();
            os << Units::MetersPerSecondSpeed(id->second[ix].IAS).value();
            os << Units::SecondsTime(id->second[ix].time_to_go).value();
            os << Units::MetersLength(id->second[ix].distance_to_go).value();
            os << Units::MetersPerSecondSpeed(id->second[ix].GS).value();
            os << Units::MetersPerSecondSpeed(id->second[ix].TAS).value();
            os << id->second[ix].VwePred.value();
            os << id->second[ix].VwnPred.value();
            os << id->second[ix].algorithm;
            os << id->second[ix].flap_setting;

            os << NEWLINE;
            os.flush();
         }
      }

      for (auto id = algorithm_prediction_target.cbegin(); id != algorithm_prediction_target.cend(); ++id) {
         for (auto ix = 0; ix < id->second.size(); ++ix) {
            os << id->second[ix].iteration_number;
            os << id->second[ix].acid;
            os << id->second[ix].source;
            os << Units::SecondsTime(id->second[ix].simulation_time).value();
            os << Units::MetersLength(id->second[ix].altitude).value();
            os << Units::MetersPerSecondSpeed(id->second[ix].IAS).value();
            os << Units::SecondsTime(id->second[ix].time_to_go).value();
            os << Units::MetersLength(id->second[ix].distance_to_go).value();
            os << Units::MetersPerSecondSpeed(id->second[ix].GS).value();
            os << Units::MetersPerSecondSpeed(id->second[ix].TAS).value();
            os << id->second[ix].VwePred.value();
            os << id->second[ix].VwnPred.value();
            os << id->second[ix].algorithm;
            os << id->second[ix].flap_setting;

            os << NEWLINE;
            os.flush();
         }
      }

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

      const VerticalPath *ownship_vert_path;
      const VerticalPath *target_vert_path;

      switch (im_algorithm_type) {
         case IMUtils::IMAlgorithmTypes::TIMEBASEDACHIEVE:
         case IMUtils::IMAlgorithmTypes::TIMEBASEDACHIEVEMUTABLEASG:
         case IMUtils::IMAlgorithmTypes::DISTANCEBASEDACHIEVE:
            ownship_vert_path =
                  &im_kinematic_achieve->GetOwnshipKinematicPredictor().GetVerticalPredictor()->GetVerticalPath();
            target_vert_path =
                  &im_kinematic_achieve->GetTargetKinematicPredictor().GetVerticalPredictor()->GetVerticalPath();
            break;
         case IMUtils::IMAlgorithmTypes::RTA:
            ownship_vert_path =
                  &im_kinematic_achieve->GetOwnshipKinematicPredictor().GetVerticalPredictor()->GetVerticalPath();
            break;
         case IMUtils::IMAlgorithmTypes::NONE:
         case IMUtils::IMAlgorithmTypes::TESTSPEEDCONTROL:
         case IMUtils::IMAlgorithmTypes::KINETICACHIEVE:
         case IMUtils::IMAlgorithmTypes::KINETICTARGETACHIEVE:
         case IMUtils::IMAlgorithmTypes::RTA_TOAC_NOT_IMALGORITHM:
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

      if (im_operation_is_active) {
         std::vector<PredictionData> &ownship_prediction_data = algorithm_prediction_ownship[aircraft_id];

         if (TrajectoryWasRegenerated(ownship_prediction_data, *ownship_vert_path, iteration,
                                      PredictionData::DataSource::IM_ALGO_OWNSHIP)) {

            if (im_algorithm_type != IMUtils::IMAlgorithmTypes::RTA) {
               std::vector<PredictionData> &target_prediction_data = algorithm_prediction_target[aircraft_id];
               if (TrajectoryWasRegenerated(target_prediction_data, *target_vert_path, iteration,
                                            PredictionData::DataSource::IM_ALGO_TARGET)) {

                  auto prediction_data_vector = ExtractPredictionDataFromVerticalPath(
                        iteration, simulation_time, aircraft_id, *ownship_vert_path,
                        PredictionData::DataSource::IM_ALGO_OWNSHIP);
                  ownship_prediction_data.insert(ownship_prediction_data.end(),
                                                 std::make_move_iterator(prediction_data_vector.begin()),
                                                 std::make_move_iterator(prediction_data_vector.end()));
                  prediction_data_vector.clear();
                  prediction_data_vector = ExtractPredictionDataFromVerticalPath(
                        iteration, simulation_time, aircraft_id, *target_vert_path,
                        PredictionData::DataSource::IM_ALGO_TARGET);
                  target_prediction_data.insert(target_prediction_data.end(),
                                                std::make_move_iterator(prediction_data_vector.begin()),
                                                std::make_move_iterator(prediction_data_vector.end()));
               }
            } else {
               auto prediction_data_vector =
                     ExtractPredictionDataFromVerticalPath(iteration, simulation_time, aircraft_id, *ownship_vert_path,
                                                           PredictionData::DataSource::IM_ALGO_OWNSHIP);
               ownship_prediction_data.insert(ownship_prediction_data.end(),
                                              std::make_move_iterator(prediction_data_vector.begin()),
                                              std::make_move_iterator(prediction_data_vector.end()));
            }
         }
      }
   }
}

const bool PredictionFileKinematic::TrajectoryWasRegenerated(
      const std::vector<PredictionData> &prediction_data_single_acid, const VerticalPath vertical_path,
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