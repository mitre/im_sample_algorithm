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

#pragma once

#include "public/PredictionFileBase.h"
#include "public/FlightDeckApplication.h"

namespace interval_management::open_source {
class PredictionFileKinematic final : private PredictionFileBase, public OutputHandler {
  public:
   PredictionFileKinematic();

   void Finish() override;

   void Gather(const int iteration, const Units::Time simulation_time, std::string aircraft_id,
               const std::shared_ptr<const aaesim::open_source::FlightDeckApplication> &flightdeck_application);

  private:
   const bool TrajectoryWasRegenerated(const std::vector<PredictionData> &prediction_data_single_acid,
                                       const VerticalPath vertical_path, const int iteration,
                                       const PredictionData::DataSource source) const;

   std::map<std::string, std::vector<PredictionData> > algorithm_prediction_ownship;
   std::map<std::string, std::vector<PredictionData> > algorithm_prediction_target;

   static log4cplus::Logger logger;
};
}  // namespace interval_management::open_source