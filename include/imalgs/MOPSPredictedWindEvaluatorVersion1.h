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

#include "public/PredictedWindEvaluator.h"

namespace interval_management {
namespace open_source {
class MOPSPredictedWindEvaluatorVersion1 : public aaesim::open_source::PredictedWindEvaluator {
  public:
   const static std::shared_ptr<aaesim::open_source::PredictedWindEvaluator> &GetInstance();

   virtual ~MOPSPredictedWindEvaluatorVersion1();

   /**
    * Employs an algorithm to check the difference between the predicted wind and the
    * provided sensed wind.
    * Returns true if the algorithm shows that the two winds are sufficiently similar. Returns false otherwise.
    *
    * Algorithm designed by Lesley, provided summer 2014.
    *
    * @param acstate and aircraft_state_vector that represents the most recent navigations state
    * @param predictedwindx wind in the x
    * @param predictedwindy wind in the y
    * @author sbowman
    */
   virtual bool ArePredictedWindsAccurate(const aaesim::open_source::AircraftState &state,
                                          const aaesim::open_source::WeatherPrediction &weatherPrediction,
                                          const Units::Speed reference_cas, const Units::Length reference_altitude,
                                          const Atmosphere *sensed_atmosphere) const;

  private:
   const static Units::Angle toleranceAngle;
   const static Units::Speed toleranceSpeed;

   static std::shared_ptr<aaesim::open_source::PredictedWindEvaluator> mInstance;

   MOPSPredictedWindEvaluatorVersion1();
};
}  // namespace open_source
}  // namespace interval_management