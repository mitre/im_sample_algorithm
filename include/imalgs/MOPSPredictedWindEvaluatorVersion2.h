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

#pragma once

#include "public/PredictedWindEvaluator.h"

namespace interval_management {
namespace open_source {
class MOPSPredictedWindEvaluatorVersion2 : public aaesim::open_source::PredictedWindEvaluator {
  public:
   static Units::KnotsSpeed MAX_PERMITTED_GROUNDSPEED_ERROR;

   const static std::shared_ptr<aaesim::open_source::PredictedWindEvaluator> &GetInstance();

   virtual ~MOPSPredictedWindEvaluatorVersion2();

   /**
    * Employs an algorithm to check the difference between the predicted wind and the
    * provided sensed wind.
    * Returns true if the algorithm shows that the two winds are sufficiently similar. Returns false otherwise.
    *
    * @param state and aircraft_state_vector that represents the most recent navigations state
    * @param weatherPrediction The predicted winds to be evaluated
    */
   virtual bool ArePredictedWindsAccurate(const aaesim::open_source::AircraftState &state,
                                          const WeatherPrediction &weatherPrediction, const Units::Speed reference_cas,
                                          const Units::Length reference_altitude,
                                          const Atmosphere *sensed_atmosphere) const;

  private:
   static log4cplus::Logger m_logger;

   static std::shared_ptr<aaesim::open_source::PredictedWindEvaluator> mInstance;

   static void LogWindDisagreeMetaData(
         const Units::KelvinTemperature predicted_temperature, const Units::KelvinTemperature true_temperature,
         const Units::PascalsPressure predicted_pressure, const Units::PascalsPressure true_pressure,
         const Units::KilogramsMeterDensity predicted_density, Units::KilogramsMeterDensity true_density,
         const Units::Length reference_altitude, const Units::FeetLength true_altitude,
         const Units::Speed reference_cas, const Units::KnotsSpeed tas1, const Units::KnotsSpeed tas2,
         const Units::KnotsSpeed gs1, const Units::KnotsSpeed gs2, const Units::Speed predicted_wind_x,
         const Units::Speed predicted_wind_y, const aaesim::open_source::AircraftState &state);

   MOPSPredictedWindEvaluatorVersion2();
};
}  // namespace open_source
}  // namespace interval_management