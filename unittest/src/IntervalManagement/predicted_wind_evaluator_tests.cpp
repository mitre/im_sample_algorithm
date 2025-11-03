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

#include "public/Atmosphere.h"
#include "imalgs/MOPSPredictedWindEvaluatorVersion1.h"
#include "imalgs/MOPSPredictedWindEvaluatorVersion2.h"
#include "public/NullAtmosphere.h"
#include "public/USStandardAtmosphere1976.h"
#include "public/Wind.h"

#include "gtest/gtest.h"

using namespace aaesim::open_source;
using namespace interval_management::open_source;

namespace aaesim {
namespace test {
namespace imalgo {
class PredictedWindEvaluatorTest : public ::testing::Test {
  protected:
   PredictedWindEvaluatorTest()
      : aircraft_state(),
        reference_cas(Units::ZERO_SPEED),
        reference_altitude(Units::ZERO_LENGTH),
        sensed_atmosphere(new USStandardAtmosphere1976()),
        weather_prediction(WeatherPrediction::CreateZeroWindPrediction(sensed_atmosphere)) {}

   aaesim::open_source::AircraftState aircraft_state{};
   Units::Speed reference_cas;
   Units::Length reference_altitude;
   std::shared_ptr<Atmosphere> sensed_atmosphere;
   WeatherPrediction weather_prediction;
};

TEST_F(PredictedWindEvaluatorTest, MOPSPredictedWindEvaluatorVersion1) {
   const std::shared_ptr<PredictedWindEvaluator> mops_predicted_wind_evaluator_v1 =
         MOPSPredictedWindEvaluatorVersion1::GetInstance();
   ASSERT_TRUE(mops_predicted_wind_evaluator_v1->ArePredictedWindsAccurate(
         aircraft_state, weather_prediction, Units::ZERO_SPEED, Units::ZERO_LENGTH, nullptr));

   aircraft_state =
         AircraftState::Builder(0, 0).SensedWindComponents(Units::MetersPerSecondSpeed(10), Units::ZERO_SPEED)->Build();
   ASSERT_FALSE(mops_predicted_wind_evaluator_v1->ArePredictedWindsAccurate(
         aircraft_state, weather_prediction, Units::ZERO_SPEED, Units::ZERO_LENGTH, nullptr));

   aircraft_state =
         AircraftState::Builder(0, 0).SensedWindComponents(Units::ZERO_SPEED, Units::MetersPerSecondSpeed(1))->Build();
   ASSERT_FALSE(mops_predicted_wind_evaluator_v1->ArePredictedWindsAccurate(
         aircraft_state, weather_prediction, Units::ZERO_SPEED, Units::ZERO_LENGTH, nullptr));
}

TEST_F(PredictedWindEvaluatorTest, MOPSPredictedWindEvaluatorVersion2) {
   const std::shared_ptr<PredictedWindEvaluator> mops_predicted_wind_evaluator_v2 =
         MOPSPredictedWindEvaluatorVersion2::GetInstance();
   ASSERT_TRUE(mops_predicted_wind_evaluator_v2->ArePredictedWindsAccurate(
         aircraft_state, weather_prediction, reference_cas, reference_altitude, sensed_atmosphere));

   reference_altitude = Units::FeetLength(10000);
   reference_cas = Units::KnotsSpeed(100);
   ASSERT_FALSE(mops_predicted_wind_evaluator_v2->ArePredictedWindsAccurate(
         aircraft_state, weather_prediction, reference_cas, reference_altitude, sensed_atmosphere));
}

}  // namespace imalgo
}  // namespace test
}  // namespace aaesim
