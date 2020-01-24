// ****************************************************************************
// NOTICE
//
// This is the copyright work of The MITRE Corporation, and was produced
// for the U. S. Government under Contract Number DTFAWA-10-C-00080, and
// is subject to Federal Aviation Administration Acquisition Management
// System Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV
// (Oct. 1996).  No other use other than that granted to the U. S.
// Government, or to those acting on behalf of the U. S. Government,
// under that Clause is authorized without the express written
// permission of The MITRE Corporation. For further information, please
// contact The MITRE Corporation, Contracts Office, 7515 Colshire Drive,
// McLean, VA  22102-7539, (703) 983-6000. 
//
// Copyright 2020 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

/*
 * MOPSPredictedWindEvaluator.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: klewis
 */

#include "imalgs/MOPSPredictedWindEvaluatorVersion1.h"
#include "public/Environment.h"

// NOTE: Do not change these values without discussing with Lesley, these come from the MOPS.
const Units::Angle MOPSPredictedWindEvaluatorVersion1::toleranceAngle(Units::DegreesAngle(15.0));
const Units::Speed MOPSPredictedWindEvaluatorVersion1::toleranceSpeed(Units::KnotsSpeed(10.0));

std::shared_ptr<PredictedWindEvaluator> MOPSPredictedWindEvaluatorVersion1::mInstance;

const std::shared_ptr<PredictedWindEvaluator> &MOPSPredictedWindEvaluatorVersion1::getInstance() {
   if (!mInstance) {
      mInstance = std::shared_ptr<PredictedWindEvaluator>(new MOPSPredictedWindEvaluatorVersion1());
   }
   return mInstance;
}

MOPSPredictedWindEvaluatorVersion1::MOPSPredictedWindEvaluatorVersion1() {
}

MOPSPredictedWindEvaluatorVersion1::~MOPSPredictedWindEvaluatorVersion1() {
}

bool MOPSPredictedWindEvaluatorVersion1::ArePredictedWindsAccurate(
      const AircraftState &state,
      const WeatherPrediction &weatherPrediction,
      const Units::Speed reference_cas,
      const Units::Length reference_altitude,
      const Atmosphere *sensed_atmosphere) const {

   Units::MetersPerSecondSpeed windeastcomp, windnorthcomp; // units of mps as returned from AircraftCalculations
   Units::Frequency dtmp;
   weatherPrediction.getAtmosphere()->CalculateWindGradientAtAltitude(Units::FeetLength(state.m_z),
                                                                      weatherPrediction.east_west,
                                                                      windeastcomp, dtmp); // mps
   weatherPrediction.getAtmosphere()->CalculateWindGradientAtAltitude(Units::FeetLength(state.m_z),
                                                                      weatherPrediction.north_south,
                                                                      windnorthcomp, dtmp); // mps

   // Spd

   Units::Speed predicted_spd = sqrt(Units::sqr(windeastcomp) + Units::sqr(windnorthcomp));
   Units::Speed Vwx_sensed = Units::MetersPerSecondSpeed(state.m_Vwx);
   Units::Speed Vwy_sensed = Units::MetersPerSecondSpeed(state.m_Vwy);
   Units::Speed sensed_spd = sqrt((Vwx_sensed * Vwx_sensed) + (Vwy_sensed * Vwy_sensed));

   bool isWindSpdAccurate = abs(abs(sensed_spd) - abs(predicted_spd)) <= toleranceSpeed;

   if (!isWindSpdAccurate) {
      return isWindSpdAccurate; // just return now, same as returning false
   }

   // Dir
   Units::Angle predicted_dir = Units::arctan2(windnorthcomp.value(),
                                               windeastcomp.value()); // ENU, so east is the x-component
   Units::Angle measured_dir = Units::arctan2(state.m_Vwy, state.m_Vwx);
   Units::SignedDegreesAngle difference = measured_dir - predicted_dir;

   bool isWindDirAccurate = abs(difference) <= toleranceAngle;

   if (!isWindDirAccurate) {
      return isWindDirAccurate; // return now, same as returning false
   }

   // default return
   return true; // same as (isWindDirAccurate || isWindSpdAccurate)
}
