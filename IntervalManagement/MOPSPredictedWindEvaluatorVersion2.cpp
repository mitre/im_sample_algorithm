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

#include "imalgs/MOPSPredictedWindEvaluatorVersion2.h"
#include "public/Environment.h"

log4cplus::Logger MOPSPredictedWindEvaluatorVersion2::m_logger = log4cplus::Logger::getInstance(
      LOG4CPLUS_TEXT("MOPSPredictedWindEvaluatorVersion2"));

Units::KnotsSpeed MOPSPredictedWindEvaluatorVersion2::MAX_PERMITTED_GROUNDSPEED_ERROR(8);

std::shared_ptr<PredictedWindEvaluator> MOPSPredictedWindEvaluatorVersion2::mInstance;

const std::shared_ptr<PredictedWindEvaluator> &MOPSPredictedWindEvaluatorVersion2::getInstance() {
   if (!mInstance) {
      mInstance = std::shared_ptr<PredictedWindEvaluator>(new MOPSPredictedWindEvaluatorVersion2());
   }
   return mInstance;
}

MOPSPredictedWindEvaluatorVersion2::MOPSPredictedWindEvaluatorVersion2() {
}

MOPSPredictedWindEvaluatorVersion2::~MOPSPredictedWindEvaluatorVersion2() {
}

bool MOPSPredictedWindEvaluatorVersion2::ArePredictedWindsAccurate(
      const AircraftState &state,
      const WeatherPrediction &weather_prediction,
      const Units::Speed reference_cas,
      const Units::Length reference_altitude,
      const Atmosphere *sensed_atmosphere) const {

   Units::FeetLength true_altitude(state.m_z);
   Units::KnotsSpeed tas1 = weather_prediction.CAS2TAS(reference_cas, reference_altitude);
   Units::KnotsSpeed tas2 = sensed_atmosphere->CAS2TAS(reference_cas, true_altitude);

   // use predicted winds for calc #1
   Units::Speed predicted_wind_x, predicted_wind_y;
   Units::HertzFrequency dVwx_dh, dVwy_dh;
   weather_prediction.GetForecastAtmosphere()->CalculateWindGradientAtAltitude(
         reference_altitude, weather_prediction.east_west, predicted_wind_x, dVwx_dh);
   weather_prediction.GetForecastAtmosphere()->CalculateWindGradientAtAltitude(
         reference_altitude, weather_prediction.north_south, predicted_wind_y, dVwy_dh);

   double sin_heading(sin(state.m_psi));
   double cos_heading(cos(state.m_psi));

   // calculate predicted tailwind and crosswind based on true course
   Units::Speed vw_para1(predicted_wind_x * cos_heading + predicted_wind_y * sin_heading);
   Units::Speed vw_perp1(-predicted_wind_x * sin_heading + predicted_wind_y * cos_heading);
   Units::MetersPerSecondSpeed vw_para2(state.m_Vw_para);
   Units::MetersPerSecondSpeed vw_perp2(state.m_Vw_perp);

   // Pythagorean-subtract crosswind
   Units::KnotsSpeed tas_para1 = Units::sqrt(Units::sqr(tas1) - Units::sqr(vw_perp1));
   Units::KnotsSpeed tas_para2 = Units::sqrt(Units::sqr(tas2) - Units::sqr(vw_perp2));

   // add tailwind
   Units::KnotsSpeed gs1 = tas_para1 + vw_para1;
   Units::KnotsSpeed gs2 = tas_para2 + vw_para2;

   if (abs(gs1 - gs2) > MAX_PERMITTED_GROUNDSPEED_ERROR) {
      Units::KelvinTemperature predicted_temperature = weather_prediction.getAtmosphere()->GetTemperature(
            reference_altitude);
      Units::PascalsPressure predicted_pressure;
      Units::KilogramsMeterDensity predicted_density;
      weather_prediction.getAtmosphere()->AirDensity(reference_altitude, predicted_density, predicted_pressure);

      Units::KelvinTemperature true_temperature = sensed_atmosphere->GetTemperature(true_altitude);
      Units::PascalsPressure true_pressure;
      Units::KilogramsMeterDensity true_density;
      sensed_atmosphere->AirDensity(true_altitude, true_density, true_pressure);

      // This table format can be pasted into JIRA
      LOG4CPLUS_DEBUG(m_logger, "Winds inaccurate:"
            << std::endl
            << "||time||source||altitude(ft)||temperature(C)||pressure(Pa)||density(kg/m^3)||wind_ew(kts)||wind_ns(kts)"
            << "||CAS(kts)||TAS(kts)||GS(kts)||"
            << std::endl
            << "|"
            << state.m_time
            << "|predicted|"
            <<
            Units::FeetLength(reference_altitude).value()
            << "|"
            <<
            (predicted_temperature.value() - 273.15)
            << "|"
            <<
            predicted_pressure.value()
            << "|"
            <<
            predicted_density.value()
            << "|"
            <<
            Units::KnotsSpeed(predicted_wind_x).value()
            << "|"
            <<
            Units::KnotsSpeed(predicted_wind_y).value()
            << "|"
            <<
            Units::KnotsSpeed(reference_cas).value()
            << "|"
            <<
            Units::KnotsSpeed(tas1).value()
            << "|"
            <<
            Units::KnotsSpeed(gs1).value()
            << "|"
            << std::endl
            << "|"
            << state.m_time
            << "|  true   |"
            <<
            true_altitude.value()
            << "|"
            <<
            (true_temperature.value() - 273.15)
            << "|"
            <<
            true_pressure.value()
            << "|"
            <<
            true_density.value()
            << "|"
            <<
            Units::KnotsSpeed(Units::MetersPerSecondSpeed(state.m_Vwx)).value()
            << "|"
            <<
            Units::KnotsSpeed(Units::MetersPerSecondSpeed(state.m_Vwy)).value()
            << "|"
            <<
            Units::KnotsSpeed(reference_cas).value()
            << "|"
            <<
            Units::KnotsSpeed(tas2).value()
            << "|"
            <<
            Units::KnotsSpeed(gs2).value()
            << "|");

/*    Sensed state (alt, GS, temp, press, density, wind)
    Predicted Weather params: temp, press, density, wind
    Truth Weather params
    both CAS, TAS, GS values*/

      return false;
   }

   // default return
   return true;
}
