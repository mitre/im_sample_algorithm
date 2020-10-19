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

#pragma once

#include "public/PredictedWindEvaluator.h"

class MOPSPredictedWindEvaluatorVersion1 : public PredictedWindEvaluator
{
public:
   const static std::shared_ptr<PredictedWindEvaluator> &getInstance();

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
   virtual bool ArePredictedWindsAccurate(
         const AircraftState &state,
         const WeatherPrediction &weatherPrediction,
         const Units::Speed reference_cas,
         const Units::Length reference_altitude,
         const Atmosphere *sensed_atmosphere) const;

private:
   const static Units::Angle toleranceAngle;
   const static Units::Speed toleranceSpeed;

   static std::shared_ptr<PredictedWindEvaluator> mInstance;

   MOPSPredictedWindEvaluatorVersion1();
};

