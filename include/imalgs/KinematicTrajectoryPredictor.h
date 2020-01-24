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

#include "utility/Logging.h"
#include "aaesim/TrajectoryPredictor.h"
#include "aaesim/KineticTrajectoryPredictor.h"
#include "imalgs/KinematicDescent4DPredictor.h"
#include "public/PrecalcWaypoint.h"
#include "public/HorizontalPath.h"
#include "public/AircraftIntent.h"
#include "public/Guidance.h"
#include "public/AircraftState.h"
#include <vector>
#include <Angle.h>


class KinematicTrajectoryPredictor : public TrajectoryPredictor
{
public:

   KinematicTrajectoryPredictor();

   KinematicTrajectoryPredictor(const KinematicTrajectoryPredictor &obj);

   virtual ~KinematicTrajectoryPredictor();

   void CalculateWaypoints(const AircraftIntent &aircraft_intent) override;

   void SetMembers(const KineticTrajectoryPredictor &kinetic_trajectory_predictor);

   KinematicTrajectoryPredictor &operator=(const KinematicTrajectoryPredictor &obj);

   bool operator==(const KinematicTrajectoryPredictor &obj) const;

   bool operator!=(const KinematicTrajectoryPredictor &boj) const;

   const std::vector<double> &GetVerticalPathDistances() const;

   const double GetVerticalPathDistanceByIndex(int index) const;

   const std::vector<double> &GetVerticalPathTimes() const;

   const double GetVerticalPathTimeByIndex(int index) const;

   const std::vector<double> &GetVerticalPathGroundspeeds() const;

   const std::vector<double> &GetVerticalPathVelocities() const;

   const double GetVerticalPathVelocityByIndex(int index) const;

   const std::vector<double> &GetVerticalPathAltitudes() const;

   const double GetVerticalPathAltitudeByIndex(const int index) const;

   std::shared_ptr<KinematicDescent4DPredictor> GetKinematicDescent4dPredictor() const;

private:
   static log4cplus::Logger m_logger;
};


inline const std::vector<double> &KinematicTrajectoryPredictor::GetVerticalPathDistances() const {
   return m_vertical_predictor->GetVerticalPath().x;
}

inline const double KinematicTrajectoryPredictor::GetVerticalPathDistanceByIndex(int index) const {
   return m_vertical_predictor->GetVerticalPath().x[index];
}

inline const std::vector<double> &KinematicTrajectoryPredictor::GetVerticalPathTimes() const {
   return m_vertical_predictor->GetVerticalPath().time;
}

inline const double KinematicTrajectoryPredictor::GetVerticalPathTimeByIndex(int index) const {
   return m_vertical_predictor->GetVerticalPath().time[index];
}

inline const std::vector<double> &KinematicTrajectoryPredictor::GetVerticalPathGroundspeeds() const {
   return m_vertical_predictor->GetVerticalPath().gs;
}

inline const std::vector<double> &KinematicTrajectoryPredictor::GetVerticalPathVelocities() const {
   return m_vertical_predictor->GetVerticalPath().v;
}

inline const double KinematicTrajectoryPredictor::GetVerticalPathVelocityByIndex(int index) const {
   return m_vertical_predictor->GetVerticalPath().v[index];
}

inline const std::vector<double> &KinematicTrajectoryPredictor::GetVerticalPathAltitudes() const {
   return m_vertical_predictor->GetVerticalPath().h;
}

inline const double KinematicTrajectoryPredictor::GetVerticalPathAltitudeByIndex(const int index) const {
   return m_vertical_predictor->GetVerticalPath().h[index];
}

