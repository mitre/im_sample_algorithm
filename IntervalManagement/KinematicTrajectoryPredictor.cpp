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

#include "public/InternalObserver.h"
#include "imalgs/KinematicTrajectoryPredictor.h"
#include "public/AircraftCalculations.h"

using namespace std;

log4cplus::Logger KinematicTrajectoryPredictor::m_logger = log4cplus::Logger::getInstance(
      LOG4CPLUS_TEXT("KinematicTrajectoryPredictor"));

KinematicTrajectoryPredictor::KinematicTrajectoryPredictor() {
   m_vertical_predictor = std::shared_ptr<VerticalPredictor>(new KinematicDescent4DPredictor());
}

KinematicTrajectoryPredictor::KinematicTrajectoryPredictor(const KinematicTrajectoryPredictor &obj) {
   operator=(obj);
}

KinematicTrajectoryPredictor::~KinematicTrajectoryPredictor() {
}

void KinematicTrajectoryPredictor::CalculateWaypoints(const AircraftIntent &aircraft_intent) {
   Units::Length altitude_at_faf = Units::MetersLength(aircraft_intent.GetFms().m_altitude[aircraft_intent.GetNumberOfWaypoints() - 1]);
   Units::Speed ias_at_faf = Units::FeetPerSecondSpeed(
         aircraft_intent.GetFms().m_nominal_ias[aircraft_intent.GetNumberOfWaypoints() - 1]);
   GetKinematicDescent4dPredictor()->SetConditionsAtEndOfRoute(altitude_at_faf, ias_at_faf);

   TrajectoryPredictor::CalculateWaypoints(aircraft_intent);
}

void KinematicTrajectoryPredictor::SetMembers(const KineticTrajectoryPredictor &kinetic_trajectory_predictor) {
   m_bank_angle = kinetic_trajectory_predictor.GetBankAngle();
   GetKinematicDescent4dPredictor()->SetMembers(*kinetic_trajectory_predictor.GetVertPredictor());
}

KinematicTrajectoryPredictor &KinematicTrajectoryPredictor::operator=(const KinematicTrajectoryPredictor &obj) {
   if (this != &obj) {
      TrajectoryPredictor::operator=(obj);

      std::shared_ptr<KinematicDescent4DPredictor> kin = obj.GetKinematicDescent4dPredictor();

      if (kin == NULL) {
         m_vertical_predictor = std::shared_ptr<VerticalPredictor>((KinematicDescent4DPredictor *) NULL);
      } else {
         m_vertical_predictor = std::shared_ptr<VerticalPredictor>(new KinematicDescent4DPredictor(*kin));
      }
   }

   return *this;
}

bool KinematicTrajectoryPredictor::operator==(const KinematicTrajectoryPredictor &obj) const {
   bool match = TrajectoryPredictor::operator==(obj);

   match = match && (*GetKinematicDescent4dPredictor() == *obj.GetKinematicDescent4dPredictor());

   return match;
}

bool KinematicTrajectoryPredictor::operator!=(const KinematicTrajectoryPredictor &obj) const {
   return !operator==(obj);
}

std::shared_ptr<KinematicDescent4DPredictor> KinematicTrajectoryPredictor::GetKinematicDescent4dPredictor() const {
   return std::shared_ptr<KinematicDescent4DPredictor>(
         static_pointer_cast<KinematicDescent4DPredictor>(m_vertical_predictor));
}
