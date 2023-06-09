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

#include <deque>
#include <public/StandardAtmosphere.h>
#include "IMAlgorithm.h"
#include "KineticAlgorithm.h"

namespace interval_management {
namespace open_source {
class TestVectorSpeedControl : public IMAlgorithm, public KineticAlgorithm, Loadable {
  public:
   TestVectorSpeedControl();
   virtual ~TestVectorSpeedControl();

   virtual aaesim::open_source::Guidance Update(
         const aaesim::open_source::Guidance &prevguidance, const aaesim::open_source::DynamicsState &dynamicsstate,
         const interval_management::open_source::AircraftState &owntruthstate,
         const interval_management::open_source::AircraftState &targettruthstate,
         const std::vector<interval_management::open_source::AircraftState> &targethistory);

   bool load(DecodedStream *input) override;

   void ResetDefaults() override;

   void InitializeFmsPredictors(const Wgs84KineticDescentPredictor &ownship_kinetic_trajectory_predictor,
                                const Wgs84KineticDescentPredictor &target_kinetic_trajectory_predictor) override;

   void SetBlendWind(bool wind_blending_enabled) override;

  protected:
   virtual void SetAssignedSpacingGoal(const IMClearance &clearance);

   virtual const double GetSpacingError() const;

  private:
   static unsigned int DEFAULT_DECELERATION_START_TIME_SEC, DEFAULT_ACCELERATION_START_TIME_SEC;
   static unsigned long DEFAULT_DECELERATION_DELTA_IAS, DEFAULT_ACCELERATION_DELTA_IAS;

   Wgs84AlongPathDistanceCalculator m_distance_calculator;
   Units::KnotsSpeed m_acceleration_phase_target_ias;
   Units::KnotsSpeed m_deceleration_phase_target_ias;
   unsigned long m_acceleration_phase_hold_duration;
   unsigned long m_acceleration_phase_count;
   Units::KnotsSpeed m_acceleration_phase_delta_ias;
   Units::KnotsSpeed m_deceleration_phase_delta_ias;
   unsigned int m_deceleration_start_time_sec, m_acceleration_start_time_sec;
   bool m_acceleration_phase_complete;
   bool m_acceleration_target_achieved;
   std::deque<Units::Speed> m_pilot_delayed_speeds;
   const aaesim::Wgs84KineticDescentPredictor *m_target_kinetic_trajectory_predictor;
   const aaesim::Wgs84KineticDescentPredictor *m_ownship_kinetic_trajectory_predictor;
};

inline void TestVectorSpeedControl::SetBlendWind(bool wind_blending_enabled) { /* required by the interface, but not
                                                                                  used */
}
}  // namespace open_source
}  // namespace interval_management