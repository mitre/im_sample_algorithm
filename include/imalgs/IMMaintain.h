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

#include "imalgs/IMAlgorithm.h"
#include "imalgs/IMAchieve.h"
#include <scalar/Frequency.h>
#include "public/EuclideanTrajectoryPredictor.h"
namespace interval_management {
namespace open_source {

class IMMaintain : public IMAlgorithm {
  public:
   static const Units::HertzFrequency MAINTAIN_CONTROL_GAIN_DEFAULT;

   IMMaintain();

   IMMaintain(const IMMaintain &obj);

   virtual ~IMMaintain() = default;

   virtual void IterationReset();

   virtual void ResetDefaults();

   void InitializeScenario(IMAchieve *obj, const Units::Frequency maintain_control_gain);

   virtual void Prepare(Units::Speed previous_im_speed_command, Units::Speed previous_ias_command,
                        interval_management::open_source::FIMSpeedLimiter speed_limiter, double previous_mach_command,
                        const aaesim::open_source::EuclideanTrajectoryPredictor &ownship_trajectory_predictor,
                        const AlongPathDistanceCalculator &im_distance_calculator,
                        const std::vector<interval_management::open_source::AircraftState> &target_adsb_track_history,
                        const IMClearance &im_clearance,
                        const std::vector<interval_management::open_source::FIMSpeedLimiter::RfLegLimit> &rf_limits);

   Units::Frequency GetMaintainControlGain() const;

   virtual const double GetSpacingError() const;

   virtual void SetAssignedSpacingGoal(const IMClearance &clearance);

   virtual void DumpParameters(const std::string &parameters_to_print);

   void SetBlendWind(bool wind_blending_enabled) override;

  protected:
   void Copy(const IMMaintain &obj);

   AlongPathDistanceCalculator m_ownship_decrementing_distance_calculator;
   AlongPathDistanceCalculator m_ownship_distance_calculator;

  private:
   static log4cplus::Logger m_logger;
};

inline void IMMaintain::SetBlendWind(bool wind_blending_enabled) { /* required by the interface, but not used */
}
}  // namespace open_source
}  // namespace interval_management