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

#include "imalgs/IMAlgorithm.h"
#include "imalgs/IMAchieve.h"
#include "KinematicTrajectoryPredictor.h"
#include <Frequency.h>

class IMMaintain : public IMAlgorithm
{
public:
   static const Units::HertzFrequency MAINTAIN_CONTROL_GAIN_DEFAULT;

   IMMaintain();

   IMMaintain(const IMMaintain &obj);

   virtual ~IMMaintain();

   virtual void IterationReset();

   virtual void ResetDefaults();

   void InitializeScenario(IMAchieve *obj,
                           const Units::Frequency maintain_control_gain);

   virtual void Prepare(Units::Speed previous_im_speed_command,
                        Units::Speed previous_ias_command,
                        double previous_mach_command,
                        std::shared_ptr<TangentPlaneSequence> tangent_plane_sequence,
                        const TrajectoryPredictor &ownship_trajectory_predictor,
                        const AlongPathDistanceCalculator &im_distance_calculator,
                        const vector<AircraftState> &target_adsb_track_history,
                        const IMClearance &im_clearance,
                        const bool has_rf_leg,
                        const std::vector<std::pair<Units::Length, Units::Speed>> &rf_limits);

   Units::Frequency GetMaintainControlGain() const;

   virtual const double GetSpacingError() const;

   virtual void SetAssignedSpacingGoal(const IMClearance &clearance);

   virtual void DumpParameters(const std::string &parameters_to_print);

protected:
   void Copy(const IMMaintain &obj);

   Units::Frequency m_maintain_control_gain;

   AlongPathDistanceCalculator m_ownship_decrementing_distance_calculator;
   AlongPathDistanceCalculator m_ownship_distance_calculator;

private:
   void IterClearIMMain();

   //FIXME aaes-820 m_logger shadows name in IMAlgorithm
   static log4cplus::Logger logger;
};
