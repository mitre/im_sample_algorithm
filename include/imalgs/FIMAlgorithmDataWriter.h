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

#include <string>
#include <scalar/Time.h>
#include <scalar/Length.h>
#include <scalar/Speed.h>
#include <scalar/Angle.h>
#include <scalar/Temperature.h>
#include <vector>

#include "public/Logging.h"
#include "public/SimulationTime.h"
#include "public/OutputHandler.h"
#include "public/Guidance.h"
#include "public/FlightDeckApplication.h"
#include "imalgs/AircraftState.h"
#include "imalgs/IMAlgorithm.h"

namespace interval_management {
namespace open_source {
class FIMAlgorithmDataWriter : public OutputHandler {

  public:
   FIMAlgorithmDataWriter();
   ~FIMAlgorithmDataWriter() = default;

   virtual void Finish();

   void Gather(const int iteration_number, const aaesim::open_source::SimulationTime &time, std::string aircraft_id,
               std::shared_ptr<const aaesim::open_source::FlightDeckApplication> application);

  private:
   struct SimData {
      SimData() {
         iteration_number = -1;
         acid = "";
         simulation_time = Units::SecondsTime(-1.0);
         m_target_acid = -1;

         m_imspeed_ias = Units::MetersPerSecondSpeed(-1.0);
         m_ownship_ttg_to_abp = Units::SecondsTime(-1.0);
         m_ownship_dtg_to_abp = Units::MetersLength(-1.0);
         m_flight_stage = interval_management::open_source::IMAlgorithm::FlightStage::UNSET;
         m_measured_spacing_interval = -1.0;
         m_predicted_spacing_interval = -1.0;
         m_assigned_spacing_goal = -INFINITY;
         m_im_speed_limit_flags = 0L;
         m_ownship_dtg_to_ptp = Units::MetersLength(-1.0);
      };

      int iteration_number;
      Units::Time simulation_time;
      std::string acid;

      int m_target_acid;

      interval_management::open_source::IMAlgorithm::FlightStage m_flight_stage;
      double m_assigned_spacing_goal;
      unsigned long int m_current_imspeed_count;

      // algorithm intermediate calculations
      double m_measured_spacing_interval;
      double m_predicted_spacing_interval;
      Units::Time m_ownship_ttg_to_abp;
      Units::Time m_ownship_ttg_to_ptp;
      Units::Length m_ownship_dtg_to_abp;
      Units::Length m_ownship_dtg_to_ptp;
      Units::Time m_target_ttg_to_trp;
      Units::Length m_target_dtg_to_trp;
      Units::Length m_target_projected_position_x;
      Units::Length m_target_projected_position_y;
      unsigned long int m_im_speed_limit_flags;
      double m_target_is_aligned;  // can be -infinity, 0, or 1

      // algorithm output
      Units::Speed m_imspeed_ias;
      Units::Speed m_imspeed_delayed_ias;

      // reference trajectory parameters used by the algorithm
      Units::Speed m_ownship_reference_ias;
      Units::Speed m_target_reference_ias;
      Units::Speed m_ownship_reference_groundspeed;
      Units::Speed m_target_reference_groundspeed;
      Units::Length m_ownship_reference_altitude;
      Units::Length m_target_reference_altitude;
   };

   std::vector<SimData> m_sim_data;

   static log4cplus::Logger m_logger;
};
}  // namespace open_source
}  // namespace interval_management