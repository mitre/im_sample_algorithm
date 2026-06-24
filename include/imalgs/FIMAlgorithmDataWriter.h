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
// (c) 2026 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <log4cplus/logger.h>
#include <scalar/Angle.h>
#include <scalar/Length.h>
#include <scalar/Speed.h>
#include <scalar/Temperature.h>
#include <scalar/Time.h>

#include <memory>
#include <string>
#include <vector>

#include "imalgs/AircraftState.h"
#include "imalgs/IMAlgorithm.h"
#include "public/FlightDeckApplication.h"
#include "public/Guidance.h"
#include "public/OutputHandler.h"
#include "public/SimulationTime.h"

namespace interval_management {
namespace open_source {
class FIMAlgorithmDataWriter final : public OutputHandler {
  public:
   FIMAlgorithmDataWriter();
   ~FIMAlgorithmDataWriter() = default;

   virtual void Finish();

   void Gather(const int iteration_number, const aaesim::open_source::SimulationTime &time,
               const std::string &aircraft_id,
               std::shared_ptr<const aaesim::open_source::FlightDeckApplication> application);

  private:
   struct SimData {
      SimData() = default;

      int iteration_number{-1};
      Units::Time simulation_time{Units::SecondsTime(-1.0)};
      std::string acid{};
      int m_target_acid{-1};
      interval_management::open_source::IMAlgorithm::FlightStage m_flight_stage{
            interval_management::open_source::IMAlgorithm::FlightStage::UNSET};
      double m_assigned_spacing_goal{INT32_MIN};
      unsigned long int m_current_imspeed_count{0};
      double m_measured_spacing_interval{INT32_MIN};
      double m_predicted_spacing_interval{INT32_MIN};
      Units::Time m_ownship_ttg_to_abp{Units::negInfinity()};
      Units::Time m_ownship_ttg_to_ptp{Units::negInfinity()};
      Units::Length m_ownship_dtg_to_abp{Units::negInfinity()};
      Units::Length m_ownship_dtg_to_ptp{Units::negInfinity()};
      Units::Time m_target_ttg_to_trp{Units::negInfinity()};
      Units::Length m_target_dtg_to_trp{Units::negInfinity()};
      Units::Length m_target_projected_position_x{Units::negInfinity()};
      Units::Length m_target_projected_position_y{Units::negInfinity()};
      unsigned long int m_im_speed_limit_flags{0L};
      double m_target_is_aligned{-INFINITY};  // can be -infinity, 0, or 1
      Units::Speed m_imspeed_ias{Units::negInfinity()};
      Units::Speed m_imspeed_delayed_ias{Units::negInfinity()};
      Units::Speed m_ownship_reference_ias{Units::negInfinity()};
      Units::Speed m_target_reference_ias{Units::negInfinity()};
      Units::Speed m_ownship_reference_groundspeed{Units::negInfinity()};
      Units::Speed m_target_reference_groundspeed{Units::negInfinity()};
      Units::Length m_ownship_reference_altitude{Units::negInfinity()};
      Units::Length m_target_reference_altitude{Units::negInfinity()};
   };

   std::vector<SimData> m_sim_data;

   static log4cplus::Logger m_logger;
};
}  // namespace open_source
}  // namespace interval_management
