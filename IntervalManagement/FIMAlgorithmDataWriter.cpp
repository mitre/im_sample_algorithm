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

#include "imalgs/FIMAlgorithmDataWriter.h"

#include "imalgs/FIMAlgorithmAdapter.h"
#include "imalgs/IMKinematicAchieve.h"
#include "public/AircraftSpeed.h"
#include "public/CoreUtils.h"

log4cplus::Logger interval_management::open_source::FIMAlgorithmDataWriter::m_logger =
      log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("FIMAlgorithmDataWriter"));

interval_management::open_source::FIMAlgorithmDataWriter::FIMAlgorithmDataWriter()
   : OutputHandler("", "_IMAlgorithm.csv"), m_sim_data() {
   m_sim_data.reserve(10000);
}

void interval_management::open_source::FIMAlgorithmDataWriter::Finish() {
   if (!m_sim_data.empty()) {
      // Open file and set delimiter.
      mini::csv::ofstream os(filename.c_str());

      if (!os.is_open()) {
         std::string emsg = "Cannot open " + filename;
         LOG4CPLUS_FATAL(interval_management::open_source::FIMAlgorithmDataWriter::m_logger, emsg);
         return;
      }

      os.set_delimiter(',', ",");

      // Write header.

      os << "iteration";
      os << "ownship_acid";
      os << "time_secs";
      os << "target_acid";
      os << "stage";
      os << "asg";
      os << "imspeed_ias_mps";
      os << "imspeed_delayed_ias_mps";
      os << "msi";
      os << "psi";

      os << "ownship_ttg_to_abp_sec";
      os << "ownship_dtg_to_abp_m";
      os << "ownship_ttg_to_ptp_sec";
      os << "ownship_dtg_to_ptp_m";
      os << "target_ttg_to_trp_sec";
      os << "target_dtg_to_trp_m";
      os << "imspeed_limit_flag";
      os << "target_is_aligned";

      os << "ownship_ref_ias_mps";
      os << "ownship_ref_gs_mps";
      os << "ownship_ref_alt_m";
      os << "target_ref_ias_mps";
      os << "target_ref_gs_mps";
      os << "target_ref_alt_m";

      os << "target_pos_x_proj_asg_adjusted_m";
      os << "target_pos_y_proj_asg_adjusted_m";

      os << "current_imspeed_command_count";

      os << NEWLINE;
      os.flush();

      // Important for outputting double data.
      os.set_precision(10);

      for (auto &ix : m_sim_data) {
         os << ix.iteration_number;
         os << ix.acid;
         os << Units::SecondsTime(ix.simulation_time).value();
         os << ix.m_target_acid;
         os << ix.m_flight_stage;
         os << ix.m_assigned_spacing_goal;
         os << Units::MetersPerSecondSpeed(ix.m_imspeed_ias).value();
         os << Units::MetersPerSecondSpeed(ix.m_imspeed_delayed_ias).value();
         os << ix.m_measured_spacing_interval;
         os << ix.m_predicted_spacing_interval;

         os << Units::SecondsTime(ix.m_ownship_ttg_to_abp).value();
         os << Units::MetersLength(ix.m_ownship_dtg_to_abp).value();
         os << Units::SecondsTime(ix.m_ownship_ttg_to_ptp).value();
         os << Units::MetersLength(ix.m_ownship_dtg_to_ptp).value();
         os << Units::SecondsTime(ix.m_target_ttg_to_trp).value();
         os << Units::MetersLength(ix.m_target_dtg_to_trp).value();
         os << ix.m_im_speed_limit_flags;

         os << ix.m_target_is_aligned;

         os << Units::MetersPerSecondSpeed(ix.m_ownship_reference_ias).value();
         os << Units::MetersPerSecondSpeed(ix.m_ownship_reference_groundspeed).value();
         os << Units::MetersLength(ix.m_ownship_reference_altitude).value();
         os << Units::MetersPerSecondSpeed(ix.m_target_reference_ias).value();
         os << Units::MetersPerSecondSpeed(ix.m_target_reference_groundspeed).value();
         os << Units::MetersLength(ix.m_target_reference_altitude).value();

         os << Units::MetersLength(ix.m_target_projected_position_x).value();
         os << Units::MetersLength(ix.m_target_projected_position_y).value();

         os << Units::MetersLength(ix.m_current_imspeed_count).value();

         os << NEWLINE;
         os.flush();
      }

      os.close();
   }

   m_finished = true;
   m_sim_data.clear();
}

void interval_management::open_source::FIMAlgorithmDataWriter::Gather(
      const int iteration_number, const aaesim::open_source::SimulationTime &time, std::string aircraft_id,
      std::shared_ptr<const aaesim::open_source::FlightDeckApplication> application) {

   const bool is_fim_application =
         CoreUtils::InstanceOf<interval_management::open_source::FIMAlgorithmAdapter>(application.get());
   if (!is_fim_application) {
      return;
   }
   std::shared_ptr<IMAlgorithm> im_algorithm;
   IMUtils::IMAlgorithmTypes im_algorithm_type;
   std::shared_ptr<const interval_management::open_source::FIMAlgorithmAdapter> im_algorithm_adapter =
         std::dynamic_pointer_cast<const interval_management::open_source::FIMAlgorithmAdapter>(application);
   im_algorithm = im_algorithm_adapter->GetImAlgorithm();
   im_algorithm_type = im_algorithm_adapter->GetImAlgorithmType();
   if (im_algorithm->GetClearance().GetTargetId() < 0) {
      return;
   }

   SimData s;
   s.iteration_number = iteration_number;
   s.simulation_time = time.GetCurrentSimulationTime();
   s.acid = aircraft_id;
   s.m_target_acid = im_algorithm->GetClearance().GetTargetId();
   s.m_flight_stage = im_algorithm->GetFlightStage();
   s.m_current_imspeed_count = im_algorithm->GetSpeedChangeCount();

   // Note: don't grab the ASG value from the clearance object. That is static
   // and sometimes the software is allowed to mutate the ASG value inside the
   // algorithm object (e.g. IMTimeBasedAchieveMutableASG).
   if (im_algorithm_type == IMUtils::IMAlgorithmTypes::DISTANCEBASEDACHIEVE) {
      s.m_assigned_spacing_goal =
            Units::MetersLength(Units::NauticalMilesLength(im_algorithm->GetAssignedSpacingGoal())).value();
      s.m_predicted_spacing_interval = Units::MetersLength(Units::NauticalMilesLength(im_algorithm->GetPsi())).value();
      s.m_measured_spacing_interval = Units::MetersLength(Units::NauticalMilesLength(im_algorithm->GetMsi())).value();
   } else if (im_algorithm_type == IMUtils::IMAlgorithmTypes::KINETICACHIEVE ||
              im_algorithm_type == IMUtils::IMAlgorithmTypes::KINETICTARGETACHIEVE) {
      s.m_assigned_spacing_goal = Units::SecondsTime(im_algorithm->GetAssignedSpacingGoal()).value();
   } else if (im_algorithm_type == IMUtils::IMAlgorithmTypes::NONE) {
      s.m_assigned_spacing_goal = -INFINITY;
   } else {
      s.m_assigned_spacing_goal = Units::SecondsTime(im_algorithm->GetAssignedSpacingGoal()).value();
      s.m_predicted_spacing_interval = Units::SecondsTime(im_algorithm->GetPsi()).value();
      s.m_measured_spacing_interval = Units::SecondsTime(im_algorithm->GetMsi()).value();
   }

   s.m_imspeed_ias = im_algorithm->GetImSpeedCommandIas();
   s.m_imspeed_delayed_ias = im_algorithm->GetDelayedImSpeedCommandIas();

   s.m_ownship_ttg_to_abp = im_algorithm->GetOwnshipTtgtoAbp();
   s.m_ownship_dtg_to_abp = im_algorithm->GetOwnshipDtgtoAbp();
   s.m_ownship_dtg_to_ptp = im_algorithm->GetOwnshipDtgToPtp();
   s.m_ownship_ttg_to_ptp = im_algorithm->GetOwnshipTtgtoPtp();
   s.m_target_ttg_to_trp = im_algorithm->GetTargetTtgToTrp();
   s.m_target_dtg_to_trp = im_algorithm->GetTargetKinematicDtgToTrp();
   s.m_im_speed_limit_flags = im_algorithm->GetActiveFilter();

   s.m_ownship_reference_ias = Units::negInfinity();
   s.m_ownship_reference_groundspeed = Units::negInfinity();
   s.m_ownship_reference_altitude = Units::negInfinity();
   s.m_target_reference_ias = Units::negInfinity();
   s.m_target_reference_groundspeed = Units::negInfinity();
   s.m_target_reference_altitude = Units::negInfinity();
   s.m_target_is_aligned = -INFINITY;
   s.m_target_projected_position_x = Units::negInfinity();
   s.m_target_projected_position_y = Units::negInfinity();
   if (im_algorithm_type == IMUtils::IMAlgorithmTypes::DISTANCEBASEDACHIEVE ||
       im_algorithm_type == IMUtils::IMAlgorithmTypes::TIMEBASEDACHIEVE) {
      std::shared_ptr<IMKinematicAchieve> im_kinematic_achieve =
            std::dynamic_pointer_cast<IMKinematicAchieve>(im_algorithm);
      if (im_algorithm->GetFlightStage() == interval_management::open_source::IMAlgorithm::ACHIEVE) {
         s.m_ownship_reference_ias = im_algorithm->GetOwnshipReferenceIas();
         s.m_ownship_reference_groundspeed = im_algorithm->GetOwnshipReferenceGroundspeed();
         s.m_ownship_reference_altitude = im_algorithm->GetOwnshipReferenceAltitude();
         s.m_target_reference_ias = im_algorithm->GetTargetReferenceIas();
         s.m_target_reference_groundspeed = im_algorithm->GetTargetReferenceGroundspeed();
         s.m_target_reference_altitude = im_algorithm->GetTargetReferenceAltitude();
      } else if (im_algorithm->GetFlightStage() == IMAlgorithm::MAINTAIN) {
         // Check target aligned status. This check is not _always_ used at the start of a MAINTAIN operation.
         // things must occur in order for this check to be run. If it is not run, the boolean will be False.
         s.m_target_is_aligned = im_kinematic_achieve->IsTargetAligned();

         s.m_target_projected_position_x = im_kinematic_achieve->GetTargetStateProjectedAsgAdjusted().GetPositionX();
         s.m_target_projected_position_y = im_kinematic_achieve->GetTargetStateProjectedAsgAdjusted().GetPositionY();
      }
   }

   m_sim_data.push_back(s);
}
