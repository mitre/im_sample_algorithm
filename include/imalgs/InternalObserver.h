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

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "imalgs/AchieveObserver.h"
#include "imalgs/ClosestPointMetric.h"
#include "imalgs/CrossTrackObserver.h"
#include "imalgs/MaintainMetric.h"
#include "imalgs/MergePointMetric.h"
#include "imalgs/NMObserver.h"
#include "public/AircraftState.h"
#include "public/Guidance.h"
#include "public/WeatherPrediction.h"

namespace interval_management {
namespace open_source {

class InternalObserver final {
  public:
   static void FatalError(const char *str) {
      LOG4CPLUS_FATAL(logger, str);
      throw std::logic_error(str);
   }
   static InternalObserver *getInstance();

   void process();
   void process_NM_aircraft();
   void outputMaintainMetrics();
   void updateFinalGS(int id, double gs);
   void outputFinalGS();
   void processFinalGS();
   void outputMergePointMetric();
   void processMergePointMetric();
   void outputClosestPointMetric();
   void addPredictedWind(int id, const aaesim::open_source::WeatherPrediction &weatherPrediction);
   std::string predWindsHeading(int lastIx);
   std::string predWindsData(int id, int row, const std::string &field, const aaesim::open_source::WindStack &mat);
   std::string predTempData(int id, const std::string &field,
                            const aaesim::open_source::WeatherPrediction &weatherPrediction);
   void addAchieveRcd(size_t aircraftId, double tm, double target_ttg_to_ach, double own_ttg_to_ach,
                      double curr_distance, double reference_distance);
   NMObserver &GetNMObserver(int id);
   MaintainMetric &GetMaintainMetric(int id);
   MergePointMetric &GetMergePointMetric(int id);
   ClosestPointMetric &GetClosestPointMetric(int id);
   void set_scenario_name(const std::string &in);
   void initializeIteration();
   void setNMOutput(bool NMflag);
   bool outputNM();
   void SetRecordMaintainMetrics(bool new_value);
   const bool GetRecordMaintainMetrics() const;

   void SetScenarioIter(int scenario_iter);

   ~InternalObserver() = default;

  private:
   static std::unique_ptr<InternalObserver> m_instance;
   static log4cplus::Logger logger;

   struct AircraftIterationStats {
      MergePointMetric m_merge_point_metric{};
      MaintainMetric m_maintain_metric{};
      ClosestPointMetric m_closest_point_metric{};
      double finalGS{-1.0};
   };

   struct AircraftScenarioStats {
      NMObserver m_nm_observer{};
      std::vector<AchieveObserver> m_achieve_list{};
   };

   InternalObserver();

   void dumpPredictedWind();
   void process_NM_stats();
   void processMaintainMetrics();
   void processClosestPointMetric();
   void dumpAchieveList();

   bool outputNMFiles{true};
   bool m_save_maintain_metrics{true};
   std::string scenario_name{};
   int m_scenario_iter{0};
   CrossTrackObserver m_cross_entry{};
   std::vector<std::string> predWinds{};

   std::map<int, AircraftIterationStats> m_aircraft_iteration_stats{};
   std::map<int, AircraftScenarioStats> m_aircraft_scenario_stats{};

   std::vector<std::string> maintainOutput{};
   std::vector<std::string> finalGSOutput{};
   std::vector<std::string> mergePointOutput{};
   std::vector<std::string> closestPointOutput{};
};
}  // namespace open_source
}  // namespace interval_management
