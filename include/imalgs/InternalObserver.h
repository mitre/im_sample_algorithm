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

#include <vector>
#include <string>
#include <stdexcept>
#include "public/AircraftState.h"
#include "public/Guidance.h"
#include "public/WeatherPrediction.h"
#include "imalgs/NMObserver.h"
#include "imalgs/MergePointMetric.h"
#include "imalgs/CrossTrackObserver.h"
#include "imalgs/AchieveObserver.h"
#include "imalgs/MaintainMetric.h"
#include "imalgs/ClosestPointMetric.h"

namespace interval_management {
namespace open_source {

class InternalObserver {
  public:
   static void FatalError(const char *str) {
      LOG4CPLUS_FATAL(logger, str);
      throw std::logic_error(str);
   }
   static InternalObserver *getInstance();
   static void clearInstance();

   void process();
   void collect_ptis_b_report(Sensor::ADSB::ADSBSVReport adsb_sv_report);
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
   std::string predWindsData(int id, int row, std::string field, const aaesim::open_source::WindStack &mat);
   std::string predTempData(int id, std::string field, const aaesim::open_source::WeatherPrediction &weatherPrediction);
   void addAchieveRcd(size_t aircraftId, double tm, double target_ttg_to_ach, double own_ttg_to_ach,
                      double curr_distance, double reference_distance);
   NMObserver &GetNMObserver(int id);
   MaintainMetric &GetMaintainMetric(int id);
   MergePointMetric &GetMergePointMetric(int id);
   ClosestPointMetric &GetClosestPointMetric(int id);
   void set_scenario_name(std::string in);
   void initializeIteration();
   void setNMOutput(bool NMflag);
   bool outputNM(void);
   void SetRecordMaintainMetrics(bool new_value);
   const bool GetRecordMaintainMetrics() const;
   int GetScenarioIter() const;
   void SetScenarioIter(int scenario_iter);
   CrossTrackObserver &GetCrossEntry();

  private:
   static InternalObserver *mInstance;
   static log4cplus::Logger logger;

   class AircraftIterationStats {
     public:
      MergePointMetric m_merge_point_metric;
      MaintainMetric m_maintain_metric;
      ClosestPointMetric m_closest_point_metric;
      double finalGS;
      AircraftIterationStats();
   };

   class AircraftScenarioStats {
     public:
      NMObserver m_nm_observer;
      std::vector<AchieveObserver> m_achieve_list;
   };

   InternalObserver();
   ~InternalObserver() = default;
   void process_ptis_b_reports();
   void dumpPredictedWind();
   void process_NM_stats();
   void processMaintainMetrics();
   void processClosestPointMetric();
   void dumpAchieveList();

   bool outputNMFiles;
   bool m_save_maintain_metrics;
   std::string scenario_name;
   int m_scenario_iter;  // variable to store the current scenario iteration
   CrossTrackObserver m_cross_entry;
   std::vector<std::string> predWinds;

   // Data for individual aircraft
   std::map<int, AircraftIterationStats> m_aircraft_iteration_stats;  // cleared between iterations
   std::map<int, AircraftScenarioStats> m_aircraft_scenario_stats;    // never cleared

   // output data vectors
   std::vector<Sensor::ADSB::ADSBSVReport> ptis_b_report_list;

   // string vectors for file output
   std::vector<std::string> maintainOutput;
   std::vector<std::string> finalGSOutput;
   std::vector<std::string> mergePointOutput;
   std::vector<std::string> closestPointOutput;
};
}  // namespace open_source
}  // namespace interval_management
