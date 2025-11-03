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

#include "imalgs/InternalObserver.h"

#include <iomanip>

#include "public/StereographicProjection.h"

using namespace std;
using namespace aaesim::open_source::constants;
using namespace interval_management::open_source;

std::unique_ptr<InternalObserver> InternalObserver::m_instance = nullptr;

log4cplus::Logger InternalObserver::logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("InternalObserver"));

InternalObserver *InternalObserver::getInstance() {
   if (m_instance == NULL) {
      m_instance = std::unique_ptr<InternalObserver>(new InternalObserver());
   }
   return m_instance.get();
}

InternalObserver::InternalObserver() = default;

void InternalObserver::process() {
   dumpPredictedWind();
   process_NM_stats();
   processMaintainMetrics();
   processFinalGS();
   processMergePointMetric();
   processClosestPointMetric();
   dumpAchieveList();
}

void InternalObserver::set_scenario_name(const std::string &in) { scenario_name = in; }

MergePointMetric &InternalObserver::GetMergePointMetric(int id) {
   auto return_val = m_aircraft_iteration_stats.insert(std::make_pair(id, AircraftIterationStats()));
   return return_val.first->second.m_merge_point_metric;
}

MaintainMetric &InternalObserver::GetMaintainMetric(int id) {
   auto return_val = m_aircraft_iteration_stats.insert(std::make_pair(id, AircraftIterationStats()));
   return return_val.first->second.m_maintain_metric;
}

ClosestPointMetric &InternalObserver::GetClosestPointMetric(int id) {
   auto return_val = m_aircraft_iteration_stats.insert(std::make_pair(id, AircraftIterationStats()));
   return return_val.first->second.m_closest_point_metric;
}

NMObserver &InternalObserver::GetNMObserver(int id) {
   auto return_val = m_aircraft_scenario_stats.insert(std::make_pair(id, AircraftScenarioStats()));
   return return_val.first->second.m_nm_observer;
}

void InternalObserver::SetScenarioIter(int scenario_iter) { this->m_scenario_iter = scenario_iter; }

void InternalObserver::process_NM_aircraft() {
   if (outputNM()) {
      for (auto ix = m_aircraft_scenario_stats.begin(); ix != m_aircraft_scenario_stats.end(); ++ix) {
         NMObserver &nm_observer = ix->second.m_nm_observer;

         if (!nm_observer.entry_list.empty()) {
            char *temp = new char[10];

            snprintf(temp, 10, "%d", ix->first);

            string output_file_name = scenario_name + "_aircraft_" + temp + "_nm_output.csv";
            delete[] temp;

            ofstream out;

            if (m_scenario_iter == 0) {
               out.open(output_file_name.c_str());
            } else {
               out.open(output_file_name.c_str(), ios::out | ios::app);
            }

            if (out.is_open()) {
               if (m_scenario_iter == 0) {
                  out << "AC_ID,Iteration,Predicted_Distance(NM),True_Distance(NM),Time,Own_Command_IAS(Knots),Own_"
                         "Current_GroundSpeed(Knots),Target_GroundSpeed(Knots),Min_IAS_Command(Knots),Max_IAS_Command("
                         "Knots),Min_GS_Command(Knots),Max_GS_Command(Knots)"
                      << endl;
               }

               nm_observer.initialize_stats();

               for (unsigned int index = 0; index < nm_observer.entry_list.size(); index++) {
                  out << ix->first << ",";
                  out << m_scenario_iter << ",";
                  out << nm_observer.entry_list[index].predictedDistance / NAUTICAL_MILES_TO_METERS << ",";
                  out << nm_observer.entry_list[index].trueDistance / NAUTICAL_MILES_TO_METERS << ",";
                  out << nm_observer.entry_list[index].time << ",";
                  out << nm_observer.entry_list[index].acIAS / KNOTS_TO_METERS_PER_SECOND << ",";
                  out << nm_observer.entry_list[index].acGS / KNOTS_TO_METERS_PER_SECOND << ",";
                  out << nm_observer.entry_list[index].targetGS / KNOTS_TO_METERS_PER_SECOND << ",";
                  out << nm_observer.entry_list[index].minIAS / KNOTS_TO_METERS_PER_SECOND << ",";
                  out << nm_observer.entry_list[index].maxIAS / KNOTS_TO_METERS_PER_SECOND << ",";
                  out << nm_observer.entry_list[index].minTAS / KNOTS_TO_METERS_PER_SECOND << ",";
                  out << nm_observer.entry_list[index].maxTAS / KNOTS_TO_METERS_PER_SECOND << endl;

                  nm_observer.predictedDistance[index] =
                        nm_observer.entry_list[index].predictedDistance / NAUTICAL_MILES_TO_METERS;
                  nm_observer.trueDistance[index] =
                        nm_observer.entry_list[index].trueDistance / NAUTICAL_MILES_TO_METERS;
                  nm_observer.ac_IAS_stats[index].Insert(nm_observer.entry_list[index].acIAS /
                                                         KNOTS_TO_METERS_PER_SECOND);
                  nm_observer.ac_GS_stats[index].Insert(nm_observer.entry_list[index].acGS /
                                                        KNOTS_TO_METERS_PER_SECOND);
                  nm_observer.target_GS_stats[index].Insert(nm_observer.entry_list[index].targetGS /
                                                            KNOTS_TO_METERS_PER_SECOND);
                  nm_observer.min_IAS_stats[index].Insert(nm_observer.entry_list[index].minIAS /
                                                          KNOTS_TO_METERS_PER_SECOND);
                  nm_observer.max_IAS_stats[index].Insert(nm_observer.entry_list[index].maxIAS /
                                                          KNOTS_TO_METERS_PER_SECOND);
               }

               nm_observer.entry_list.clear();
               nm_observer.curr_NM = -2;
               out.close();
            }
         }
      }
   }
}

void InternalObserver::process_NM_stats() {
   if (outputNM()) {
      for (auto ix = m_aircraft_scenario_stats.begin(); ix != m_aircraft_scenario_stats.end(); ++ix) {
         NMObserver &nm_observer = ix->second.m_nm_observer;

         if (nm_observer.predictedDistance.size() > 0) {
            char *temp = new char[10];

            snprintf(temp, 10, "%d", ix->first);

            string output_file_name = scenario_name + "_aircraft_" + temp + "_stats_nm_output.csv";

            delete[] temp;

            ofstream out;

            out.open(output_file_name.c_str(), ios::out);

            if (out.is_open()) {
               out << "Predicted_Distance,True_Distance,AC_IAS_Mean,AC_IAS_Dev,AC_GS_Mean,AC_GS_Dev,Target_GS_Mean,"
                      "Target_GS_Dev,Min_Mean,Min_Dev,Max_Mean,Max_Dev"
                   << endl;

               for (unsigned int index = 0; index < nm_observer.predictedDistance.size(); index++) {
                  out << nm_observer.predictedDistance[index] << ",";
                  out << nm_observer.trueDistance[index] << ",";
                  out << nm_observer.ac_IAS_stats[index].GetMean() << ",";
                  out << nm_observer.ac_IAS_stats[index].ComputeStandardDeviation() << ",";
                  out << nm_observer.ac_GS_stats[index].GetMean() << ",";
                  out << nm_observer.ac_GS_stats[index].ComputeStandardDeviation() << ",";
                  out << nm_observer.target_GS_stats[index].GetMean() << ",";
                  out << nm_observer.target_GS_stats[index].ComputeStandardDeviation() << ",";
                  out << nm_observer.min_IAS_stats[index].GetMean() << ",";
                  out << nm_observer.min_IAS_stats[index].ComputeStandardDeviation() << ",";
                  out << nm_observer.max_IAS_stats[index].GetMean() << ",";
                  out << nm_observer.max_IAS_stats[index].ComputeStandardDeviation() << endl;
               }

               out.close();
            }
         }
      }
   }
}

void InternalObserver::initializeIteration() {
   m_aircraft_iteration_stats.clear();
   predWinds.clear();
}

void InternalObserver::outputMaintainMetrics() {
   string body;
   char bfr[121];

   if (maintainOutput.empty()) {
      body = "Iteration";

      for (auto ix = m_aircraft_iteration_stats.begin(); ix != m_aircraft_iteration_stats.end(); ++ix) {
         int acid = ix->first;
         MaintainMetric &maintain_metric = ix->second.m_maintain_metric;
         if (!maintain_metric.IsOutputEnabled()) continue;
         snprintf(bfr, sizeof(bfr), ",ac %d-mean,ac %d-stdev,ac %d-95bound,ac %d-maintainTime,ac %d-timeGreaterThan10",
                  acid, acid, acid, acid, acid);
         body = body + bfr;
      }

      maintainOutput.push_back(body);
   }

   snprintf(bfr, sizeof(bfr), "%d", ((int)maintainOutput.size() - 1));
   body = bfr;

   for (auto ix = m_aircraft_iteration_stats.begin(); ix != m_aircraft_iteration_stats.end(); ++ix) {
      MaintainMetric &maintain_metric = ix->second.m_maintain_metric;
      if (!maintain_metric.IsOutputEnabled()) continue;
      if (maintain_metric.hasSamples()) {
         snprintf(bfr, sizeof(bfr), ",%f,%f,%f,%f,%d", maintain_metric.getMeanErr(), maintain_metric.getStdErr(),
                  maintain_metric.getBound95(), maintain_metric.getTotMaintain(), maintain_metric.getNumCycles());
      } else {
         snprintf(bfr, sizeof(bfr), ",No samples,,,,");
      }

      body = body + bfr;
   }

   maintainOutput.push_back(body);
}

void InternalObserver::processMaintainMetrics() {
   string output_file_name = scenario_name + "_maintain_metrics.csv";
   ofstream out;
   out.open(output_file_name.c_str());

   if (out.is_open()) {
      for (size_t ix = 0; ix < maintainOutput.size(); ix++) {
         out << maintainOutput[ix] << endl;
      }

      out.close();
   }

   maintainOutput.clear();
}

void InternalObserver::updateFinalGS(int id, double gs) {
   if (id >= 0) {
      auto return_val = m_aircraft_iteration_stats.insert(std::make_pair(id, AircraftIterationStats()));
      return_val.first->second.finalGS = gs;
   }
}

void InternalObserver::outputFinalGS() {
   string body;
   char bfr[51];

   if (finalGSOutput.empty()) {
      body = "Iteration";

      for (auto ix = m_aircraft_iteration_stats.begin(); ix != m_aircraft_iteration_stats.end(); ++ix) {
         snprintf(bfr, sizeof(bfr), ",ac %d-gs", ix->first);
         body = body + bfr;
      }

      finalGSOutput.push_back(body);
   }

   snprintf(bfr, sizeof(bfr), "%d", ((int)finalGSOutput.size() - 1));
   body = bfr;

   for (auto ix = m_aircraft_iteration_stats.begin(); ix != m_aircraft_iteration_stats.end(); ++ix) {
      snprintf(bfr, sizeof(bfr), ",%f", ix->second.finalGS);
      body = body + bfr;
   }

   finalGSOutput.push_back(body);
}

void InternalObserver::processFinalGS() {
   string output_file_name = scenario_name + "_final_groundspeed.csv";
   ofstream out;
   out.open(output_file_name.c_str());

   if (out.is_open()) {
      for (size_t ix = 0; ix < finalGSOutput.size(); ix++) {
         out << finalGSOutput[ix] << endl;
      }

      out.close();
   }

   finalGSOutput.clear();
}

void InternalObserver::outputMergePointMetric() {
   string body;
   char bfr[61];

   if (mergePointOutput.empty()) {
      body = "Iteration";

      for (auto ix = m_aircraft_iteration_stats.begin(); ix != m_aircraft_iteration_stats.end(); ++ix) {
         MergePointMetric &merge_point_metric = ix->second.m_merge_point_metric;
         if (merge_point_metric.willReportMetrics()) {
            int id1 = merge_point_metric.GetImAcId();
            int id0 = merge_point_metric.GetTargetAcId();
            snprintf(bfr, sizeof(bfr), ",ac %d-mergePt,ac %d-distTo ac %d", id1, id1, id0);
            body = body + bfr;
         }
      }

      mergePointOutput.push_back(body);
   }

   snprintf(bfr, sizeof(bfr), "%d", ((int)mergePointOutput.size() - 1));
   body = bfr;

   for (auto ix = m_aircraft_iteration_stats.begin(); ix != m_aircraft_iteration_stats.end(); ++ix) {
      MergePointMetric &merge_point_metric = ix->second.m_merge_point_metric;
      if (merge_point_metric.willReportMetrics()) {
         snprintf(bfr, sizeof(bfr), ",%s,%f", merge_point_metric.getMergePoint().c_str(),
                  Units::NauticalMilesLength(merge_point_metric.getDist()).value());
         body = body + bfr;
      }
   }

   mergePointOutput.push_back(body);
}

void InternalObserver::processMergePointMetric() {
   string output_file_name = scenario_name + "_merge_point_metric.csv";
   ofstream out;
   out.open(output_file_name.c_str());

   if (out.is_open()) {
      for (size_t ix = 0; ix < mergePointOutput.size(); ix++) {
         out << mergePointOutput[ix] << endl;
      }

      out.close();
   }

   // Clear report vector.

   mergePointOutput.clear();
}

void InternalObserver::outputClosestPointMetric() {
   string body;
   char bfr[61];

   if (closestPointOutput.empty()) {
      body = "Iteration";
      for (auto ix = m_aircraft_iteration_stats.begin(); ix != m_aircraft_iteration_stats.end(); ++ix) {
         ClosestPointMetric &closest_point_metric = ix->second.m_closest_point_metric;
         if (closest_point_metric.IsReportMetrics()) {
            snprintf(bfr, sizeof(bfr), ",ac %d-smallestDistTo ac %d", closest_point_metric.GetImAcId(),
                     closest_point_metric.GetTargetAcId());
            body = body + bfr;
         }
      }

      closestPointOutput.push_back(body);
   }

   snprintf(bfr, sizeof(bfr), "%d", ((int)closestPointOutput.size() - 1));
   body = bfr;

   for (auto ix = m_aircraft_iteration_stats.begin(); ix != m_aircraft_iteration_stats.end(); ++ix) {
      ClosestPointMetric &closest_point_metric = ix->second.m_closest_point_metric;
      if (closest_point_metric.IsReportMetrics()) {
         snprintf(bfr, sizeof(bfr), ",%f", Units::NauticalMilesLength(closest_point_metric.getMinDist()).value());
         body = body + bfr;
      }
   }

   closestPointOutput.push_back(body);
}

void InternalObserver::processClosestPointMetric() {
   string output_file_name = scenario_name + "_closest_point_metric.csv";
   ofstream out;
   out.open(output_file_name.c_str());

   if (out.is_open()) {
      for (size_t ix = 0; ix < closestPointOutput.size(); ix++) {
         out << closestPointOutput[ix] << endl;
      }

      out.close();
   }

   closestPointOutput.clear();
}

void InternalObserver::addPredictedWind(int id, const aaesim::open_source::WeatherPrediction &weatherPrediction) {
   if (predWinds.empty()) {
      predWinds.push_back(predWindsHeading(weatherPrediction.east_west().GetMaxRow()));
   }

   predWinds.push_back(predWindsData(id, 1, "Alt(feet)", weatherPrediction.east_west()));
   predWinds.push_back(predWindsData(id, 2, "XSpeed(Knots)", weatherPrediction.east_west()));
   predWinds.push_back(predWindsData(id, 2, "YSpeed(Knots)", weatherPrediction.north_south()));
   predWinds.push_back(predTempData(id, "Temperature(C)", weatherPrediction));
}

string InternalObserver::predWindsHeading(int numVals) {
   string hdr = "Aircraft_id,Field";

   for (int i = 1; i <= numVals; i++) {
      hdr += ",";
   }

   return hdr;
}

string InternalObserver::predWindsData(int id, int col, string field, const aaesim::open_source::WindStack &mat) {
   string str;

   char *txt = new char[31];

   snprintf(txt, 301, "%d", id);
   str = txt;
   str += ",";
   str += field.c_str();

   for (int i = 1; i <= mat.GetMaxRow(); i++) {
      switch (col) {
         case 1:
            snprintf(txt, 301, ",%lf", mat.GetAltitude(i).value());
            break;
         case 2:
            snprintf(txt, 301, ",%lf", mat.GetSpeed(i).value());
      }
      str += txt;
   }

   delete[] txt;

   return str;
}

string InternalObserver::predTempData(int id, string field,
                                      const aaesim::open_source::WeatherPrediction &weatherPrediction) {
   string str;

   char *txt = new char[31];

   snprintf(txt, 31, "%d", id);
   str = txt;

   str += ",";
   str += field.c_str();

   const aaesim::open_source::WindStack &mat(weatherPrediction.east_west());
   for (int i = 1; i <= mat.GetMaxRow(); i++) {
      Units::Length alt = mat.GetAltitude(i);
      Units::KelvinTemperature temperature = weatherPrediction.GetForecastAtmosphere()->GetTemperature(alt);
      snprintf(txt, 31, ",%lf", temperature.value() - 273.15);
      str += txt;
   }

   delete[] txt;

   return str;
}

void InternalObserver::dumpPredictedWind() {
   string fileName = scenario_name + "_predicted_winds.csv";
   ofstream out;
   out.open(fileName.c_str());

   if (out.is_open()) {
      for (size_t ix = 0; ix < predWinds.size(); ix++) {
         out << predWinds[ix] << endl;
      }

      out.close();
   }

   predWinds.clear();
}

void InternalObserver::addAchieveRcd(size_t aircraftId, double tm, double target_ttg_to_ach, double own_ttg_to_ach,
                                     double curr_distance, double reference_distance) {
   AchieveObserver achievercd(this->m_scenario_iter, aircraftId, tm, target_ttg_to_ach, own_ttg_to_ach, curr_distance,
                              reference_distance);
   m_aircraft_scenario_stats[aircraftId].m_achieve_list.push_back(achievercd);
}

void InternalObserver::dumpAchieveList() {
   string fileName = scenario_name + "_time_to_go.csv";
   ofstream out;

   bool needHdr = true;

   for (auto ix = m_aircraft_scenario_stats.begin(); ix != m_aircraft_scenario_stats.end(); ++ix) {
      vector<AchieveObserver> &achieve_list = ix->second.m_achieve_list;

      if (achieve_list.empty()) {
         continue;
      }

      if (needHdr) {
         out.open(fileName.c_str());
         if (!out.is_open()) {
            LOG4CPLUS_ERROR(logger, "Cannot open " << fileName << " for achieve list output.");
            return;
         }
         out << achieve_list[0].Hdr().c_str() << endl;
         needHdr = false;
      }

      for (size_t ix = 0; ix < achieve_list.size(); ix++) {
         out << achieve_list[ix].ToString().c_str() << endl;
      }
   }
   if (out.is_open()) out.close();
}

void InternalObserver::setNMOutput(bool NMflag) { this->outputNMFiles = NMflag; }

bool InternalObserver::outputNM() { return this->outputNMFiles; }

void InternalObserver::SetRecordMaintainMetrics(bool new_value) { m_save_maintain_metrics = new_value; }

const bool InternalObserver::GetRecordMaintainMetrics() const { return m_save_maintain_metrics; }
