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
using namespace interval_management::open_source;

InternalObserver *InternalObserver::mInstance = NULL;
log4cplus::Logger InternalObserver::logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("InternalObserver"));

InternalObserver *InternalObserver::getInstance() {
   if (mInstance == NULL) {
      mInstance = new InternalObserver();
   }
   return mInstance;
}

void InternalObserver::clearInstance() {
   if (mInstance != NULL) {
      delete mInstance;
      mInstance = NULL;  // blow away the instance
   }
}

InternalObserver::InternalObserver() {
   m_save_maintain_metrics = true;
   m_scenario_iter = 0;
   outputNMFiles = true;
}

void InternalObserver::process(void) {
   dumpPredictedWind();
   process_NM_stats();
   processMaintainMetrics();
   processFinalGS();
   processMergePointMetric();
   processClosestPointMetric();
   process_ptis_b_reports();
   dumpAchieveList();
}

void InternalObserver::set_scenario_name(string in) { scenario_name = in; }

MergePointMetric &InternalObserver::GetMergePointMetric(int id) {
   return m_aircraft_iteration_stats[id].m_merge_point_metric;
}

MaintainMetric &InternalObserver::GetMaintainMetric(int id) { return m_aircraft_iteration_stats[id].m_maintain_metric; }

ClosestPointMetric &InternalObserver::GetClosestPointMetric(int id) {
   return m_aircraft_iteration_stats[id].m_closest_point_metric;
}

NMObserver &InternalObserver::GetNMObserver(int id) { return m_aircraft_scenario_stats[id].m_nm_observer; }

int InternalObserver::GetScenarioIter() const { return m_scenario_iter; }

void InternalObserver::SetScenarioIter(int scenario_iter) { this->m_scenario_iter = scenario_iter; }

// outputs the Nautical Mile report for all aircraft
void InternalObserver::process_NM_aircraft() {
   if (outputNM()) {
      // Write NM report if we are outputting NM files.

      // loop to process all of the aircraft NM reports
      for (auto ix = m_aircraft_scenario_stats.begin(); ix != m_aircraft_scenario_stats.end(); ++ix) {
         NMObserver &nm_observer = ix->second.m_nm_observer;
         // if the current aircraft has Nautical Mile output entries output them
         if (!nm_observer.entry_list.empty()) {
            char *temp = new char[10];

            sprintf(temp, "%d", ix->first);

            string output_file_name = scenario_name + "_AC" + temp + "-NM-output.csv";
            delete[] temp;

            ofstream out;

            // if first iteration create file, otherwise append file
            if (m_scenario_iter == 0) {
               out.open(output_file_name.c_str());
            } else {
               out.open(output_file_name.c_str(), ios::out | ios::app);
            }

            // if file opens properly and entry list isn't empty output the Nautical Mile results
            if (out.is_open()) {
               // if first iteration create header

               if (m_scenario_iter == 0) {
                  out << "AC_ID,Iteration,Predicted_Distance(NM),True_Distance(NM),Time,Own_Command_IAS(Knots),Own_"
                         "Current_GroundSpeed(Knots),Target_GroundSpeed(Knots),Min_IAS_Command(Knots),Max_IAS_Command("
                         "Knots),Min_GS_Command(Knots),Max_GS_Command(Knots)"
                      << endl;
               }

               nm_observer.initialize_stats();  // initialize the statistics to the size of the entry list

               // loop to process all aircraft entries
               for (unsigned int index = 0; index < nm_observer.entry_list.size(); index++) {
                  // output the report
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

                  // add entries to Statistics class
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
               nm_observer.curr_NM = -2;  // resets the current NM value
               out.close();
            }
         }
      }
   }
}

void InternalObserver::process_NM_stats() {

   // Write NM stats output NM files being processed.
   if (outputNM()) {

      // loop to process all of the aircraft NM reports
      for (auto ix = m_aircraft_scenario_stats.begin(); ix != m_aircraft_scenario_stats.end(); ++ix) {
         NMObserver &nm_observer = ix->second.m_nm_observer;

         if (nm_observer.predictedDistance.size() > 0) {
            char *temp = new char[10];

            sprintf(temp, "%d", ix->first);

            string output_file_name = scenario_name + "_AC" + temp + "-stats-NM-output.csv";

            delete[] temp;

            ofstream out;

            out.open(output_file_name.c_str(), ios::out);

            // if file opens properly and entry list isn't empty output the Nautical Mile statistics
            if (out.is_open()) {
               out << "Predicted_Distance,True_Distance,AC_IAS_Mean,AC_IAS_Dev,AC_GS_Mean,AC_GS_Dev,Target_GS_Mean,"
                      "Target_GS_Dev,Min_Mean,Min_Dev,Max_Mean,Max_Dev"
                   << endl;

               // loop to process all distance entry statistics
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
   // Post processes the maintain metric data after each iteration,
   // forming a string for each iteration and placing it in a local
   // string vector.

   string body;
   char bfr[121];

   // Add header.

   if (maintainOutput.size() == 0) {
      body = "Iteration";

      for (auto ix = m_aircraft_iteration_stats.begin(); ix != m_aircraft_iteration_stats.end(); ++ix) {
         int acid = ix->first;
         MaintainMetric &maintain_metric = ix->second.m_maintain_metric;
         if (!maintain_metric.IsOutputEnabled()) continue;
         sprintf(bfr, ",ac %d-mean,ac %d-stdev,ac %d-95bound,ac %d-maintainTime,ac %d-timeGreaterThan10", acid, acid,
                 acid, acid, acid);
         body = body + bfr;
      }

      maintainOutput.push_back(body);
   }

   // Add body.
   sprintf(bfr, "%d", ((int)maintainOutput.size() - 1));  // Iteration
   body = bfr;

   for (auto ix = m_aircraft_iteration_stats.begin(); ix != m_aircraft_iteration_stats.end(); ++ix) {
      MaintainMetric &maintain_metric = ix->second.m_maintain_metric;
      if (!maintain_metric.IsOutputEnabled()) continue;
      if (maintain_metric.hasSamples()) {
         sprintf(bfr, ",%f,%f,%f,%f,%d", maintain_metric.getMeanErr(), maintain_metric.getStdErr(),
                 maintain_metric.getBound95(), maintain_metric.getTotMaintain(), maintain_metric.getNumCycles());
      } else {
         sprintf(bfr, ",No samples,,,,");
      }

      body = body + bfr;
   }

   maintainOutput.push_back(body);
}

void InternalObserver::processMaintainMetrics() {

   // Output maintain metrics .csv file.

   string output_file_name = scenario_name + "-Maintain-Metrics.csv";
   ofstream out;
   out.open(output_file_name.c_str());

   if (out.is_open()) {
      for (size_t ix = 0; ix < maintainOutput.size(); ix++) {
         out << maintainOutput[ix] << endl;
      }

      out.close();
   }

   // Clear report vector.

   maintainOutput.clear();
}

void InternalObserver::updateFinalGS(int id, double gs) {

   // Stores/replaces final ground speed for a aircraft.
   //
   // id:id of aircraft.
   // gs:ground speed.

   if (id >= 0) {
      m_aircraft_iteration_stats[id].finalGS = gs;
   }
}

void InternalObserver::outputFinalGS() {

   // Post processes final ground speed data after each iteration,
   // forming a string for each iteration and placing it in a local
   // string vector.

   string body;
   char bfr[51];

   // Add header.

   if (finalGSOutput.size() == 0) {
      body = "Iteration";

      for (auto ix = m_aircraft_iteration_stats.begin(); ix != m_aircraft_iteration_stats.end(); ++ix) {
         sprintf(bfr, ",ac %d-gs", ix->first);
         body = body + bfr;
      }

      finalGSOutput.push_back(body);
   }

   // Add body.
   sprintf(bfr, "%d", ((int)finalGSOutput.size() - 1));  // Iteration // TODO:A better way of determining iteration.
   body = bfr;

   for (auto ix = m_aircraft_iteration_stats.begin(); ix != m_aircraft_iteration_stats.end(); ++ix) {
      sprintf(bfr, ",%f", ix->second.finalGS);
      body = body + bfr;
   }

   // Add string to output.

   finalGSOutput.push_back(body);
}

void InternalObserver::processFinalGS() {

   // Outputs the final groundspeed .csv file.

   // Open file

   string output_file_name = scenario_name + "-Final-Groundspeed.csv";
   ofstream out;
   out.open(output_file_name.c_str());

   if (out.is_open()) {
      for (size_t ix = 0; ix < finalGSOutput.size(); ix++) {
         out << finalGSOutput[ix] << endl;
      }

      out.close();
   }

   // Clear report vector.

   finalGSOutput.clear();
}

void InternalObserver::outputMergePointMetric() {

   // Creates report for merge point metric, first a column header
   // and then for each iteration, one line with merge point stats.
   // Each line is a string stored in an output vector.

   string body;
   char bfr[61];

   // Add header.

   if (mergePointOutput.size() == 0) {
      body = "Iteration";

      for (auto ix = m_aircraft_iteration_stats.begin(); ix != m_aircraft_iteration_stats.end(); ++ix) {
         MergePointMetric &merge_point_metric = ix->second.m_merge_point_metric;
         if (merge_point_metric.willReportMetrics()) {
            int id1 = merge_point_metric.GetImAcId();
            int id0 = merge_point_metric.GetTargetAcId();
            sprintf(bfr, ",ac %d-mergePt,ac %d-distTo ac %d", id1, id1, id0);
            body = body + bfr;
         }
      }

      mergePointOutput.push_back(body);
   }

   // Add body.
   sprintf(bfr, "%d", ((int)mergePointOutput.size() - 1));  // Iteration
   body = bfr;

   for (auto ix = m_aircraft_iteration_stats.begin(); ix != m_aircraft_iteration_stats.end(); ++ix) {
      MergePointMetric &merge_point_metric = ix->second.m_merge_point_metric;
      if (merge_point_metric.willReportMetrics()) {
         sprintf(bfr, ",%s,%f", merge_point_metric.getMergePoint().c_str(),
                 Units::NauticalMilesLength(merge_point_metric.getDist()).value());
         body = body + bfr;
      }
   }

   mergePointOutput.push_back(body);
}

void InternalObserver::processMergePointMetric() {

   // Output merge point metric to a .csv file.

   string output_file_name = scenario_name + "-Merge-Point-Metric.csv";
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

   // Creates report text for the closest point metric.
   // A column header is created the first time through.
   // A line containing the closest point metric stats is
   // created for each iteration.

   string body;
   char bfr[61];

   // Add header.

   if (closestPointOutput.size() == 0) {
      body = "Iteration";
      for (auto ix = m_aircraft_iteration_stats.begin(); ix != m_aircraft_iteration_stats.end(); ++ix) {
         ClosestPointMetric &closest_point_metric = ix->second.m_closest_point_metric;
         if (closest_point_metric.IsReportMetrics()) {
            sprintf(bfr, ",ac %u-smallestDistTo ac %u", closest_point_metric.GetImAcId(),
                    closest_point_metric.GetTargetAcId());
            body = body + bfr;
         }
      }

      closestPointOutput.push_back(body);
   }

   // Add body.
   sprintf(bfr, "%d", ((int)closestPointOutput.size() - 1));  // Iteration
   body = bfr;

   for (auto ix = m_aircraft_iteration_stats.begin(); ix != m_aircraft_iteration_stats.end(); ++ix) {
      ClosestPointMetric &closest_point_metric = ix->second.m_closest_point_metric;
      if (closest_point_metric.IsReportMetrics()) {
         sprintf(bfr, ",%f", Units::NauticalMilesLength(closest_point_metric.getMinDist()).value());
         body = body + bfr;
      }
   }

   closestPointOutput.push_back(body);
}

void InternalObserver::processClosestPointMetric() {

   // Output closest point metric to a .csv file.

   string output_file_name = scenario_name + "-Closest-Point-Metric.csv";
   ofstream out;
   out.open(output_file_name.c_str());

   if (out.is_open()) {
      for (size_t ix = 0; ix < closestPointOutput.size(); ix++) {
         out << closestPointOutput[ix] << endl;
      }

      out.close();
   }

   // Clear report vector.

   closestPointOutput.clear();
}

// collect pTIS_B reports into this class
void InternalObserver::collect_ptis_b_report(Sensor::ADSB::ADSBSVReport adsb_sv_report) {
   ptis_b_report_list.push_back(adsb_sv_report);
}

void InternalObserver::process_ptis_b_reports()  // process the ADS-B reports
{
   // Figure the maximum id out of all the IDs of  in the receiver_id field in ptis_b_ether_with_receiver_ID_list
   int max_id = -100;
   // loop through the receiver_id fields in ptis_b_ether_with_receiver_ID_list
   for (size_t i = 0; i < ptis_b_report_list.size(); i++) {
      int this_id = ptis_b_report_list[i].GetId();
      if (this_id > max_id) {
         max_id = this_id;
      }
   }

   // loop through all ac ids
   for (int ac_id = 0; ac_id <= max_id; ac_id++) {

      // open report data file
      std::ostringstream ostr_ac_id;           // output string stream
      ostr_ac_id << ac_id;                     // use the string stream to convert ac_id into an output string stream
      string ac_id_string = ostr_ac_id.str();  // convert to string

      string output_file_name = scenario_name + "-TIS-B-Report-output-TargetACID-" + ac_id_string + ".csv";
      ofstream out;
      out.open(output_file_name.c_str());

      // if file opens successfully, process the output
      if (out.is_open() == true && ptis_b_report_list.empty() == false) {
         // print the header
         out << "TOA,24bitAddress,Lat,Lon,Alt,EWVel,NSVel,NACp,NIC,NACv,SIL,SDA,VertRate" << endl;

         for (size_t i = 0; i < ptis_b_report_list.size(); i++) {
            Sensor::ADSB::ADSBSVReport return_report;
            return_report = ptis_b_report_list[i];
            if (return_report.GetId() == ac_id) {
               // print out current record
               out << return_report.GetTime().value() << ",";  // outputs the TOA
               out << return_report.GetId() << ",";            // output  id
               Units::DegreesAngle lat_out, long_out;
               StereographicProjection::xy_to_ll(
                     Units::FeetLength(return_report.GetX()), Units::FeetLength(return_report.GetY()), lat_out,
                     long_out);  // call the Stereographic Projection to convert the aircraft X/Y to Lat/Long
               out.precision(10);
               out << lat_out.value() << ",";               // output the  lat in degrees
               out << long_out.value() << ",";              // output lon in degrees
               out << return_report.GetZ().value() << ",";  // output the current altitude value in feet
               out << Units::KnotsSpeed(return_report.GetXd()).value()
                   << ",";  // output the current x velocity in knots; the unit of return_report.getxd is assumed to be
                            // feet/second
               out << Units::KnotsSpeed(return_report.GetYd()).value()
                   << ",";  // output the current y velocity in knots; the unit of return_report.getyd is assumed to be
                            // feet/second
               out << return_report.GetNacp() << ",";  // output the NACp
               out << return_report.GetNicp() << ",";  // output the NICp
               out << return_report.GetNacv() << ",";  // output the NACv
               out << 2 << ",";                        // output the SIL (set at 2)
               out << 2 << ",";                        // output the SDA (set at 2)
               out << Units::FeetPerMinuteSpeed(return_report.GetZd()).value()
                   << endl;  // output the current vertical velocity feet per minute; the unit of return_report.zd is
                             // assumed to be feet/second
            }                // end if(return_report.id == ac_id)
         }                   // end for(int i = 0; i <  ptis_b_ether_with_receiver_ID_list.size(); i++)
         out.close();
      }  // end if( out.is_open() == true && ads_b_ether_list.empty() == false)
   }     // end for(int ac_id = 0; ac_id <= max_id; ac_id++)
}

void InternalObserver::addPredictedWind(int id, const aaesim::open_source::WeatherPrediction &weatherPrediction) {
   // Adds predicted wind entry for an aircraft.
   //
   // id:aircraft id.
   // weatherPrediction.east_west, weatherPrediction.north_south:predicted wind data for aircraft.
   //                      altitudes in feet, wind speeds in knots.

   // Add header.
   if (predWinds.size() == 0) {
      predWinds.push_back(predWindsHeading(weatherPrediction.east_west.GetMaxRow()));
   }

   // Add altitudes, x speed, y speed.
   predWinds.push_back(predWindsData(id, 1, "Alt(feet)", weatherPrediction.east_west));
   predWinds.push_back(predWindsData(id, 2, "XSpeed(Knots)", weatherPrediction.east_west));
   predWinds.push_back(predWindsData(id, 2, "YSpeed(Knots)", weatherPrediction.north_south));
   predWinds.push_back(predTempData(id, "Temperature(C)", weatherPrediction));
}

string InternalObserver::predWindsHeading(int numVals) {
   // Formats header for predicted winds metric.
   //
   // numVals:number of values in the predicted winds matrices where
   //         each value is an altitude, speed pair.
   //
   // returns header line.

   string hdr = "Aircraft_id,Field";

   // Only aircraft id for now.  This will need clarification from Lesley.
   // Still need to add a blank column title for each column.

   for (int i = 1; i <= numVals; i++) {
      hdr += ",";
   }

   return hdr;
}

string InternalObserver::predWindsData(int id, int col, string field, const aaesim::open_source::WindStack &mat) {
   // Formats data line for predicted winds metric for an aircraft
   // for a data row.  With respect between the data line output and
   // how the wind matrices are setup, the rows and columns are
   // inverted.  Altitudes output in meters, speeds in meters/second.
   //
   // id:aircraft id.
   // field:field name.
   // col:col of data being formatted-1 for altitude, 2 for speed.
   // mat:matrix containing data to format into string.
   //
   // returns data line.

   string str;

   char *txt = new char[31];

   // Aircraft id

   sprintf(txt, "%d", id);
   str = txt;

   // Field

   str += ",";
   str += field.c_str();

   // Data line-all in meters, meters/second.

   for (int i = 1; i <= mat.GetMaxRow(); i++) {
      switch (col) {
         case 1:
            sprintf(txt, ",%lf", mat.GetAltitude(i).value());
            break;
         case 2:
            sprintf(txt, ",%lf", mat.GetSpeed(i).value());
      }
      str += txt;
   }

   delete[] txt;

   return str;
}

string InternalObserver::predTempData(int id, string field,
                                      const aaesim::open_source::WeatherPrediction &weatherPrediction) {
   // Formats data line for predicted winds metric for an aircraft
   // for a data row.  With respect between the data line output and
   // how the wind matrices are setup, the rows and columns are
   // inverted.  Altitudes output in meters, speeds in meters/second.
   //
   // id:aircraft id.
   // field:field name.
   // col:col of data being formatted-1 for altitude, 2 for speed.
   // mat:matrix containing data to format into string.
   //
   // returns data line.

   string str;

   char *txt = new char[31];

   // Aircraft id

   sprintf(txt, "%d", id);
   str = txt;

   // Field

   str += ",";
   str += field.c_str();

   // Data line-all in meters, meters/second.
   const aaesim::open_source::WindStack &mat(weatherPrediction.east_west);
   for (int i = 1; i <= mat.GetMaxRow(); i++) {
      Units::Length alt = mat.GetAltitude(i);
      Units::KelvinTemperature temperature = weatherPrediction.GetForecastAtmosphere()->GetTemperature(alt);
      sprintf(txt, ",%lf", temperature.value() - 273.15);
      str += txt;
   }

   delete[] txt;

   return str;
}

void InternalObserver::dumpPredictedWind() {
   // Outputs predicted winds .csv file.

   string fileName = scenario_name + "-Predicted-Winds.csv";
   ofstream out;
   out.open(fileName.c_str());

   if (out.is_open()) {
      for (size_t ix = 0; ix < predWinds.size(); ix++) {
         out << predWinds[ix] << endl;
      }

      out.close();
   }

   // Clear predicted winds output.
   predWinds.clear();
}

void InternalObserver::addAchieveRcd(size_t aircraftId, double tm, double target_ttg_to_ach, double own_ttg_to_ach,
                                     double curr_distance, double reference_distance) {
   // Adds data record for achieve algorithms.
   //
   // aircraftId:aircraft id.
   // tm:time (seconds).
   // target_ttg_to_ach:target time to go to achieve (seconds).
   // own_ttg_to_ach:own time to go to achieve (seconds).
   // curr_distance:current distance (meters).
   // reference_distance:reference distance (meters).

   AchieveObserver achievercd(this->m_scenario_iter, aircraftId, tm, target_ttg_to_ach, own_ttg_to_ach, curr_distance,
                              reference_distance);
   m_aircraft_scenario_stats[aircraftId].m_achieve_list.push_back(achievercd);
}

void InternalObserver::dumpAchieveList() {
   // Writes output from achieve algorithms to time to go .csv file.

   string fileName = scenario_name + "-time-to-go.csv";
   ofstream out;

   bool needHdr = true;

   for (auto ix = m_aircraft_scenario_stats.begin(); ix != m_aircraft_scenario_stats.end(); ++ix) {
      vector<AchieveObserver> &achieve_list = ix->second.m_achieve_list;

      if (achieve_list.empty()) {
         continue;
      }  // Nada for this ac

      // Header
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

void InternalObserver::setNMOutput(bool NMflag) {
   // Sets flag to output NM data.
   //
   // NMFlag:NM output flag.
   //        true-output NM data.
   //        false-don't output NM data.

   this->outputNMFiles = NMflag;
}

bool InternalObserver::outputNM(void) {
   // Determines whether NM data being output or not.
   //
   // returns true if outputting NM data.
   //         false if not outputting NM data.

   return this->outputNMFiles;
}

void InternalObserver::SetRecordMaintainMetrics(bool new_value) { m_save_maintain_metrics = new_value; }

const bool InternalObserver::GetRecordMaintainMetrics() const { return m_save_maintain_metrics; }

CrossTrackObserver &InternalObserver::GetCrossEntry() { return m_cross_entry; }

InternalObserver::AircraftIterationStats::AircraftIterationStats() : finalGS(-1.0) {}
