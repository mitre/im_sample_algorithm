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

#include "imalgs/InternalObserverScenarioWriter.h"
#include "imalgs/InternalObserver.h"

void interval_management::open_source::InternalObserverScenarioWriter::IterationBegin(const int &iteration_number) {
   InternalObserver::getInstance()->initializeIteration();
   InternalObserver::getInstance()->SetScenarioIter(iteration_number);
}

void interval_management::open_source::InternalObserverScenarioWriter::IterationEnd(const int &iteration_number) {
   InternalObserver::getInstance()->outputMaintainMetrics();
   InternalObserver::getInstance()->outputFinalGS();
   InternalObserver::getInstance()->outputMergePointMetric();
   InternalObserver::getInstance()->outputClosestPointMetric();
   InternalObserver::getInstance()->process_NM_aircraft();
}

void interval_management::open_source::InternalObserverScenarioWriter::ScenarioBegin(const std::string &scenario_name) {
   InternalObserver::getInstance()->set_scenario_name(scenario_name);
   InternalObserver::getInstance()->setNMOutput(true);
}

void interval_management::open_source::InternalObserverScenarioWriter::ScenarioEnd(const std::string &scenario_name) {
   InternalObserver::getInstance()->process();
   InternalObserver::clearInstance();
}
