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

#include "imalgs/MaintainMetric.h"
#include <math.h>

using namespace interval_management::open_source;

MaintainMetric::MaintainMetric(void) {
   achieveByTime = -1.0;
   totalMaintainTime = 0.0;
   numCyclesOutsideThreshold = 0;
   m_output_enabled = false;
}

void MaintainMetric::AddSpacingErrorSec(double err) {

   // Adds data to be added for each pass through an IM::update method.
   // Also increments number of cycles if outside of threshold.
   //
   // err:input spacing error.

   spacingError.Insert(err);

   // TODO:Need to include time step in this if.

   if (fabs(err) > CYCLE_THRESHOLD) {
      numCyclesOutsideThreshold++;
   }
}

void MaintainMetric::SetTimeAtAbp(double aTime) {

   // Sets time aircraft went by achieve by point.
   //
   // aTime:achieve by time.

   achieveByTime = aTime;
}

void MaintainMetric::ComputeTotalMaintainTime(double cTime) {

   // Computes total maintain time subtracting the achieveByTime
   // from the current time.
   //
   // cTime:current time.

   totalMaintainTime = cTime - achieveByTime;
}

bool MaintainMetric::TimeAtAbpRecorded() {

   // Boolean to determine if achieveBy set.
   //
   // return:true if achieve by has valid time.
   //        false if achieve by does not have valid time.

   return (achieveByTime >= 0.0);
}

double MaintainMetric::getMeanErr() {

   // Gets mean spacing error.
   //
   // returns mean error.

   return spacingError.GetMean();
}

double MaintainMetric::getStdErr() {

   // Gets standard deviation of spacing error.
   //
   // returns standard deviation of error.

   return spacingError.ComputeStandardDeviation();
}

double MaintainMetric::getBound95() {

   // Gets 95th bound of spacing error.
   //
   // returns 95th bound of spacing error.

   return spacingError.Get95thBounds();
}

double MaintainMetric::getTotMaintain() {

   // Gets total maintain time.
   //
   // returns total maintain time.

   return totalMaintainTime;
}

int MaintainMetric::getNumCycles() {

   // Gets number of cycles with spacing errors > cycle threshold
   //
   // returns number of cycles.

   return numCyclesOutsideThreshold;
}

bool MaintainMetric::hasSamples() {

   // Determines whether there are any samples collected.
   //
   // returns true if there are samples
   //		   else false.

   return (spacingError.GetNumberOfSamples() > 0);
}

bool MaintainMetric::IsOutputEnabled() const { return m_output_enabled; }

void MaintainMetric::SetOutputEnabled(bool output_enabled) { m_output_enabled = output_enabled; }
