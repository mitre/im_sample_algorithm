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

#include "imalgs/NMObserverEntry.h"

using namespace interval_management::open_source;

NMObserverEntry::NMObserverEntry(void) {
   predictedDistance = 0.0;
   trueDistance = 0.0;
   time = 0.0;
   acIAS = 0.0;
   acGS = 0.0;
   targetGS = 0.0;
   minIAS = 0.0;
   maxIAS = 0.0;
   minTAS = 0.0;
   maxTAS = 0.0;
}

NMObserverEntry::~NMObserverEntry(void) {}
