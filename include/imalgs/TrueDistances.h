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
#include <scalar/Time.h>
#include <scalar/Length.h>

namespace interval_management {
namespace open_source {
class TrueDistances {

  public:
   static const Units::Time END_OF_ROUTE_CROSSING_TIME_TOLERANCE;

   TrueDistances();

   TrueDistances(const TrueDistances &obj);

   virtual ~TrueDistances();

   void Add(const Units::Time time_in, const Units::Length true_distance_in);

   void Clear();

   const Units::Length ComputeTrueDtgToPtpAtTime(const Units::Time &time_in);

  private:
   void Copy(const TrueDistances &obj);

   std::vector<Units::Time> m_times;
   std::vector<Units::Length> m_true_distances;
};
}  // namespace open_source
}  // namespace interval_management