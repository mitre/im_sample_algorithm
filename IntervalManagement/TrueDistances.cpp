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

#include "imalgs/TrueDistances.h"

using namespace interval_management::open_source;

const Units::Time TrueDistances::END_OF_ROUTE_CROSSING_TIME_TOLERANCE = Units::SecondsTime(1.1);

TrueDistances::TrueDistances() : m_times(), m_true_distances() {}

TrueDistances::TrueDistances(const TrueDistances &obj) { Copy(obj); }

TrueDistances::~TrueDistances() {}

void TrueDistances::Copy(const TrueDistances &obj) {
   m_times = obj.m_times;
   m_true_distances = obj.m_true_distances;
}

void TrueDistances::Add(const Units::Time time_in, const Units::Length true_distance_in) {
   // Adds a new true distance.
   //
   // time:time (s).
   // trueDist:true distance (m).

   m_times.push_back(time_in);
   m_true_distances.push_back(true_distance_in);
}

const Units::Length TrueDistances::ComputeTrueDtgToPtpAtTime(const Units::Time &time_in) {
   // Computes true distances for a time from
   // stored true distances.
   //
   // time:time (s).
   //
   // returns computed true distance in meters or
   //         negative number if data not available
   //         for time.

   // Find time in vector.

   int ix = -1;

   if (!m_times.empty()) {
      if ((time_in > m_times.back()) && Units::abs(time_in - m_times.back()) < END_OF_ROUTE_CROSSING_TIME_TOLERANCE) {
         ix = static_cast<int>(m_times.size() - 1);
      } else {
         for (int i = 1; (i < m_times.size()) && (ix == -1); ++i) {
            if ((time_in >= m_times[i - 1]) && (time_in <= m_times[i])) {
               ix = i;
            }
         }
      }
   }

   if (ix >= 0) {
      // We have found a bounding pair.
      if (time_in == m_times[ix - 1]) {
         return m_true_distances[ix - 1];
      } else if (time_in == m_times[ix]) {
         return m_true_distances[ix];
      } else {
         // Interpolate
         double ratio = (time_in - m_times[ix - 1]) / (m_times[ix] - m_times[ix - 1]);
         return ((1.0 - ratio) * m_true_distances[ix - 1]) + (ratio * m_true_distances[ix]);
      }
   }

   return Units::NegInfinity();
}

void TrueDistances::Clear() {
   m_times.clear();
   m_true_distances.clear();
}