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

#include <log4cplus/logger.h>
#include <scalar/Length.h>

#include <string>

#include "public/AircraftIntent.h"

namespace interval_management {
namespace open_source {

class MergePointMetric final {
  public:
   MergePointMetric();

   ~MergePointMetric();

   // Determines and stores the merge point.
   void determineMergePoint(const AircraftIntent &IMIntent, const AircraftIntent &targIntent);

   // Updates IM and target position.
   void update(double imXNew, double imYNew, double targXNew, double targYNew);

   // Gets merge point (waypoint name).
   const std::string &getMergePoint();

   // Gets computed distance.
   Units::Length getDist();

   // Returns whether merge point is set or not.
   bool mergePointFound();

   bool willReportMetrics() const;
   int GetImAcId() const;
   int GetTargetAcId() const;

  private:
   static log4cplus::Logger logger;

   // Checks if newest IM position closer to waypoint than the stored IM position.
   bool newPointCloser(double x, double y);

   int m_im_ac_id{0};
   int m_target_ac_id{0};

   std::string mMergePointName{};
   Units::Length mMergePointX{Units::ZERO_LENGTH};
   Units::Length mMergePointY{Units::ZERO_LENGTH};

   double m_im_x_ft{0};
   double m_im_y_ft{0};

   Units::Length mIMDist{Units::infinity()};

   double m_targ_x_ft{0};
   double m_targ_y_ft{0};

   Units::Length mMergeDist{Units::infinity()};

   bool mReportMetrics{false};
};

}  // namespace open_source
}  // namespace interval_management
