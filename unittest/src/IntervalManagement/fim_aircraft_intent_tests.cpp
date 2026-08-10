// ****************************************************************************
// NOTICE
//
// (c) 2026 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include <gtest/gtest.h>

#include <vector>

#include "imalgs/FIMAircraftIntent.h"
#include "public/SingleTangentPlaneSequence.h"

namespace interval_management::open_source::test {

namespace {

std::vector<Waypoint> BuildDescentWaypoints() {
   return {Waypoint{"first", Units::DegreesAngle(35.0), Units::DegreesAngle(-77.0)},
           Waypoint{"last", Units::DegreesAngle(36.0), Units::DegreesAngle(-76.0)}};
}

}  // namespace

TEST(FIMAircraftIntentBuilder, InsertsWaypointAtRequestedIndex) {
   SingleTangentPlaneSequence::ClearStaticMembers();
   const auto intent = FIMAircraftIntent::Builder()
                         .LoadWaypoints({}, {}, BuildDescentWaypoints())
                         .InsertWaypointAtIndex(
                               Waypoint{"inserted", Units::DegreesAngle(35.5), Units::DegreesAngle(-76.5)}, 1)
                         .Build();

   EXPECT_EQ(3U, intent.GetNumberOfWaypoints());
   EXPECT_EQ(std::optional<std::string>{"inserted"}, intent.GetWaypointName(1));
}

TEST(FIMAircraftIntentBuilder, InsertsLocalPairAtRequestedIndex) {
   SingleTangentPlaneSequence::ClearStaticMembers();
   const auto intent = FIMAircraftIntent::Builder()
                         .LoadWaypoints({}, {}, BuildDescentWaypoints())
                         .InsertPairAtIndex("merge", Units::MetersLength(0.0), Units::MetersLength(0.0), 1)
                         .Build();

   EXPECT_EQ(3U, intent.GetNumberOfWaypoints());
   EXPECT_EQ(std::optional<std::string>{"merge"}, intent.GetWaypointName(1));
}

}  // namespace interval_management::open_source::test
