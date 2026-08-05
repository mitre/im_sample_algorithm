// ****************************************************************************
// NOTICE
//
// (c) 2026 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <memory>
#include <string>

#include "public/DefaultAircraftIntent.h"
#include "public/AircraftIntentUtils.h"
#include "public/ScenarioUtils.h"

namespace interval_management::open_source {

class FIMAircraftIntent final : public aaesim::open_source::AircraftIntent {
 public:
   class Builder final {
    public:
      Builder() = default;
      explicit Builder(const aaesim::open_source::DefaultAircraftIntent &intent) : intent_(intent) {}
      explicit Builder(const std::shared_ptr<const aaesim::open_source::AircraftIntent> &intent)
         : intent_(*aaesim::open_source::DefaultAircraftIntent::Builder(*intent).Build()) {}
      explicit Builder(const std::shared_ptr<aaesim::open_source::AircraftIntent> &intent)
         : intent_(*aaesim::open_source::DefaultAircraftIntent::Builder(*intent).Build()) {}
      explicit Builder(const aaesim::open_source::AircraftIntent &intent)
         : intent_(*aaesim::open_source::DefaultAircraftIntent::Builder(intent).Build()) {}

      Builder &SetId(int id) { id_ = id; return *this; }
      Builder &SetAircraftId(const std::string &aircraft_id) {
         id_ = aaesim::open_source::ScenarioUtils::GetUniqueIdForAircraftId(aircraft_id);
         return *this;
      }
      Builder &ClearWaypoints() { intent_.ClearWaypoints(); return *this; }
      Builder &SetPlannedCruiseAltitude(Units::Length altitude) {
         intent_.SetPlannedCruiseAltitude(altitude);
         return *this;
      }
      Builder &InsertWaypointAtIndex(const Waypoint &waypoint, int index) {
         intent_.InsertWaypointAtIndex(waypoint, index);
         return *this;
      }
      Builder &InsertPairAtIndex(const std::string &name, Units::Length x, Units::Length y, int index) {
         intent_.InsertPairAtIndex(name, x, y, index);
         return *this;
      }
      Builder &UpdateWaypoint(const Waypoint &waypoint) { intent_.UpdateWaypoint(waypoint); return *this; }
      Builder &LoadWaypoints(const std::vector<Waypoint> &ascent, const std::vector<Waypoint> &cruise,
                             const std::vector<Waypoint> &descent) {
         intent_.LoadWaypoints(ascent, cruise, descent);
         return *this;
      }
      FIMAircraftIntent Build() const { return FIMAircraftIntent(intent_, id_); }

    private:
      aaesim::open_source::DefaultAircraftIntent intent_{};
      int id_{aaesim::open_source::ScenarioUtils::AIRCRAFT_ID_NOT_IN_MAP};
   };

   FIMAircraftIntent() = default;
   int GetId() const { return id_; }

   static FIMAircraftIntent CopyAndTrimAfterNamedWaypoint(const FIMAircraftIntent &intent,
                                                           const std::string &waypoint_name) {
      return Builder(aaesim::open_source::AircraftIntentUtils::CopyAndTrimAfterNamedWaypoint(intent, waypoint_name))
            .SetId(intent.id_).Build();
   }

   std::optional<Waypoint> GetWaypoint(unsigned int i) const override { return intent_.GetWaypoint(i); }
   const std::vector<Waypoint> &GetWaypoints() const override { return intent_.GetWaypoints(); }
   std::optional<std::string> GetWaypointName(unsigned int i) const override { return intent_.GetWaypointName(i); }
   Units::MetersLength GetPlannedCruiseAltitude() const override { return intent_.GetPlannedCruiseAltitude(); }
   const RouteData &GetRouteData() const override { return intent_.GetRouteData(); }
   std::optional<unsigned int> GetWaypointIndexByName(const std::string &name) const override {
      return intent_.GetWaypointIndexByName(name);
   }
   std::pair<int, int> FindCommonWaypoint(const AircraftIntent &intent) const override {
      return intent_.FindCommonWaypoint(intent);
   }
   unsigned int GetNumberOfWaypoints() const override { return intent_.GetNumberOfWaypoints(); }
   const std::vector<Waypoint> &GetAscentWaypoints() const override { return intent_.GetAscentWaypoints(); }
   const std::vector<Waypoint> &GetCruiseWaypoints() const override { return intent_.GetCruiseWaypoints(); }
   const std::vector<Waypoint> &GetDescentWaypoints() const override { return intent_.GetDescentWaypoints(); }
   double GetPlannedCruiseMach() const override { return intent_.GetPlannedCruiseMach(); }
   bool ContainsWaypointName(const std::string &name) const override { return intent_.ContainsWaypointName(name); }

   bool IsLoaded() const { return intent_.IsLoaded(); }
   Units::MetersLength GetWaypointX(unsigned int index) const { return intent_.GetWaypointX(index); }
   Units::MetersLength GetWaypointY(unsigned int index) const { return intent_.GetWaypointY(index); }

 private:
   FIMAircraftIntent(const aaesim::open_source::DefaultAircraftIntent &intent, int id) : intent_(intent), id_(id) {}

   aaesim::open_source::DefaultAircraftIntent intent_{};
   int id_{aaesim::open_source::ScenarioUtils::AIRCRAFT_ID_NOT_IN_MAP};
};

}  // namespace interval_management::open_source
