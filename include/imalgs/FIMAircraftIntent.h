// ****************************************************************************
// NOTICE
//
// (c) 2026 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "public/DefaultAircraftIntent.h"
#include "public/AircraftIntentUtils.h"
#include "public/ScenarioUtils.h"
#include "public/SingleTangentPlaneSequence.h"
#include "utility/BoundedValue.h"

namespace interval_management::open_source {

using namespace mitre::oss::simcore;

class FIMAircraftIntent final : public mitre::oss::simcore::AircraftIntent {
 public:
   class Builder final {
    public:
      Builder() = default;
      explicit Builder(const mitre::oss::simcore::DefaultAircraftIntent &intent) : intent_(intent) {}
      explicit Builder(const std::shared_ptr<const mitre::oss::simcore::AircraftIntent> &intent)
         : intent_(*mitre::oss::simcore::DefaultAircraftIntent::Builder().CopyFrom(*intent).Build()) {}
      explicit Builder(const std::shared_ptr<mitre::oss::simcore::AircraftIntent> &intent)
         : intent_(*mitre::oss::simcore::DefaultAircraftIntent::Builder().CopyFrom(*intent).Build()) {}
      explicit Builder(const mitre::oss::simcore::AircraftIntent &intent)
         : intent_(*mitre::oss::simcore::DefaultAircraftIntent::Builder().CopyFrom(intent).Build()) {}

      Builder &SetId(int id) { id_ = id; return *this; }
      Builder &SetAircraftId(const std::string &aircraft_id) {
         id_ = mitre::oss::simcore::ScenarioUtils::GetUniqueIdForAircraftId(aircraft_id);
         return *this;
      }
      Builder &SetPlannedCruiseAltitude(Units::Length altitude) {
         intent_ = *mitre::oss::simcore::DefaultAircraftIntent::Builder().CopyFrom(intent_)
                            .SetPlannedCruiseAltitude(altitude)
                            .Build();
         return *this;
      }
      Builder &SetPlannedCruiseMach(BoundedValue<double, 0, 1> mach) {
         intent_ = *mitre::oss::simcore::DefaultAircraftIntent::Builder().CopyFrom(intent_)
                            .SetPlannedCruiseMach(mach)
                            .Build();
         return *this;
      }
      Builder &InsertWaypointAtIndex(const Waypoint &waypoint, int index) {
         if (index < 0 || static_cast<unsigned int>(index) > intent_.GetNumberOfWaypoints()) {
            throw std::out_of_range("Waypoint insertion index is outside the intent route");
         }

         auto ascent = intent_.GetAscentWaypoints();
         auto cruise = intent_.GetCruiseWaypoints();
         auto descent = intent_.GetDescentWaypoints();
         const auto insertion_index = static_cast<std::size_t>(index);
         if (insertion_index < ascent.size()) {
            ascent.insert(ascent.begin() + index, waypoint);
         } else if (insertion_index < ascent.size() + cruise.size()) {
            cruise.insert(cruise.begin() + index - static_cast<int>(ascent.size()), waypoint);
         } else {
            descent.insert(descent.begin() + index - static_cast<int>(ascent.size() + cruise.size()), waypoint);
         }
         RebuildIntent(ascent, cruise, descent);
         return *this;
      }
      Builder &InsertPairAtIndex(const std::string &name, Units::Length x, Units::Length y, int index) {
         const auto &waypoints = intent_.GetWaypoints();
         if (waypoints.empty() || index < 0 || static_cast<unsigned int>(index) > waypoints.size()) {
            throw std::out_of_range("Pair insertion index is outside the intent route");
         }

         SingleTangentPlaneSequence position_converter(waypoints);
         EarthModel::LocalPositionEnu local_position{};
         local_position.x = x;
         local_position.y = y;
         local_position.z = Units::ZERO_LENGTH;
         EarthModel::GeodeticPosition geodetic_position{};
         position_converter.ConvertLocalToGeodetic(local_position, geodetic_position);

         Waypoint waypoint{};
         waypoint.SetRfTurnArcRadius(Units::ZERO_LENGTH);
         waypoint.SetWaypointLatLon(geodetic_position.latitude, geodetic_position.longitude);
         waypoint.SetName(name);
         const auto &previous_waypoint = waypoints[index == 0 ? 0 : index - 1];
         const auto &next_waypoint = waypoints[index == static_cast<int>(waypoints.size()) ? waypoints.size() - 1 : index];
         waypoint.SetAltitudeConstraintHigh(previous_waypoint.GetAltitudeConstraintHigh());
         waypoint.SetSpeedConstraintHigh(previous_waypoint.GetSpeedConstraintHigh());
         waypoint.SetAltitudeConstraintLow(next_waypoint.GetAltitudeConstraintLow());
         waypoint.SetSpeedConstraintLow(next_waypoint.GetSpeedConstraintLow());
         return InsertWaypointAtIndex(waypoint, index);
      }
      Builder &UpdateWaypoint(const Waypoint &waypoint) {
         auto updated_waypoints = intent_.GetWaypoints();
         const auto matching_waypoints = std::count_if(
               updated_waypoints.cbegin(), updated_waypoints.cend(),
               [&waypoint](const Waypoint &existing_waypoint) { return existing_waypoint.GetName() == waypoint.GetName(); });
         if (matching_waypoints != 1) {
            throw std::runtime_error("Expected exactly one waypoint with the supplied name.");
         }
         std::replace_if(updated_waypoints.begin(), updated_waypoints.end(),
                         [&waypoint](const Waypoint &existing_waypoint) {
                            return existing_waypoint.GetName() == waypoint.GetName();
                         },
                         waypoint);
         intent_ = *mitre::oss::simcore::DefaultAircraftIntent::Builder()
                            .SetDescentWaypoints(updated_waypoints)
                            .SetPlannedCruiseMach(BoundedValue<double, 0, 1>(intent_.GetPlannedCruiseMach()))
                            .SetPlannedCruiseAltitude(intent_.GetPlannedCruiseAltitude())
                            .Build();
         return *this;
      }
      Builder &LoadWaypoints(const std::vector<Waypoint> &ascent, const std::vector<Waypoint> &cruise,
                             const std::vector<Waypoint> &descent) {
         intent_ = *mitre::oss::simcore::DefaultAircraftIntent::Builder()
                            .SetAscentWaypoints(ascent)
                            .SetCruiseWaypoints(cruise)
                            .SetDescentWaypoints(descent)
                            .Build();
         return *this;
      }
      FIMAircraftIntent Build() const { return FIMAircraftIntent(intent_, id_); }

    private:
      void RebuildIntent(const std::vector<Waypoint> &ascent, const std::vector<Waypoint> &cruise,
                         const std::vector<Waypoint> &descent) {
         intent_ = *mitre::oss::simcore::DefaultAircraftIntent::Builder()
                            .SetAscentWaypoints(ascent)
                            .SetCruiseWaypoints(cruise)
                            .SetDescentWaypoints(descent)
                            .SetPlannedCruiseMach(BoundedValue<double, 0, 1>(intent_.GetPlannedCruiseMach()))
                            .SetPlannedCruiseAltitude(intent_.GetPlannedCruiseAltitude())
                            .Build();
      }

      mitre::oss::simcore::DefaultAircraftIntent intent_{};
      int id_{mitre::oss::simcore::ScenarioUtils::AIRCRAFT_ID_NOT_IN_MAP};
   };

   FIMAircraftIntent() = default;
   int GetId() const { return id_; }

   static FIMAircraftIntent CopyAndTrimAfterNamedWaypoint(const FIMAircraftIntent &intent,
                                                           const std::string &waypoint_name) {
      return Builder(mitre::oss::simcore::AircraftIntentUtils::CopyAndTrimAfterNamedWaypoint(intent, waypoint_name))
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

   bool IsLoaded() const { return GetNumberOfWaypoints() > 0; }
   Units::MetersLength GetWaypointX(unsigned int index) const { return intent_.GetWaypointX(index); }
   Units::MetersLength GetWaypointY(unsigned int index) const { return intent_.GetWaypointY(index); }

 private:
   FIMAircraftIntent(const mitre::oss::simcore::DefaultAircraftIntent &intent, int id) : intent_(intent), id_(id) {}

   mitre::oss::simcore::DefaultAircraftIntent intent_{};
   int id_{mitre::oss::simcore::ScenarioUtils::AIRCRAFT_ID_NOT_IN_MAP};
};

}  // namespace interval_management::open_source
