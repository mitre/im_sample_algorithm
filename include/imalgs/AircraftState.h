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

#include "public/Logging.h"
#include "public/EarthModel.h"
#include <scalar/Density.h>
#include <scalar/Frequency.h>
#include <scalar/Pressure.h>
#include <scalar/Speed.h>
#include <scalar/Temperature.h>
#include <scalar/UnsignedAngle.h>

namespace interval_management {
namespace open_source {
class AircraftState {
  public:
   AircraftState();

   ~AircraftState() = default;

   // FIXME rename method or replace callers with GetPsi
   const Units::UnsignedRadiansAngle GetHeadingCcwFromEastRadians() const;

   const Units::Speed GetGroundSpeed() const;

   AircraftState &Interpolate(const AircraftState &a, const AircraftState &b, const Units::SecondsTime time);

   AircraftState &Extrapolate(const AircraftState &in, const Units::SecondsTime &time);

   // FIXME Refactor to use a Builder pattern
   AircraftState &Create(const int &id, const Units::Time &time, const EarthModel::LocalPositionEnu &enu_position,
                         const Units::Speed &xd, const Units::Speed &yd, const Units::Speed &zd,
                         const Units::Angle &gamma, const Units::Speed &sensed_wind_east,
                         const Units::Speed &sensed_wind_north, const Units::Speed &sensed_wind_parallel,
                         const Units::Speed &sensed_wind_perpendicular, const Units::Temperature &sensed_temperature,
                         const Units::Angle &psi_enu);

   EarthModel::LocalPositionEnu GetPosition() const;

   Units::Length GetPositionX() const;

   Units::Length GetPositionY() const;

   Units::Length GetPositionZ() const;

   Units::Speed GetSpeedXd() const;

   Units::Speed GetSpeedYd() const;

   Units::Speed GetSpeedZd() const;

   Units::Speed GetTrueAirspeed() const;

   Units::SecondsTime GetTimeStamp() const;

   Units::SignedRadiansAngle GetPsi() const;

   Units::RadiansAngle GetGamma() const;

   Units::Temperature GetSensedTemperature() const;

   Units::Speed GetSensedWindParallelComponent() const;

   Units::Speed GetSensedWindPerpendicularComponent() const;

   Units::Speed GetSensedWindEastComponent() const;

   Units::Speed GetSensedWindNorthComponent() const;

   int GetId() const;

   // FIXME make these private and unitized
   double m_x, m_y, m_z;     // position (ft)
   double m_xd, m_yd, m_zd;  // speed (ft/s)
   double m_distance_to_go_meters;

  private:
   static log4cplus::Logger m_logger;
   int m_id;
   Units::Time m_time;
   Units::Speed m_sensed_wind_east_component, m_sensed_wind_north_component, m_sensed_wind_perpendicular_component,
         m_sensed_wind_parallel_component;
   Units::SignedRadiansAngle m_psi_enu;
   Units::RadiansAngle m_gamma;
   Units::Temperature m_sensed_temperature;
};

inline Units::Length AircraftState::GetPositionX() const { return Units::FeetLength(m_x); }

inline Units::Length AircraftState::GetPositionY() const { return Units::FeetLength(m_y); }

inline Units::Length AircraftState::GetPositionZ() const { return Units::FeetLength(m_z); }

inline Units::Speed AircraftState::GetSpeedXd() const { return Units::FeetPerSecondSpeed(m_xd); }

inline Units::Speed AircraftState::GetSpeedYd() const { return Units::FeetPerSecondSpeed(m_yd); }

inline Units::Speed AircraftState::GetSpeedZd() const { return Units::FeetPerSecondSpeed(m_zd); }

inline Units::SecondsTime AircraftState::GetTimeStamp() const { return m_time; }

inline int AircraftState::GetId() const { return m_id; }

inline Units::SignedRadiansAngle AircraftState::GetPsi() const { return m_psi_enu; }

inline Units::RadiansAngle AircraftState::GetGamma() const { return m_gamma; }

inline Units::Temperature AircraftState::GetSensedTemperature() const { return m_sensed_temperature; }

inline Units::Speed AircraftState::GetSensedWindParallelComponent() const { return m_sensed_wind_parallel_component; }

inline Units::Speed AircraftState::GetSensedWindPerpendicularComponent() const {
   return m_sensed_wind_perpendicular_component;
}

inline Units::Speed AircraftState::GetSensedWindEastComponent() const { return m_sensed_wind_east_component; }

inline Units::Speed AircraftState::GetSensedWindNorthComponent() const { return m_sensed_wind_north_component; }

inline EarthModel::LocalPositionEnu AircraftState::GetPosition() const {
   return EarthModel::LocalPositionEnu::Of(GetPositionX(), GetPositionY(), GetPositionZ());
}
}  // namespace open_source
}  // namespace interval_management