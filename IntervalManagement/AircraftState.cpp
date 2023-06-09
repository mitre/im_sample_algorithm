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
// 2022 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "imalgs/AircraftState.h"
#include "math/CustomMath.h"
#include "utility/CustomUnits.h"
#include "imalgs/IMUtils.h"

using namespace interval_management::open_source;

log4cplus::Logger interval_management::open_source::AircraftState::m_logger =
      log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("AircraftState"));

AircraftState::AircraftState()
   : m_x(0),
     m_y(0),
     m_z(0),
     m_xd(0),
     m_yd(0),
     m_zd(0),
     m_distance_to_go_meters(-INFINITY),
     m_id(IMUtils::UNINITIALIZED_AIRCRAFT_ID),
     m_time(Units::ZERO_TIME),
     m_sensed_wind_east_component(Units::negInfinity()),
     m_sensed_wind_north_component(Units::negInfinity()),
     m_sensed_wind_perpendicular_component(Units::negInfinity()),
     m_sensed_wind_parallel_component(Units::negInfinity()),
     m_psi_enu(Units::ZERO_ANGLE),
     m_gamma(Units::ZERO_ANGLE),
     m_sensed_temperature(Units::negInfinity()) {}

const Units::UnsignedRadiansAngle AircraftState::GetHeadingCcwFromEastRadians() const {
   double result = atan3(m_yd, m_xd);
   return Units::UnsignedRadiansAngle(result);
}

const Units::Speed AircraftState::GetGroundSpeed() const {
   return Units::FeetPerSecondSpeed(sqrt(pow(m_xd, 2) + pow(m_yd, 2)));
}

AircraftState &AircraftState::Interpolate(const AircraftState &a, const AircraftState &b,
                                          const Units::SecondsTime time) {

   const double dt = Units::SecondsTime(b.GetTimeStamp() - a.GetTimeStamp()).value();
   double weight_a, weight_b;
   if (a.GetTimeStamp() == b.GetTimeStamp()) {
      // avoid divide by zero
      weight_a = 1;
      weight_b = 0;
      LOG4CPLUS_ERROR(m_logger, "Attempt to interpolate between two states for the same time.");
   } else {
      weight_a = (b.GetTimeStamp().value() - time.value()) / dt;
      weight_b = 1 - weight_a;
   }
   m_id = a.m_id;
   m_time = time;
   m_x = a.m_x * weight_a + b.m_x * weight_b;
   m_y = a.m_y * weight_a + b.m_y * weight_b;
   m_z = a.m_z * weight_a + b.m_z * weight_b;
   m_xd = a.m_xd * weight_a + b.m_xd * weight_b;
   m_yd = a.m_yd * weight_a + b.m_yd * weight_b;
   m_zd = a.m_zd * weight_a + b.m_zd * weight_b;

   return *this;
}

AircraftState &AircraftState::Extrapolate(const AircraftState &in, const Units::SecondsTime &time) {
   const double dt = time.value() - in.GetTimeStamp().value();
   m_time = time;
   m_id = in.m_id;
   m_x = in.m_x + in.m_xd * dt;
   m_y = in.m_y + in.m_yd * dt;
   m_z = in.m_z + in.m_zd * dt;
   m_xd = in.m_xd;
   m_yd = in.m_yd;
   m_zd = in.m_zd;
   return *this;
}

Units::Speed AircraftState::GetTrueAirspeed() const {
   Units::MetersPerSecondSpeed tas_x, tas_y;
   tas_x = Units::FeetPerSecondSpeed(m_xd) - m_sensed_wind_east_component;
   tas_y = Units::FeetPerSecondSpeed(m_yd) - m_sensed_wind_north_component;
   Units::MetersPerSecondSpeed tas = Units::sqrt(Units::sqr(tas_x) + Units::sqr(tas_y));
   return tas;
}

AircraftState &AircraftState::Create(const int &id, const Units::Time &time,
                                     const EarthModel::LocalPositionEnu &enu_position, const Units::Speed &xd,
                                     const Units::Speed &yd, const Units::Speed &zd, const Units::Angle &gamma,
                                     const Units::Speed &sensed_wind_east, const Units::Speed &sensed_wind_north,
                                     const Units::Speed &sensed_wind_parallel,
                                     const Units::Speed &sensed_wind_perpendicular,
                                     const Units::Temperature &sensed_temperature, const Units::Angle &psi_enu) {

   this->m_id = id;
   this->m_time = time;
   this->m_x = Units::FeetLength(enu_position.x).value();
   this->m_y = Units::FeetLength(enu_position.y).value();
   this->m_z = Units::FeetLength(enu_position.z).value();
   this->m_xd = Units::FeetPerSecondSpeed(xd).value();
   this->m_yd = Units::FeetPerSecondSpeed(yd).value();
   this->m_zd = Units::FeetPerSecondSpeed(zd).value();
   this->m_gamma = gamma;
   this->m_sensed_wind_east_component = sensed_wind_east;
   this->m_sensed_wind_north_component = sensed_wind_north;
   this->m_sensed_wind_parallel_component = sensed_wind_parallel;
   this->m_sensed_wind_perpendicular_component = sensed_wind_perpendicular;
   this->m_sensed_temperature = sensed_temperature;
   this->m_psi_enu = psi_enu;
   return *this;
}