// ****************************************************************************
// NOTICE
//
// This is the copyright work of The MITRE Corporation, and was produced
// for the U. S. Government under Contract Number DTFAWA-10-C-00080, and
// is subject to Federal Aviation Administration Acquisition Management
// System Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV
// (Oct. 1996).  No other use other than that granted to the U. S.
// Government, or to those acting on behalf of the U. S. Government,
// under that Clause is authorized without the express written
// permission of The MITRE Corporation. For further information, please
// contact The MITRE Corporation, Contracts Office, 7515 Colshire Drive,
// McLean, VA  22102-7539, (703) 983-6000. 
//
// Copyright 2020 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include "public/AircraftState.h"
#include "public/ADSBSVReport.h"
#include "public/AircraftIntent.h"
#include "imalgs/IMAlgorithm.h"
#include "public/LoggingLoadable.h"
#include "public/Guidance.h"
#include "aaesim/KineticTrajectoryPredictor.h"
#include "public/ThreeDOFDynamics.h"
#include "aaesim/ASSAP.h"
#include "utility/Logging.h"
#include <string>
#include <vector>
#include <map>
#include "aaesim/ADSBReceiver.h"
#include "aaesim/SimpleAircraft.h"
#include "imalgs/IMClearance.h"


class AirborneApplication : public LoggingLoadable
{
public:
   AirborneApplication(void);

   ~AirborneApplication(void);

   AirborneApplication(const AirborneApplication &in);

   AirborneApplication &operator=(const AirborneApplication &in);

   /**
    *
    * @param target_aircraft
    * @param ownship_intent
    * @param target_intent_from_clearance
    * @param clearance
    * @param ownship_numeric_id
    * @param do_wind_blending
    * @param kinetic_trajectory_prediction
    * @param receiver surveillance ADS-B receiver
    * @param weather_prediction This is a non-const reference. It can be updated by the loaded application.
    */
   void Initialize(const SimpleAircraft &target_aircraft,
                   const AircraftIntent &ownship_intent,
                   const AircraftIntent &target_intent_from_clearance,
                   const IMClearance &clearance,
                   int ownship_numeric_id,
                   bool do_wind_blending,
                   const KineticTrajectoryPredictor &kinetic_trajectory_prediction,
                   const Sensor::ADSB::ADSBReceiver &receiver,
                   WeatherPrediction &weather_prediction);

   /**
    * Compute guidance based on particular airborne application from runfile.
    *
    * @param simtime
    * @param dynamics
    * @param ownship_state
    * @param current_guidance
    * @return returns updated guidance
    */
   Guidance Update(const SimulationTime &simtime,
                   const ThreeDOFDynamics &dynamics,
                   const AircraftState &ownship_state,
                   const Guidance &current_guidance);

   bool load(DecodedStream *input);

   /**
    * add ADS B entries for the history report
    *
    * @param recent_reports
    */
   void AddSurveillanceReports(const vector<Sensor::ADSB::ADSBSVReport> &recent_reports);

   bool IsLoaded() const;

   const IMUtils::IMAlgorithmTypes GetApplicationType() const;

   int GetNumericTargetId() const;

   const bool IsPlaceboModeEnabled() const;

   std::shared_ptr<IMAlgorithm> GetImAlgorithm() const;

private:
   static log4cplus::Logger m_logger;

   void Copy(const AirborneApplication &in);

   std::shared_ptr<IMAlgorithm> m_imalgorithm;

   IMUtils::IMAlgorithmTypes m_algorithm_type;

   int m_target_numeric_id;

   const Sensor::ADSB::ADSBReceiver *m_aircraft_receiver;

   ASSAP m_assap;

   std::vector<AircraftState> m_target_adsb_history; // FIXME this stores the converted ADS_B history data for the target aircraft, not AircraftState. Use vector<Sensor::ADSB::ADSBSVReport> instead.

   bool m_im_placebo_mode_enabled;
   bool m_is_loaded;
};

inline IMUtils::IMAlgorithmTypes const AirborneApplication::GetApplicationType() const {
   return this->m_algorithm_type;
}

inline int AirborneApplication::GetNumericTargetId(void) const {
   return this->m_target_numeric_id;
}

inline bool AirborneApplication::IsLoaded() const {
   return this->m_is_loaded;
}

inline const bool AirborneApplication::IsPlaceboModeEnabled() const {
   return m_im_placebo_mode_enabled;
}

inline std::shared_ptr<IMAlgorithm> AirborneApplication::GetImAlgorithm() const {
   return m_imalgorithm;
}
