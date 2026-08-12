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

#include <memory>
#include <vector>

#include "imalgs/IMAlgorithm.h"
#include "public/ASSAP.h"
#include "public/FlightDeckApplication.h"
#include "public/TangentPlaneSequence.h"

namespace interval_management {
namespace open_source {

using namespace mitre::oss::simcore;

class FIMAlgorithmAdapter final : public mitre::oss::simcore::FlightDeckApplication {
  public:
   FIMAlgorithmAdapter(std::shared_ptr<interval_management::open_source::IMAlgorithm> im_algorithm,
                       IMUtils::IMAlgorithmTypes algorithm_type);
   ~FIMAlgorithmAdapter() = default;
   void Initialize(mitre::oss::simcore::FlightDeckApplicationInitializer &initializer_visitor) override;
   mitre::oss::simcore::Guidance Update(const mitre::oss::simcore::SimulationTime &simtime,
                                        const mitre::oss::simcore::Guidance &prevguidance,
                                        const mitre::oss::simcore::DynamicsState &dynamicsstate,
                                        const mitre::oss::simcore::AircraftState &owntruthstate) override;
   bool IsActive() const override;
   std::shared_ptr<interval_management::open_source::IMAlgorithm> GetImAlgorithm() const;
   IMUtils::IMAlgorithmTypes GetImAlgorithmType() const;

   const interval_management::open_source::IMClearance &GetImClearance() const {
      return m_im_algorithm->GetClearance();
   }

  private:
   inline static log4cplus::Logger m_logger{log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("FIMAlgorithmAdapter"))};
   static void LogAircraftState(const mitre::oss::simcore::AircraftState &state);
   void UpdateTargetHistory(const mitre::oss::simcore::SimulationTime &simtime);
   interval_management::open_source::AircraftState ConvertAircraftState(
         const mitre::oss::simcore::AircraftState &state) const;
   std::shared_ptr<interval_management::open_source::IMAlgorithm> m_im_algorithm{};
   IMUtils::IMAlgorithmTypes m_im_algorithm_type{IMUtils::IMAlgorithmTypes::NONE};
   std::shared_ptr<const mitre::oss::simcore::ASSAP> m_assap{};
   std::vector<interval_management::open_source::AircraftState> m_target_history{};
   std::shared_ptr<TangentPlaneSequence> m_position_converter{};
};

inline std::shared_ptr<interval_management::open_source::IMAlgorithm>
      interval_management::open_source::FIMAlgorithmAdapter::GetImAlgorithm() const {
   return m_im_algorithm;
}

inline IMUtils::IMAlgorithmTypes FIMAlgorithmAdapter::GetImAlgorithmType() const { return m_im_algorithm_type; }

}  // namespace open_source
}  // namespace interval_management
