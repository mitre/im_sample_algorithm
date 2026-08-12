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

#include "imalgs/FIMAlgorithmAdapter.h"
#include "imalgs/IMDistBasedAchieve.h"
#include "imalgs/IMKinematicAchieve.h"
#include "imalgs/IMTimeBasedAchieve.h"
#include "imalgs/IMTimeBasedAchieveMutableASG.h"
#include "public/FlightDeckApplication.h"
#include "public/PassThroughAssap.h"

namespace interval_management {
namespace open_source {

using namespace mitre::oss::simcore;

class FIMAlgorithmInitializer final : public mitre::oss::simcore::FlightDeckApplicationInitializer {
  public:
   FIMAlgorithmInitializer() = default;

   void Initialize(interval_management::open_source::FIMAlgorithmAdapter *algorithm);

   class Builder {
     private:
      mitre::oss::simcore::OwnshipPerformanceParameters m_performance_parameters;
      mitre::oss::simcore::OwnshipFmsPredictionParameters m_prediction_parameters;
      std::shared_ptr<const mitre::oss::simcore::ASSAP> m_surveillance_processor;

     public:
      Builder()
         : m_performance_parameters(),
           m_prediction_parameters(),
           m_surveillance_processor(std::make_shared<mitre::oss::simcore::PassThroughAssap>()) {};
      ~Builder() = default;
      const interval_management::open_source::FIMAlgorithmInitializer Build() const;
      Builder *AddOwnshipPerformanceParameters(
            const mitre::oss::simcore::OwnshipPerformanceParameters &performance_parameters);
      Builder *AddOwnshipFmsPredictionParameters(
            const mitre::oss::simcore::OwnshipFmsPredictionParameters &prediction_parameters);
      Builder *AddSurveillanceProcessor(std::shared_ptr<const mitre::oss::simcore::ASSAP> processor);

      const mitre::oss::simcore::OwnshipPerformanceParameters &GetPerformanceParameters() const {
         return m_performance_parameters;
      };
      const mitre::oss::simcore::OwnshipFmsPredictionParameters &GetFmsPredictionParameters() const {
         return m_prediction_parameters;
      };
      std::shared_ptr<const mitre::oss::simcore::ASSAP> GetSurveillanceProcessor() const {
         return m_surveillance_processor;
      };
   };

  private:
   FIMAlgorithmInitializer(const FIMAlgorithmInitializer::Builder *builder);
   void Initialize(interval_management::open_source::IMKinematicAchieve *kinematic_algorithm);
   void Initialize(interval_management::open_source::IMTimeBasedAchieveMutableASG *test_vector_algorithm);
   void Initialize(interval_management::open_source::IMTimeBasedAchieve *time_achieve_algorithm);
   void Initialize(interval_management::open_source::IMDistBasedAchieve *dist_achieve_algorithm);
   interval_management::open_source::IMAlgorithm::OwnshipPredictionParameters BuildOwnshipPredictionParameters() const;
};

}  // namespace open_source
}  // namespace interval_management
