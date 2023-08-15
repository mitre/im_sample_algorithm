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

#include "public/FlightDeckApplication.h"
#include "imalgs/FIMAlgorithmAdapter.h"
#include "imalgs/IMKinematicAchieve.h"
#include "imalgs/IMTimeBasedAchieveMutableASG.h"
#include "imalgs/IMTimeBasedAchieve.h"
#include "imalgs/IMDistBasedAchieve.h"
#include "public/PassThroughAssap.h"
namespace interval_management {
namespace open_source {
class FIMAlgorithmInitializer : public aaesim::open_source::FlightDeckApplicationInitializer {
  public:
   FIMAlgorithmInitializer() = default;

   void Initialize(interval_management::open_source::FIMAlgorithmAdapter *algorithm);

   class Builder {
     private:
      aaesim::open_source::OwnshipPerformanceParameters m_performance_parameters;
      aaesim::open_source::OwnshipFmsPredictionParameters m_prediction_parameters;
      std::shared_ptr<const aaesim::open_source::ASSAP> m_surveillance_processor;

     public:
      Builder() : m_performance_parameters(), m_prediction_parameters() {
         m_surveillance_processor = std::make_shared<aaesim::open_source::PassThroughAssap>();
      };
      ~Builder() = default;
      const interval_management::open_source::FIMAlgorithmInitializer Build() const;
      Builder *AddOwnshipPerformanceParameters(
            aaesim::open_source::OwnshipPerformanceParameters performance_parameters);
      Builder *AddOwnshipFmsPredictionParameters(
            aaesim::open_source::OwnshipFmsPredictionParameters prediction_parameters);
      Builder *AddSurveillanceProcessor(std::shared_ptr<const aaesim::open_source::ASSAP> processor);

      aaesim::open_source::OwnshipPerformanceParameters GetPerformanceParameters() const {
         return m_performance_parameters;
      };
      aaesim::open_source::OwnshipFmsPredictionParameters GetFmsPredictionParameters() const {
         return m_prediction_parameters;
      };
      std::shared_ptr<const aaesim::open_source::ASSAP> GetSurveillanceProcessor() const {
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