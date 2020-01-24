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

#include "imalgs/IMAlgorithmLoader.h"
#include "imalgs/IMNone.h"
#include "imalgs/IMPrecalcTargetAchieve.h"
#include "imalgs/IMTimeBasedAchieveMutableASG.h"

using namespace std;

log4cplus::Logger IMAlgorithmLoader::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("IMAlgorithmLoader"));

IMAlgorithmLoader::IMAlgorithmLoader(void) {
   pair<string, IMUtils::IMAlgorithmTypes> newPair =
         pair<string, IMUtils::IMAlgorithmTypes>("precalcAchieve", IMUtils::IMAlgorithmTypes::KINETICACHIEVE);
   m_algorithm_mapping.insert(newPair);

   newPair = pair<string, IMUtils::IMAlgorithmTypes>("precalcTargetAchieve",
                                                     IMUtils::IMAlgorithmTypes::KINETICTARGETACHIEVE);
   m_algorithm_mapping.insert(newPair);

   newPair = pair<string, IMUtils::IMAlgorithmTypes>("timeBasedAchieve", IMUtils::IMAlgorithmTypes::TIMEBASEDACHIEVE);
   m_algorithm_mapping.insert(newPair);

   newPair = pair<string, IMUtils::IMAlgorithmTypes>("distBasedAchieve",
                                                     IMUtils::IMAlgorithmTypes::DISTANCEBASEDACHIEVE);
   m_algorithm_mapping.insert(newPair);

   newPair = pair<string, IMUtils::IMAlgorithmTypes>("none", IMUtils::IMAlgorithmTypes::NONE);
   m_algorithm_mapping.insert(newPair);

   newPair = pair<string, IMUtils::IMAlgorithmTypes>("timeBasedAchieveMutableASG",
                                                     IMUtils::IMAlgorithmTypes::TIMEBASEDACHIEVEMUTABLEASG);
   m_algorithm_mapping.insert(newPair);

   newPair = pair<string, IMUtils::IMAlgorithmTypes>("rta", IMUtils::IMAlgorithmTypes::RTA);
   m_algorithm_mapping.insert(newPair);

   newPair = pair<string, IMUtils::IMAlgorithmTypes>("testspeedcontrol", IMUtils::IMAlgorithmTypes::TESTSPEEDCONTROL);
   m_algorithm_mapping.insert(newPair);

}

IMAlgorithmLoader::~IMAlgorithmLoader(void) {
}

shared_ptr<IMAlgorithm> IMAlgorithmLoader::GetImAlgorithmSharedPointer(IMUtils::IMAlgorithmTypes im_algorithm_type) {
   // Gets IM algorithm shared_ptr based on applicationtype.

   // Gets an IM algorithm object shared_ptr on IM application type
   // and any loaded IM_ parameters.  Error checks are made based
   // on application type.  Any fatal error causes a log message
   // and a program exit.
   //
   // applicationtype:IM application type.
   //
   // returns the IM algorithm shared_ptr.


   bool fatalerr = false;

   shared_ptr<IMAlgorithm> imalg;

   if (im_algorithm_type == IMUtils::IMAlgorithmTypes::NONE) {
      imalg = shared_ptr<IMAlgorithm>(new IMNone());
   } else if (im_algorithm_type == IMUtils::IMAlgorithmTypes::KINETICACHIEVE) {
      fatalerr = (!m_im_precalc_achieve.IsLoaded());

      if (!fatalerr) {
         imalg = shared_ptr<IMAlgorithm>(new IMPrecalcAchieve(m_im_precalc_achieve));
      }
   } else if (im_algorithm_type == IMUtils::IMAlgorithmTypes::KINETICTARGETACHIEVE) {
      fatalerr = (!m_im_precalc_achieve.IsLoaded());

      if (!fatalerr) {
         imalg = shared_ptr<IMAlgorithm>(new IMPrecalcTargetAchieve(m_im_precalc_achieve));
      }
   } else if (im_algorithm_type == IMUtils::IMAlgorithmTypes::TIMEBASEDACHIEVE) {
      fatalerr = (!m_im_time_based_achieve.IsLoaded());

      if (!fatalerr) {
         imalg = shared_ptr<IMAlgorithm>(new IMTimeBasedAchieve(m_im_time_based_achieve));
      }
   } else if (im_algorithm_type == IMUtils::IMAlgorithmTypes::TIMEBASEDACHIEVEMUTABLEASG) {
      fatalerr = (!m_im_time_based_achieve_mutable_asg.IsLoaded());

      if (!fatalerr) {
         imalg = shared_ptr<IMAlgorithm>(new IMTimeBasedAchieveMutableASG(m_im_time_based_achieve_mutable_asg));
      }
   } else if (im_algorithm_type == IMUtils::IMAlgorithmTypes::DISTANCEBASEDACHIEVE) {
      fatalerr = (!m_im_dist_based_achieve.IsLoaded());

      if (!fatalerr) {
         imalg = shared_ptr<IMAlgorithm>(new IMDistBasedAchieve(m_im_dist_based_achieve));
      }
   } else if (im_algorithm_type == IMUtils::IMAlgorithmTypes::RTA) {
      fatalerr = (!m_rta.IsLoaded());

      if (!fatalerr) {
         imalg = shared_ptr<IMAlgorithm>(std::make_shared<Rta>(m_rta));
      }
   } else if (im_algorithm_type == IMUtils::IMAlgorithmTypes::TESTSPEEDCONTROL) {
      if (!fatalerr) {
         imalg = shared_ptr<IMAlgorithm>(std::make_shared<TestVectorSpeedControl>(m_testspeedcontrol));
      }
   } else {
      fatalerr = true;
   }

   if (fatalerr) {
      char msg[100];
      sprintf(msg, "%s %d",
              "Parameters were not loaded for application type",
              im_algorithm_type);
      LOG4CPLUS_FATAL(IMAlgorithmLoader::m_logger, msg);
      throw logic_error(msg);
   }

   return imalg;
}

const IMUtils::IMAlgorithmTypes IMAlgorithmLoader::GetAlgorithmType(string algorithm_name) const {
   map<string, IMUtils::IMAlgorithmTypes>::const_iterator itr = m_algorithm_mapping.find(algorithm_name);
   if (itr == m_algorithm_mapping.end()) {
      LOG4CPLUS_FATAL(m_logger, "Not a valid algorithm string: " + algorithm_name + ". Something is wrong.");
   } else {
      return itr->second;
   }

   return IMUtils::IMAlgorithmTypes::NONE; // just use a default return and hope the user notices the FATAL logger
}

