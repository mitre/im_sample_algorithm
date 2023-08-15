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

#include <gtest/gtest.h>
#include "loader/LoadError.h"
#include "loader/DecodedStream.h"
#include "imalgs/IMClearanceLoader.h"
#include "imalgs/IMClearance.h"
#include "loader/Loadable.h"
#include "imalgs/IMUtils.h"
#include "public/SingleTangentPlaneSequence.h"
#include "utils/public/PublicUtils.h"

using namespace std;
using namespace aaesim::test::utils;
using namespace interval_management::open_source;
using namespace interval_management;

namespace aaesim {
namespace test {
namespace imalgo {

class WrapIMClearance : public Loadable {

   /**
    * Wrapper class to test IMClearance::load.
    */

  public:
   WrapIMClearance(void){};

   ~WrapIMClearance(void){};

   bool load(DecodedStream *input) {

      set_stream(input);

      register_loadable_with_brackets("IM_Clearance", &mIMLoader, false);

      bool loaded = complete();

      return loaded;
   };

   IMClearance build(void) { return mIMLoader.BuildClearance(); };

  private:
   IMClearanceLoader mIMLoader;
};

bool validateTest(WrapIMClearance loader, const AircraftIntent &intent, const IMUtils::IMAlgorithmTypes apptype) {
   IMClearance clearance = loader.build();
   return clearance.Validate(intent, apptype);
}

bool validateTestDeconflictIds(WrapIMClearance loader, const AircraftIntent &intent,
                               const IMUtils::IMAlgorithmTypes apptype) {
   // AircraftIntent secondIntent(intent);
   // secondIntent.SetId(intent.GetId() + 1);  // ensure they have different ids
   IMClearance clearance = loader.build();
   return clearance.Validate(intent, apptype);
}

TEST(IMClearance, basic) {

   auto builder1 = IMClearance::Builder(IMClearance::ClearanceType::MAINTAIN, 4, AircraftIntent(), "VENUS", "PLUTO",
                                        IMClearance::SpacingGoalType::DIST, 108);
   IMClearance c0 = builder1.Build();

   if (c0.GetClearanceType() != IMClearance::ClearanceType::MAINTAIN) {
      FAIL();
   }

   // 2) Basic getters.

   EXPECT_EQ(4, c0.GetTargetId());

   if (c0.GetAchieveByPoint() != "VENUS") {
      FAIL();
   }

   if (c0.GetPlannedTerminationPoint() != "PLUTO") {
      FAIL();
   }

   if (c0.GetAssignedSpacingGoalType() != IMClearance::SpacingGoalType::DIST) {
      FAIL();
   }

   if (c0.GetAssignedDistanceSpacingGoal() != Units::NauticalMilesLength(108)) {
      FAIL();
   }

   // 3) Goal getters usage tests.
   auto builder2 = IMClearance::Builder(IMClearance::ClearanceType::CAPTURE, 4, AircraftIntent(), "YOKXO", "DERVL",
                                        IMClearance::SpacingGoalType::TIME, 120);
   IMClearance c1 = builder2.Build();

   // Correct usage
   try {
      c0.GetAssignedDistanceSpacingGoal();
      if (c1.GetAssignedTimeSpacingGoal() != Units::SecondsTime(120)) {
         FAIL();
      }
   } catch (logic_error &e) {
      cout << e.what() << endl;
      FAIL();
   }

   // Attempted time getter on distance clearance should throw an error
   try {
      c0.GetAssignedTimeSpacingGoal();
      FAIL();  // above code should have throw an error
   } catch (logic_error &e) {
      // expected
   }

   // Attempted distance getter on time clearance should throw an error
   try {
      c1.GetAssignedDistanceSpacingGoal();
      FAIL();  // above code should have throw an error
   } catch (logic_error &e) {
      // expected
   }

   // 4) Comparator Operators
   auto builder3 = IMClearance::Builder(IMClearance::ClearanceType::CAPTURE, 4, AircraftIntent(), "YOKXO", "DERVL",
                                        IMClearance::SpacingGoalType::TIME, 120);
   IMClearance c2 = builder3.Build();
   auto builder4 = IMClearance::Builder(IMClearance::ClearanceType::ACHIEVE, 4, AircraftIntent(), "YOKXO", "DERVL",
                                        IMClearance::SpacingGoalType::TIME, 120);
   IMClearance c3 = builder4.Build();

   // ==
   if (!(c1 == c2)) {
      FAIL();
   }

   // !=
   if (!(c1 != c3)) {
      FAIL();
   }

   // 5) Operator = (copy)
   IMClearance c4 = c2;
   if (!(c4 == c2)) {
      FAIL();
   }
}

TEST(IMClearance, load_build) {
   SingleTangentPlaneSequence::ClearStaticMembers();
   DecodedStream stream;
   string testData = "./resources/imClearanceTimeValidSpacing.txt";
   FILE *fp;
   fp = fopen(testData.c_str(), "r");
   if (fp == NULL) {
      stream.report_error("Cannot find imClearanceTimeValidSpacing.txt\n");
      FAIL();
   }

   bool r = stream.open_file(testData);
   if (!r) {
      stream.report_error("Cannot load imClearanceTimeValidSpacing.txt\n");
      FAIL();
   }

   stream.set_echo(false);
   WrapIMClearance loader;
   if (!loader.load(&stream)) {
      stream.report_error("IM Clearance failed to load\n");
      FAIL();
   }
   fclose(fp);

   EXPECT_NO_THROW(IMClearance clearBasicBuild = loader.build());
}

TEST(IMClearance, validate_null_clearance) {
   DecodedStream stream;
   string testData = "./resources/imClearanceTimeValidSpacing.txt";
   FILE *fp;
   fp = fopen(testData.c_str(), "r");
   if (fp == NULL) {
      stream.report_error("Cannot find imClearanceTimeValidSpacing.txt\n");
      FAIL();
   }

   bool r = stream.open_file(testData);
   if (!r) {
      stream.report_error("Cannot load imClearanceTimeValidSpacing.txt\n");
      FAIL();
   }

   stream.set_echo(false);

   WrapIMClearance loader;
   if (!loader.load(&stream)) {
      stream.report_error("IM Clearance failed to load\n");
      FAIL();
   }

   IMClearance null_clearance;

   // These should be valid intents, so we know it's actually the null clearance causing validate() to fail.
   AircraftIntent valid_ownship = PublicUtils::PrepareAircraftIntent("./resources/imClearValidIntent.txt");
   AircraftIntent valid_target = PublicUtils::PrepareAircraftIntent("./resources/imClearValidIntent.txt");

   // Set these to be different so we know the test isn't failing due to "identical" indents.
   valid_ownship.SetId(0);
   valid_target.SetId(1);

   try {
      null_clearance.Validate(valid_ownship, IMUtils::IMAlgorithmTypes::DISTANCEBASEDACHIEVE);
      stream.report_error("Clearance validation with null clearance failed to throw expected exception.");
      FAIL();
   } catch (LoadError &le) {
      // Expected result.
   }

   fclose(fp);
}

TEST(IMClearance, validate_clearance_valid_ownship_null_target) {
   DecodedStream stream;
   std::string testData = "./resources/imClearanceTimeValidSpacing.txt";
   FILE *fp;
   fp = fopen(testData.c_str(), "r");
   if (fp == NULL) {
      stream.report_error("Cannot find imClearanceTimeValidSpacing.txt\n");
      FAIL();
   }

   bool r = stream.open_file(testData);
   if (!r) {
      stream.report_error("Cannot load imClearanceTimeValidSpacing.txt\n");
      FAIL();
   }
   stream.set_echo(false);

   WrapIMClearance loader;
   if (!loader.load(&stream)) {
      stream.report_error("IM Clearance failed to load\n");
      FAIL();
   }

   AircraftIntent valid_ownship = PublicUtils::PrepareAircraftIntent("./resources/imClearValidIntent.txt");
   AircraftIntent null_target;

   // Set these to be different so we know the test isn't failing due to "identical" indents.
   valid_ownship.SetId(0);
   null_target.SetId(1);

   IMClearance clearance = loader.build();
   try {
      clearance.Validate(valid_ownship,
                         IMUtils::IMAlgorithmTypes::TIMEBASEDACHIEVE);  // expect this to throw because of null_target
      stream.report_error("Clearance validation vs null target aircraft intent failed to throw expected exception.");
      // FIXME imClearanceTimeValidSpacing contains a target_intent, as required.
      // Unable to inject null_target, so Validate() will no longer fail.
      // FAIL();
   } catch (LoadError &le) {
      // Expected result.
   }

   fclose(fp);
}

TEST(IMClearance, validate_clearance_invalid_reference_point_target) {
   // Open file, setup stream.

   DecodedStream stream;

   std::string testData = "./resources/imClearanceTimeValidSpacing.txt";
   FILE *fp;
   fp = fopen(testData.c_str(), "r");
   if (fp == NULL) {
      stream.report_error("Cannot find imClearanceTimeValidSpacing.txt\n");
      FAIL();
   }

   bool r = stream.open_file(testData);
   if (!r) {
      stream.report_error("Cannot load imClearanceTimeValidSpacing.txt\n");
      FAIL();
   }

   // Set echo to false.  Turned on by EchoStream.

   stream.set_echo(false);

   WrapIMClearance loader;

   if (!loader.load(&stream)) {
      stream.report_error("IM Clearance failed to load\n");
      FAIL();
   }

   IMClearance clearVsInvalidTargetReferencePtTarget = loader.build();

   AircraftIntent valid_ownship = PublicUtils::PrepareAircraftIntent("./resources/imClearValidIntent.txt");
   // AircraftIntent invalid_reference_point_target =
   //       Preparators::PrepareAircraftIntent("./resources/TACUStoRELINIntent.txt");

   // Set these to be different so we know the test isn't failing due to "identical" indents.
   valid_ownship.SetId(0);
   // invalid_reference_point_target.SetId(1);

   try {
      clearVsInvalidTargetReferencePtTarget.Validate(valid_ownship, IMUtils::IMAlgorithmTypes::TIMEBASEDACHIEVE);
      stream.report_error("Clearance validation vs null target aircraft intent failed to throw expected exception.");
      // FIXME imClearanceTimeValidSpacing contains a target_intent, as required.
      // Unable to inject null_target, so Validate() will no longer fail.
      // FAIL();
   } catch (LoadError &le) {
      // Expected result.
   }

   fclose(fp);
}

TEST(IMClearance, validate_clearance_invalid_planned_termination_point) {
   // Open file, setup stream.

   DecodedStream stream;

   std::string testData = "./resources/imClearanceTimeValidSpacing.txt";
   FILE *fp;
   fp = fopen(testData.c_str(), "r");
   if (fp == NULL) {
      stream.report_error("Cannot find imClearanceTimeValidSpacing.txt\n");
      FAIL();
   }

   bool r = stream.open_file(testData);
   if (!r) {
      stream.report_error("Cannot load imClearanceTimeValidSpacing.txt\n");
      FAIL();
   }

   // Set echo to false.  Turned on by EchoStream.

   stream.set_echo(false);

   WrapIMClearance loader;

   if (!loader.load(&stream)) {
      stream.report_error("IM Clearance failed to load\n");
      FAIL();
   }

   IMClearance clearVsInvalidPlannedTerminationPtTarget = loader.build();

   AircraftIntent valid_ownship = PublicUtils::PrepareAircraftIntent("./resources/imClearValidIntent.txt");
   // AircraftIntent invalid_termination_point_target =
   //       Preparators::PrepareAircraftIntent("./resources/imClearInvalidTerminationPtIntent.txt");

   // Set these to be different so we know the test isn't failing due to "identical" indents.
   valid_ownship.SetId(0);
   // invalid_termination_point_target.SetId(1);

   try {
      clearVsInvalidPlannedTerminationPtTarget.Validate(valid_ownship, IMUtils::IMAlgorithmTypes::NONE);
      // stream.report_error("Clearance validation vs target missing waypoint matching clearance planned termination
      // point failed to throw expected exception."); FAIL();
   } catch (LoadError &le) {
      // Expected result.
      std::cout << le.what() << std::endl;
      FAIL();
   }

   fclose(fp);
}

TEST(IMClearance, validate_clearance_valid_intents) {
   // Open file, setup stream.

   DecodedStream stream;

   std::string testData = "./resources/imClearanceTimeValidSpacing.txt";
   FILE *fp;
   fp = fopen(testData.c_str(), "r");
   if (fp == NULL) {
      stream.report_error("Cannot find imClearanceTimeValidSpacing.txt\n");
      FAIL();
   }

   bool r = stream.open_file(testData);
   if (!r) {
      stream.report_error("Cannot load imClearanceTimeValidSpacing.txt\n");
      FAIL();
   }

   // Set echo to false.  Turned on by EchoStream.

   stream.set_echo(false);

   WrapIMClearance loader;

   if (!loader.load(&stream)) {
      stream.report_error("IM Clearance failed to load\n");
      FAIL();
   }

   IMClearance clearVsValidTarget = loader.build();

   AircraftIntent valid_ownship = PublicUtils::PrepareAircraftIntent("./resources/imClearValidIntent.txt");
   // AircraftIntent valid_target = Preparators::PrepareAircraftIntent("./resources/imClearValidIntent.txt");

   // Set these to be different so we know the test isn't failing due to "identical" indents.
   valid_ownship.SetId(0);
   // valid_target.SetId(1);

   try {
      if (!clearVsValidTarget.Validate(valid_ownship, IMUtils::IMAlgorithmTypes::NONE)) {
         stream.report_error("Clearance validation with valid target returned false. true was expected.");
         FAIL();
      }
   } catch (LoadError &le) {
      cout << le.what() << endl;
      FAIL();
   }

   fclose(fp);
}

TEST(IMClearance, default_assigned_dist_spacing_tests) {

   // Tests validation using default (unset) assigned spacing for distance type.

   DecodedStream stream;
   string testData = "./resources/imClearanceDistDefaultSpacing.txt";
   FILE *fp;
   fp = fopen(testData.c_str(), "r");
   if (fp == NULL) {
      stream.report_error("Cannot find imClearanceDistDefaultSpacing.txt\n");
      FAIL();
   }

   bool r = stream.open_file(testData);
   if (!r) {
      stream.report_error("Cannot load imClearanceDistDefaultSpacing.txt\n");
      FAIL();
   }
   stream.set_echo(false);

   WrapIMClearance loader;
   if (!loader.load(&stream)) {
      stream.report_error("IM Clearance failed to load\n");
      FAIL();
   }

   AircraftIntent intent = PublicUtils::PrepareAircraftIntent("./resources/imClearValidIntent.txt");

   // Run test for each application type-NONE should pass.  The rest should throw.
   try {
      if (validateTest(loader, intent, IMUtils::IMAlgorithmTypes::TIMEBASEDACHIEVE)) {
         stream.report_error("IMClearance default distance validation for TIMEBASEDACHIEVE unexpectedly passed.");
         FAIL();
      }
   } catch (LoadError &le) {
      // expected result.
   }

   try {
      if (validateTest(loader, intent, IMUtils::IMAlgorithmTypes::DISTANCEBASEDACHIEVE)) {
         stream.report_error("IMClearance default distance validation for DISTANCEBASEDACHIEVE unexpectedly passed.");
         FAIL();
      }
   } catch (LoadError &le) {
      // Expected result
   }

   try {
      if (validateTest(loader, intent, IMUtils::IMAlgorithmTypes::KINETICACHIEVE)) {
         stream.report_error("IMClearance default distance validation for KINETICACHIEVE unexpectedly passed.");
         FAIL();
      }
   } catch (LoadError &le) {
      // Expected result
   }

   try {
      if (validateTest(loader, intent, IMUtils::IMAlgorithmTypes::KINETICTARGETACHIEVE)) {
         stream.report_error("IMClearance default distance validation for KINETICTARGETACHIEVE unexpectedly passed.");
         FAIL();
      }
   } catch (LoadError &le) {
      // Expected result
   }

   try {
      if (!validateTest(loader, intent, IMUtils::IMAlgorithmTypes::NONE)) {
         stream.report_error("IMClearance default distance validation for NONE unexpectedly passed.");
         FAIL();
      }
   } catch (LoadError &le) {
      FAIL();
   }

   fclose(fp);
}

TEST(IMClearance, default_assigned_time_spacing_tests) {

   // Tests validation using default (unset) assigned spacing for time type.

   // Open file, setup stream.

   DecodedStream stream;

   string testData = "./resources/imClearanceTimeDefaultSpacing.txt";

   FILE *fp;
   fp = fopen(testData.c_str(), "r");
   if (fp == NULL) {
      stream.report_error("Cannot find imClearanceTimeDefaultSpacing.txt\n");
      FAIL();
   }

   bool r = stream.open_file(testData);
   if (!r) {
      stream.report_error("Cannot load imClearanceTimeDefaultSpacing.txt\n");
      FAIL();
   }

   // Set echo to false.  Turned on by EchoStream.

   stream.set_echo(false);

   WrapIMClearance loader;

   if (!loader.load(&stream)) {
      stream.report_error("IM Clearance failed to load\n");
      FAIL();
   }

   // Build intent.

   AircraftIntent intent = PublicUtils::PrepareAircraftIntent("./resources/imClearValidIntent.txt");

   // Run test for each application type - NONE should pass.  The rest should fail.

   try {
      if (validateTest(loader, intent, IMUtils::IMAlgorithmTypes::TIMEBASEDACHIEVE)) {
         stream.report_error("IMClearance default time validation for TIMEBASEDACHIEVE unexpectedly passed.");
         FAIL();
      }
   } catch (LoadError &le) {
      // execpted result
   }

   try {
      if (validateTest(loader, intent, IMUtils::IMAlgorithmTypes::DISTANCEBASEDACHIEVE)) {
         stream.report_error("IMClearance default time validation for DISTANCEBASEDACHIEVE unexpectedly passed.");
         FAIL();
      }
   } catch (LoadError &le) {
      // expected result
   }

   try {
      if (validateTest(loader, intent, IMUtils::IMAlgorithmTypes::KINETICACHIEVE)) {
         stream.report_error("IMClearance default time validation for KINETICACHIEVE unexpectedly passed.");
         FAIL();
      }
   } catch (LoadError &le) {
      // expected result
   }

   try {
      if (validateTest(loader, intent, IMUtils::IMAlgorithmTypes::KINETICTARGETACHIEVE)) {
         stream.report_error("IMClearance default time validation for KINETICTARGETACHIEVE unexpectedly passed.");
         FAIL();
      }
   } catch (LoadError &le) {
      // expected result
   }

   try {
      if (!validateTest(loader, intent, IMUtils::IMAlgorithmTypes::NONE)) {
         stream.report_error("IMClearance default time validation for NONE unexpectedly failed.");
         FAIL();
      }
   } catch (LoadError &le) {
      stream.report_error("Exception caught validating clearance with default time spacing for NONE type");
      FAIL();
   }

   fclose(fp);
}

TEST(IMClearance, valid_assigned_dist_spacing_tests) {

   // Tests validation using valid assigned spacing for distance type.

   DecodedStream stream;
   string testData = "./resources/imClearanceDistValidSpacing.txt";
   FILE *fp;
   fp = fopen(testData.c_str(), "r");
   if (fp == NULL) {
      stream.report_error("Cannot find imClearanceDistValidSpacing.txt\n");
      FAIL();
   }

   bool r = stream.open_file(testData);
   if (!r) {
      stream.report_error("Cannot load imClearanceDistValidSpacing.txt\n");
      FAIL();
   }
   stream.set_echo(false);

   WrapIMClearance loader;
   if (!loader.load(&stream)) {
      stream.report_error("IM Clearance failed to load\n");
      FAIL();
   }
   AircraftIntent intent = PublicUtils::PrepareAircraftIntent("./resources/imClearValidIntent.txt");

   // Run test for each application type-DISTANCEBASEDACHIEVE, NONE should pass.  The rest should throw.
   try {
      if (validateTest(loader, intent, IMUtils::IMAlgorithmTypes::TIMEBASEDACHIEVE)) {
         stream.report_error("IMClearance default distance validation for TIMEBASEDACHIEVE unexpectedly passed.");
         FAIL();
      }
   } catch (LoadError &le) {
      // expected result
   }

   try {
      if (!validateTest(loader, intent, IMUtils::IMAlgorithmTypes::DISTANCEBASEDACHIEVE)) {
         stream.report_error("IMClearance default distance validation for DISTANCEBASEDACHIEVE unexpectedly failed.");
         FAIL();
      }
   } catch (LoadError &le) {
      stream.report_error(
            "Exception caught validating clearance with default distance spacing for DISTANCEBASEDACHIEVE type");
      FAIL();
   }

   try {
      if (validateTest(loader, intent, IMUtils::IMAlgorithmTypes::KINETICACHIEVE)) {
         stream.report_error("IMClearance default distance validation for KINETICACHIEVE unexpectedly passed.");
         FAIL();
      }
   } catch (LoadError &le) {
      // exepected result
   }

   try {
      if (validateTest(loader, intent, IMUtils::IMAlgorithmTypes::KINETICTARGETACHIEVE)) {
         stream.report_error("IMClearance default distance validation for KINETICTARGETACHIEVE unexpectedly passed.");
         FAIL();
      }
   } catch (LoadError &le) {
      // expected result
   }

   try {
      if (!validateTest(loader, intent, IMUtils::IMAlgorithmTypes::NONE)) {
         stream.report_error("IMClearance default distance validation for NONE unexpectedly passed.");
         FAIL();
      }
   } catch (LoadError &le) {
      // expected result
   }

   fclose(fp);
}

TEST(IMClearance, valid_assigned_time_spacing_tests) {

   // Tests validation using valid assigned spacing for time type.

   // Open file, setup stream.

   DecodedStream stream;

   string testData = "./resources/imClearanceTimeValidSpacing.txt";

   FILE *fp;
   fp = fopen(testData.c_str(), "r");
   if (fp == NULL) {
      stream.report_error("Cannot find imClearanceTimeValidSpacing.txt\n");
      FAIL();
   }

   bool r = stream.open_file(testData);
   if (!r) {
      stream.report_error("Cannot load imClearanceTimeValidSpacing.txt\n");
      FAIL();
   }

   // Set echo to false.  Turned on by EchoStream.

   stream.set_echo(false);

   WrapIMClearance loader;

   if (!loader.load(&stream)) {
      stream.report_error("IM Clearance failed to load\n");
      FAIL();
   }

   // Build intent.

   AircraftIntent intent = PublicUtils::PrepareAircraftIntent("./resources/imClearValidIntent.txt");

   // Run test for each application type-DISTANCEBASEDACHIEVE should fail.  The others should pass.
   try {
      if (!validateTest(loader, intent, IMUtils::IMAlgorithmTypes::TIMEBASEDACHIEVE)) {
         stream.report_error("IMClearance default time validation for TIMEBASEDACHIEVE unexpectedly failed.");
         FAIL();
      }
   } catch (LoadError &le) {
      stream.report_error("Exception caught validating clearance with default time spacing for TIMEBASEDACHIEVE type");
      FAIL();
   }

   try {
      if (validateTest(loader, intent, IMUtils::IMAlgorithmTypes::DISTANCEBASEDACHIEVE)) {
         stream.report_error("IMClearance default time validation for DISTANCEBASEDACHIEVE unexpectedly passed.");
         FAIL();
      }
   } catch (LoadError &le) {
      // expected result
   }

   try {
      if (!validateTest(loader, intent, IMUtils::IMAlgorithmTypes::KINETICACHIEVE)) {
         stream.report_error("IMClearance default time validation for KINETICACHIEVE unexpectedly failed.");
         FAIL();
      }
   } catch (LoadError &le) {
      stream.report_error("Exception caught validating clearance with default time spacing for KINETICACHIEVE type");
      FAIL();
   }

   try {
      if (!validateTest(loader, intent, IMUtils::IMAlgorithmTypes::KINETICTARGETACHIEVE)) {
         stream.report_error("IMClearance default time validation for KINETICTARGETACHIEVE unexpectedly failed.");
         FAIL();
      }
   } catch (LoadError &le) {
      stream.report_error(
            "Exception caught validating clearance with default time spacing for KINETICTARGETACHIEVE type");
      FAIL();
   }

   try {
      if (!validateTest(loader, intent, IMUtils::IMAlgorithmTypes::NONE)) {
         stream.report_error("IMClearance default time validation for NONE unexpectedly failed.");
         FAIL();
      }
   } catch (LoadError &le) {
      stream.report_error("Exception caught validating clearance with default time spacing for NONE type");
      FAIL();
   }

   fclose(fp);
}

TEST(IMClearance, FAS_MissingMergeAngle) {
   // Test for _invalid_ FAS configurations.
   DecodedStream stream;
   string clearanceData = "./resources/imClearanceInvalidFAS.txt";
   string aiData = "./resources/imClearanceValidFAS_AircraftIntent.txt";

   FILE *fp;
   fp = fopen(clearanceData.c_str(), "r");
   if (fp == NULL) {
      stream.report_error("Cannot find " + clearanceData);
      FAIL();
   }

   bool r = stream.open_file(clearanceData);
   if (!r) {
      stream.report_error("Cannot load " + clearanceData);
      FAIL();
   }

   // Set echo to false.  Turned on by EchoStream.
   stream.set_echo(false);
   WrapIMClearance loader;
   if (!loader.load(&stream)) {
      stream.report_error("IM Clearance failed to load\n");
      FAIL();
   }

   // Build intent.
   AircraftIntent intent = PublicUtils::PrepareAircraftIntent(aiData);

   // Just try to validate. Should fail because clearance is malformed. If it fails, the test passes.
   try {
      if (validateTestDeconflictIds(loader, intent, IMUtils::IMAlgorithmTypes::TIMEBASEDACHIEVE)) {
         // Validate should not pass.
         FAIL();
      }
   } catch (LoadError &loadError) {
      // If here, the test passed!
   }
}

TEST(IMClearance, FAS_WrongAppType) {
   // Test FAS configurations.
   DecodedStream stream;
   string clearanceData = "./resources/imClearanceValidFAS.txt";
   string aiData = "./resources/imClearanceValidFAS_AircraftIntent.txt";

   FILE *fp;
   fp = fopen(clearanceData.c_str(), "r");
   if (fp == NULL) {
      stream.report_error("Cannot find " + clearanceData);
      FAIL();
   }

   bool r = stream.open_file(clearanceData);
   if (!r) {
      stream.report_error("Cannot load " + clearanceData);
      FAIL();
   }

   stream.set_echo(false);
   WrapIMClearance loader;
   if (!loader.load(&stream)) {
      stream.report_error("IM Clearance failed to load\n");
      FAIL();
   }

   AircraftIntent intent = PublicUtils::PrepareAircraftIntent(aiData);

   // Run test for WRONG application type
   std::vector<IMUtils::IMAlgorithmTypes> invalidTypes;
   invalidTypes.push_back(IMUtils::IMAlgorithmTypes::KINETICACHIEVE);
   invalidTypes.push_back(IMUtils::IMAlgorithmTypes::KINETICTARGETACHIEVE);
   for (auto type = invalidTypes.begin(); type != invalidTypes.end(); ++type) {
      try {
         if (validateTestDeconflictIds(loader, intent, *type)) {
            // Validate should not pass. If here, test has failed
            FAIL();
         }
      } catch (LoadError &loadError) {
         // Expect a throw to occur. If here, the test passed!
      }
   }
}

TEST(IMClearance, FAS_InvalidIntent) {
   // Test FAS configurations.
   DecodedStream stream;
   string clearanceData = "./resources/imClearanceValidFAS.txt";
   string aiData = "./resources/aircraft-flying-east.txt";  // this is bad intent and should fail to validate

   FILE *fp;
   fp = fopen(clearanceData.c_str(), "r");
   if (fp == NULL) {
      stream.report_error("Cannot find " + clearanceData);
      FAIL();
   }

   bool r = stream.open_file(clearanceData);
   if (!r) {
      stream.report_error("Cannot load " + clearanceData);
      FAIL();
   }

   // Set echo to false.  Turned on by EchoStream.
   stream.set_echo(false);
   WrapIMClearance loader;
   if (!loader.load(&stream)) {
      stream.report_error("IM Clearance failed to load\n");
      FAIL();
   }

   // Build intent.
   AircraftIntent intent = PublicUtils::PrepareAircraftIntent(aiData);

   // Run test for WRONG application type
   try {
      if (validateTestDeconflictIds(loader, intent, IMUtils::IMAlgorithmTypes::TIMEBASEDACHIEVE)) {
         // Validate should not pass. If here, test has failed
         FAIL();
      }
   } catch (LoadError &loadError) {
      // If here, the test passed!
   }
}
}  // namespace imalgo
}  // namespace test
}  // namespace aaesim
