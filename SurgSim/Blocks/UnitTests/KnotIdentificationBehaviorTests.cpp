// This file is a part of the OpenSurgSim project.
// Copyright 2013 - 2016, SimQuest Solutions Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <boost/assign/list_of.hpp>
#include <gtest/gtest.h>

#include "SurgSim/Blocks/KnotIdentificationBehavior.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/MathConvert.h"
#include <random>

namespace
{
std::vector<SurgSim::Blocks::KnotIdentificationBehavior::Crossing> createGaussCode(const std::vector<int>& ids,
	const std::vector<int>& signs = {})
{
	std::vector<SurgSim::Blocks::KnotIdentificationBehavior::Crossing> result;
	for (size_t i = 0; i < ids.size(); ++i)
	{
		result.emplace_back(ids[i], 0, 0.0, 1);
	}
	for (size_t i = 0; i < std::min(signs.size(), result.size()); ++i)
	{
		result[i].sign = signs[i];
	}
	return result;
}
}

namespace SurgSim
{
namespace Blocks
{

class MockKnotIdentificationBehavior : public KnotIdentificationBehavior
{
public:
	explicit MockKnotIdentificationBehavior(const std::string& name) : KnotIdentificationBehavior(name) {}

	bool tryReidmeisterMove1(std::vector<Crossing>* gaussCode, std::vector<int>* erased)
	{
		return KnotIdentificationBehavior::tryReidmeisterMove1(gaussCode, erased);
	}

	bool tryReidmeisterMove2(std::vector<Crossing>* gaussCode, std::vector<int>* erased)
	{
		return KnotIdentificationBehavior::tryReidmeisterMove2(gaussCode, erased);
	}

	bool tryReidmeisterMove3(std::vector<Crossing>* gaussCode)
	{
		return KnotIdentificationBehavior::tryReidmeisterMove3(gaussCode, &data);
	}

	void adjustGaussCodeForErasedCrossings(std::vector<Crossing>* gaussCode)
	{
		return KnotIdentificationBehavior::adjustGaussCodeForErasedCrossings(gaussCode);
	}

	std::string identifyKnot(const std::vector<Crossing>& gaussCode)
	{
		return KnotIdentificationBehavior::identifyKnot(gaussCode);
	}

	KnotIdentificationBehavior::ReidmeisterMove3Data data;
};

class KnotIdentificationBehaviorTest : public testing::Test
{
public:
	KnotIdentificationBehaviorTest() : testing::Test(), mockReidmeisterMove3Behavior("Test3")
	{

	}

	void SetUp()
	{

	}

	std::shared_ptr<SurgSim::Physics::Fem1DRepresentation> makeFem1D(
		const std::string& filename,
		SurgSim::Math::RigidTransform3d t = SurgSim::Math::RigidTransform3d::Identity())
	{
		auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
		auto physics = std::make_shared<SurgSim::Physics::Fem1DRepresentation>("Physics");
		physics->setFemElementType("SurgSim::Physics::Fem1DElementBeam");
		physics->setLocalPose(SurgSim::Math::RigidTransform3d::Identity());
		physics->loadFem(filename);
		physics->setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_EULER_IMPLICIT);
		physics->setLinearSolver(SurgSim::Math::LINEARSOLVER_LU);
		physics->setRayleighDampingMass(5.0);
		physics->setRayleighDampingStiffness(0.001);
		physics->setIsGravityEnabled(true);
		physics->initialize(runtime);
		return physics;
	}

	bool testKnot(std::string filename, std::string knotname)
	{
		KnotIdentificationBehavior knotId("Knot ID");

		using SurgSim::Math::makeRigidTransform;
		using SurgSim::Math::makeRotationMatrix;

		SurgSim::Math::Vector3d axis = SurgSim::Math::Vector3d::UnitX();
		for (int i = 0; i < 4; ++i)
		{
			double angle = 0.25 * i * M_PI * 2.0;
			auto  t = makeRigidTransform(makeRotationMatrix(angle, axis), SurgSim::Math::Vector3d::Zero());
			auto fem = makeFem1D(filename, t);
			knotId.setFem1d(fem);
			knotId.doInitialize();
			knotId.update(0.0);
			EXPECT_EQ(knotname, knotId.getKnotName());
		}

		axis = SurgSim::Math::Vector3d::UnitY();
		for (int i = 0; i < 4; ++i)
		{
			double angle = 0.25 * i * M_PI * 2.0;
			auto  t = makeRigidTransform(makeRotationMatrix(angle, axis), SurgSim::Math::Vector3d::Zero());
			auto fem = makeFem1D(filename, t);
			knotId.setFem1d(fem);
			knotId.doInitialize();
			knotId.update(0.0);
			EXPECT_EQ(knotname, knotId.getKnotName());
		}

		axis = SurgSim::Math::Vector3d::UnitZ();
		for (int i = 0; i < 4; ++i)
		{
			double angle = 0.25 * i * M_PI * 2.0;
			auto  t = makeRigidTransform(makeRotationMatrix(angle, axis), SurgSim::Math::Vector3d::Zero());
			auto fem = makeFem1D(filename, t);
			knotId.setFem1d(fem);
			knotId.doInitialize();
			knotId.update(0.0);
			EXPECT_EQ(knotname, knotId.getKnotName());
		}
		return true;
	}

	bool testKnotRandom(std::string filename, std::string knotname, int iterations = 1000, unsigned int seed = 1000)
	{
		std::default_random_engine generator;
		std::uniform_real_distribution<double> distribution(0, 1);
		
		generator.seed(seed);


		using SurgSim::Math::makeRigidTransform;
		using SurgSim::Math::makeRotationMatrix;

		auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

		auto mesh = std::make_shared<SurgSim::Physics::Fem1D>();
		mesh->load(filename);

		std::cout << knotname << ":";
		SurgSim::Math::Vector3d axis = SurgSim::Math::Vector3d::UnitX();
		for (int i = 0; i < iterations; ++i)
		{
			KnotIdentificationBehavior knotId("Knot ID");
			if (i % 100 == 0) std::cout << "+";

			double angle = distribution(generator) * M_PI * 2.0;
			auto axis = SurgSim::Math::Vector3d
			(distribution(generator), distribution(generator), distribution(generator));
			axis.normalize();
			auto  t = makeRigidTransform(makeRotationMatrix(angle, axis), SurgSim::Math::Vector3d::Zero());

			auto fem = std::make_shared<SurgSim::Physics::Fem1DRepresentation>("Physics");
			fem->setFemElementType("SurgSim::Physics::Fem1DElementBeam");
			fem->setLocalPose(t);
			fem->setFem(mesh);
			fem->setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_EULER_IMPLICIT);
			fem->setLinearSolver(SurgSim::Math::LINEARSOLVER_LU);
			fem->setRayleighDampingMass(5.0);
			fem->setRayleighDampingStiffness(0.001);
			fem->setIsGravityEnabled(true);
			fem->initialize(runtime);

			knotId.setFem1d(fem);
			knotId.doInitialize();
			knotId.update(0.0);
			EXPECT_EQ(knotname, knotId.getKnotName()) << "Axis: " << axis.transpose() << " Angle: " << angle;
			if (knotname != knotId.getKnotName())
			{
				using SurgSim::Math::toBytes;
				std::vector<uint8_t> bytes;
				toBytes(angle, &bytes);
				toBytes(axis, &bytes);

				std::ofstream out("knoterror.bin", std::ios_base::binary);
				SURGSIM_ASSERT(out.is_open());

				for (auto b : bytes) out << b;
				return false;
			}
		}

		std::cout << "\n";
		return true;
	}


	bool testKnotFromFailure(std::string filename, std::string knotname, std::string failureFile)
	{

		KnotIdentificationBehavior knotId("Knot ID");

		using SurgSim::Math::makeRigidTransform;
		using SurgSim::Math::makeRotationMatrix;

		auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

		auto mesh = std::make_shared<SurgSim::Physics::Fem1D>();
		mesh->load(filename);

		double angle;
		SurgSim::Math::Vector3d axis;

		using SurgSim::Math::fromBytes;
		std::ifstream in(failureFile, std::ios_base::binary);
		EXPECT_TRUE(!in.bad());
		std::vector<uint8_t> bytes(std::istreambuf_iterator<char>(in), {});

		auto end = fromBytes(bytes, &angle);
		end += fromBytes(bytes, &axis, end);
		EXPECT_EQ(end, bytes.size());
		auto  t = makeRigidTransform(makeRotationMatrix(angle, axis), SurgSim::Math::Vector3d::Zero());

		auto fem = std::make_shared<SurgSim::Physics::Fem1DRepresentation>("Physics");
		fem->setFemElementType("SurgSim::Physics::Fem1DElementBeam");
		fem->setLocalPose(t);
		fem->setFem(mesh);
		fem->setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_EULER_IMPLICIT);
		fem->setLinearSolver(SurgSim::Math::LINEARSOLVER_LU);
		fem->setRayleighDampingMass(5.0);
		fem->setRayleighDampingStiffness(0.001);
		fem->setIsGravityEnabled(true);
		fem->initialize(runtime);

		knotId.setFem1d(fem);
		knotId.doInitialize();
		knotId.update(0.0);
		EXPECT_EQ(knotname, knotId.getKnotName()) << "Axis: " << axis.transpose() << " Angle: " << angle;
		return true;
	}
	
	void testReidmeisterMove1(std::vector<KnotIdentificationBehavior::Crossing> gaussCodeBefore,
		std::vector<KnotIdentificationBehavior::Crossing> gaussCodeAfter,
							  std::vector<int> erased = std::vector<int>())
	{
		MockKnotIdentificationBehavior mockKnotIdentificationBehavior("Test ReidmeisterMove1");
		std::vector<int> erasedCalculated;
		bool expectedReturn = gaussCodeBefore != gaussCodeAfter;
		bool actualReturn = mockKnotIdentificationBehavior.tryReidmeisterMove1(&gaussCodeBefore, &erasedCalculated);
		EXPECT_EQ(expectedReturn, actualReturn);
		EXPECT_EQ(gaussCodeBefore, gaussCodeAfter);
		std::sort(erased.begin(), erased.end(), std::greater<int>());
		std::sort(erasedCalculated.begin(), erasedCalculated.end(), std::greater<int>());
		EXPECT_EQ(erased, erasedCalculated);
	}

	void testReidmeisterMove2(std::vector<KnotIdentificationBehavior::Crossing> gaussCodeBefore,
		std::vector<KnotIdentificationBehavior::Crossing> gaussCodeAfter,
							  std::vector<int> erased = std::vector<int>())
	{
		MockKnotIdentificationBehavior mockKnotIdentificationBehavior("Test ReidmeisterMove2");
		std::vector<int> erasedCalculated;
		bool expectedReturn = gaussCodeBefore != gaussCodeAfter;
		bool actualReturn = mockKnotIdentificationBehavior.tryReidmeisterMove2(&gaussCodeBefore, &erasedCalculated);
		EXPECT_EQ(expectedReturn, actualReturn);
		EXPECT_EQ(gaussCodeBefore, gaussCodeAfter);
		std::sort(erased.begin(), erased.end(), std::greater<int>());
		std::sort(erasedCalculated.begin(), erasedCalculated.end(), std::greater<int>());
		EXPECT_EQ(erased, erasedCalculated);
	}

	void testReidmeisterMove3(std::vector<KnotIdentificationBehavior::Crossing> gaussCodeBefore,
		std::vector<KnotIdentificationBehavior::Crossing> gaussCodeAfter, bool expectedReturn)
	{
		bool actualReturn = mockReidmeisterMove3Behavior.tryReidmeisterMove3(&gaussCodeBefore);
		EXPECT_EQ(expectedReturn, actualReturn);
		EXPECT_EQ(gaussCodeBefore, gaussCodeAfter);
	}

	void testAdjustGaussCodeForErasedCrossings(std::vector<KnotIdentificationBehavior::Crossing> gaussCodeBefore,
		std::vector<KnotIdentificationBehavior::Crossing> gaussCodeAfter, std::vector<int> erased)
	{
		MockKnotIdentificationBehavior mockKnotIdentificationBehavior("Test AdjustGaussCodeForErasedCrossings");
		mockKnotIdentificationBehavior.adjustGaussCodeForErasedCrossings(&gaussCodeBefore);
		EXPECT_EQ(gaussCodeBefore, gaussCodeAfter);
	}

	void testIdentifyKnot(std::vector<KnotIdentificationBehavior::Crossing> gaussCode, std::string expected)
	{
		MockKnotIdentificationBehavior mockKnotIdentificationBehavior("Test IdentifyKnot");
		auto actual = mockKnotIdentificationBehavior.identifyKnot(gaussCode);
		EXPECT_EQ(expected, actual);
		// Clear the list of known knot codes and identify again. If gaussCode is empty, it should say "No Knot",
		// otherwise "Unknown Knot".
		mockKnotIdentificationBehavior.clearKnownKnotCodes();
		actual = mockKnotIdentificationBehavior.identifyKnot(gaussCode);
		if (gaussCode.empty())
		{
			EXPECT_EQ("No Knot", actual);
		}
		else
		{
			EXPECT_EQ("Unknown Knot", actual);
		}
	}
	
	MockKnotIdentificationBehavior mockReidmeisterMove3Behavior;
};

TEST_F(KnotIdentificationBehaviorTest, Constructor)
{
	EXPECT_NO_THROW(KnotIdentificationBehavior knotId("KnotId"));
	EXPECT_NO_THROW(new KnotIdentificationBehavior("KnotId"));
	EXPECT_NO_THROW(auto x = std::make_shared<KnotIdentificationBehavior>("KnotId"));
}

TEST_F(KnotIdentificationBehaviorTest, GetSetFem1D)
{
	KnotIdentificationBehavior knotId("KnotId");
	EXPECT_EQ(nullptr, knotId.getFem1d());
	auto physics = std::make_shared<SurgSim::Physics::Representation>("Physics Representation");
	EXPECT_THROW(knotId.setFem1d(physics), SurgSim::Framework::AssertionFailure);
	EXPECT_FALSE(knotId.doWakeUp());
	auto fem1D = std::make_shared<SurgSim::Physics::Fem1DRepresentation>("Fem1D");
	EXPECT_NO_THROW(knotId.setFem1d(fem1D));
	EXPECT_TRUE(knotId.doWakeUp());
	EXPECT_EQ(fem1D->getName(), knotId.getFem1d()->getName());
}

TEST_F(KnotIdentificationBehaviorTest, ReidmeisterMove1)
{
	{
		std::vector<int> input = boost::assign::list_of(1)(-2)(3)(-1)(2)(-3);
		std::vector<int> expected = boost::assign::list_of(1)(-2)(3)(-1)(2)(-3);
		testReidmeisterMove1(createGaussCode(input), createGaussCode(expected));
	}
	{
		std::vector<int> input = boost::assign::list_of(1)(-2)(3)(-3)(-1)(2);
		std::vector<int> expected = boost::assign::list_of(1)(-2)(-1)(2);
		std::vector<int> erased = boost::assign::list_of(3);
		testReidmeisterMove1(createGaussCode(input), createGaussCode(expected), erased);
	}
	{
		std::vector<int> input = boost::assign::list_of(3)(1)(-2)(-1)(2)(-3);
		std::vector<int> expected = boost::assign::list_of(1)(-2)(-1)(2);
		std::vector<int> erased = boost::assign::list_of(3);
		testReidmeisterMove1(createGaussCode(input), createGaussCode(expected), erased);
	}
	{
		std::vector<int> input = boost::assign::list_of(3)(1)(4)(-4)(-2)(6)(-6)(-1)(-5)(5)(2)(-3);
		std::vector<int> expected = boost::assign::list_of(1)(-2)(-1)(2);
		std::vector<int> erased = boost::assign::list_of(3)(4)(5)(6);
		testReidmeisterMove1(createGaussCode(input), createGaussCode(expected), erased);
	}
	{
		std::vector<int> input = boost::assign::list_of(3)(1)(4)(-4)(-2)(6)(7)(-6)(-7)(-1)(-5)(5)(2)(-3);
		std::vector<int> expected = boost::assign::list_of(1)(-2)(6)(7)(-6)(-7)(-1)(2);
		std::vector<int> erased = boost::assign::list_of(3)(4)(5);
		testReidmeisterMove1(createGaussCode(input), createGaussCode(expected), erased);
	}
}

TEST_F(KnotIdentificationBehaviorTest, ReidmeisterMove2)
{
	{
		std::vector<int> input = boost::assign::list_of(3)(1)(-2)(-1)(2)(-3)(4)(5)(-4)(-5);
		std::vector<int> expected = boost::assign::list_of(3)(1)(-2)(-1)(2)(-3);
		std::vector<int> erased = boost::assign::list_of(4)(5);
		testReidmeisterMove2(createGaussCode(input), createGaussCode(expected), erased);
	}
	{
		std::vector<int> input = boost::assign::list_of(-5)(3)(1)(-2)(-1)(2)(-3)(4)(5)(-4);
		std::vector<int> expected = boost::assign::list_of(3)(1)(-2)(-1)(2)(-3);
		std::vector<int> erased = boost::assign::list_of(4)(5);
		testReidmeisterMove2(createGaussCode(input), createGaussCode(expected), erased);
	}
	{
		std::vector<int> input = boost::assign::list_of(3)(1)(-2)(7)(6)(-1)(2)(-3)(4)(5)(-7)(-6)(-4)(-5);
		std::vector<int> expected = boost::assign::list_of(3)(1)(-2)(-1)(2)(-3)(4)(5)(-4)(-5);
		std::vector<int> erased = boost::assign::list_of(6)(7);
		testReidmeisterMove2(createGaussCode(input), createGaussCode(expected), erased);
	}
}

TEST_F(KnotIdentificationBehaviorTest, ReidmeisterMove3Test1)
{
	std::vector<int> input = boost::assign::list_of(-1)(-3)(-2)(1)(2)(3);
	std::vector<int> expected = boost::assign::list_of(-1)(-3)(3)(-2)(1)(2);
	testReidmeisterMove3(createGaussCode(input), createGaussCode(expected), true);
	std::vector<int> expected2 = boost::assign::list_of(2)(-3)(-2)(3)(1)(-1);
	testReidmeisterMove3(createGaussCode(expected), createGaussCode(expected2), true);
	std::vector<int> expected3 = boost::assign::list_of(3)(-1)(-3)(1)(2)(-2);
	testReidmeisterMove3(createGaussCode(expected2), createGaussCode(expected3), true);
	std::vector<int> expected4 = boost::assign::list_of(-2)(-1)(1)(-3)(2)(3);
	testReidmeisterMove3(createGaussCode(expected3), createGaussCode(expected4), true);
	testReidmeisterMove3(createGaussCode(expected4), createGaussCode(input), false);
}

TEST_F(KnotIdentificationBehaviorTest, ReidmeisterMove3Test2)
{
	std::vector<int> input = boost::assign::list_of(3)(1)(-4)(-2)(-1)(-5)(2)(-3)(4)(5);
	std::vector<int> expected = boost::assign::list_of(3)(1)(-4)(-3)(-2)(-5)(-1)(2)(4)(5);
	testReidmeisterMove3(createGaussCode(input), createGaussCode(expected), true);
}

TEST_F(KnotIdentificationBehaviorTest, AdjustGaussCodeForErasedCrossings)
{
	{
		std::vector<int> input = boost::assign::list_of(3)(-4)(5)(-3)(4)(-5);
		std::vector<int> expected = boost::assign::list_of(1)(-2)(3)(-1)(2)(-3);
		std::vector<int> erased = boost::assign::list_of(1)(2);
		testAdjustGaussCodeForErasedCrossings(createGaussCode(input), createGaussCode(expected), erased);
	}
	{
		std::vector<int> input = boost::assign::list_of(1)(-4)(5)(-1)(4)(-5);
		std::vector<int> expected = boost::assign::list_of(1)(-2)(3)(-1)(2)(-3);
		std::vector<int> erased = boost::assign::list_of(2)(3);
		testAdjustGaussCodeForErasedCrossings(createGaussCode(input), createGaussCode(expected), erased);
	}
}

TEST_F(KnotIdentificationBehaviorTest, IdentifyKnotTest1)
{
	testIdentifyKnot(createGaussCode(std::vector<int>()), "No Knot");
}

TEST_F(KnotIdentificationBehaviorTest, IdentifyKnotTest2)
{
	std::vector<int> input = boost::assign::list_of(1)(-4)(5)(-1)(4)(-5);
	testIdentifyKnot(createGaussCode(input), "Unknown Knot");
}

TEST_F(KnotIdentificationBehaviorTest, IdentifyKnotTest3)
{
	std::vector<int> ids = boost::assign::list_of(1)(-2)(3)(-1)(2)(-3);
	std::vector<int> signs = boost::assign::list_of(1)(1)(1)(1)(1)(1);
	testIdentifyKnot(createGaussCode(ids, signs), "Trefoil Knot");
}

TEST_F(KnotIdentificationBehaviorTest, IdentifyKnotTest4)
{
	std::vector<int> ids = boost::assign::list_of(-1)(2)(-3)(1)(-2)(3);
	std::vector<int> signs = boost::assign::list_of(1)(1)(1)(1)(1)(1);
	testIdentifyKnot(createGaussCode(ids, signs), "Trefoil Knot");
}

TEST_F(KnotIdentificationBehaviorTest, IdentifyKnotTest5)
{
	std::vector<int> ids = boost::assign::list_of(1)(-2)(3)(-4)(5)(-6)(4)(-5)(6)(-1)(2)(-3);
	std::vector<int> signs = boost::assign::list_of(1)(1)(1)(1)(1)(1)(1)(1)(1)(1)(1)(1);
	testIdentifyKnot(createGaussCode(ids, signs), "Granny Knot");
}

TEST_F(KnotIdentificationBehaviorTest, IdentifyKnotTest6)
{
	std::vector<int> ids = boost::assign::list_of(-1)(2)(-3)(4)(-5)(6)(-4)(5)(-6)(1)(-2)(3);
	std::vector<int> signs = boost::assign::list_of(1)(1)(1)(1)(1)(1)(1)(1)(1)(1)(1)(1);
	testIdentifyKnot(createGaussCode(ids, signs), "Granny Knot");
}

TEST_F(KnotIdentificationBehaviorTest, IdentifyKnotTest7)
{
	std::vector<int> ids = boost::assign::list_of(1)(-2)(3)(4)(-5)(6)(-4)(5)(-6)(-1)(2)(-3);
	std::vector<int> signs = boost::assign::list_of(1)(1)(1)(-1)(-1)(-1)(-1)(-1)(-1)(1)(1)(1);
	testIdentifyKnot(createGaussCode(ids, signs), "Square Knot");
}

TEST_F(KnotIdentificationBehaviorTest, Fem1DNoKnot)
{
	KnotIdentificationBehavior knotId("Knot ID");
	knotId.setFem1d(makeFem1D("Geometry/loop.ply"));
	knotId.doInitialize();
	knotId.update(0.0);
	EXPECT_EQ("No Knot", knotId.getKnotName());
}

TEST_F(KnotIdentificationBehaviorTest, Fem1DTrefoil)
{
	EXPECT_TRUE(testKnot("Geometry/trefoil_knot.ply", "Trefoil Knot"));
}

TEST_F(KnotIdentificationBehaviorTest, Fem1DSquare)
{
	EXPECT_TRUE(testKnot("Geometry/square_knot.ply", "Square Knot"));
}

TEST_F(KnotIdentificationBehaviorTest, Fem1DGranny)
{
	EXPECT_TRUE(testKnot("Geometry/granny_knot.ply", "Granny Knot"));
}

TEST_F(KnotIdentificationBehaviorTest, Fem1DTrefoilRandom)
{
	std::default_random_engine generator;
	std::uniform_int_distribution<unsigned int> distribution;

	for (;;)
	{
		auto seed = distribution(generator);

		std::cout << seed << "\n";

		ASSERT_TRUE(testKnotRandom("Geometry/trefoil_knot.ply", "Trefoil Knot", 10000, seed));
		ASSERT_TRUE(testKnotRandom("Geometry/square_knot.ply", "Square Knot", 10000, seed));
		ASSERT_TRUE(testKnotRandom("Geometry/granny_knot.ply", "Granny Knot", 10000, seed));
	}
}

TEST_F(KnotIdentificationBehaviorTest, ErrorFiles)
{
		//ASSERT_TRUE(testKnotFromFailure("Geometry/square_knot.ply", "Square Knot", "knoterror1.bin"));
		ASSERT_TRUE(testKnotFromFailure("Geometry/square_knot.ply", "Square Knot", "knoterror.bin"));
		//ASSERT_TRUE(testKnotFromFailure("Geometry/square_knot.ply", "Square Knot", "knoterror3.bin"));
}

}
}
