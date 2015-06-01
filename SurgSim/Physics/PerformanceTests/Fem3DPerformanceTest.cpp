// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
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

#include <gtest/gtest.h>

#include <unordered_map>
#include <memory>

#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Timer.h"
#include "SurgSim/Math/LinearSparseSolveAndInverse.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem3DPlyReaderDelegate.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/Fem3DElementCube.h"
#include "SurgSim/Physics/PerformanceTests/DivisibleCubeRepresentation.h"
#include "SurgSim/Physics/PerformanceTests/MockFem3DRepresentation.h"
#include "SurgSim/Testing/MockPhysicsManager.h"

using SurgSim::Math::Vector3d;

namespace
{
static const double dt = 0.001;
static const double maxIterationConstant = 0.1;
static const double tolerance = 1.0e-03;
static const int frameCount = 10;

static std::unordered_map<SurgSim::Math::IntegrationScheme, std::string, std::hash<int>> getIntegrationSchemeNames()
{
	std::unordered_map<SurgSim::Math::IntegrationScheme, std::string, std::hash<int>> result;

#define FEM3DPERFORMANCETEST_MAP_NAME(map, name) (map)[name] = #name
	FEM3DPERFORMANCETEST_MAP_NAME(result, SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER);
	FEM3DPERFORMANCETEST_MAP_NAME(result, SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EXPLICIT_EULER);
	FEM3DPERFORMANCETEST_MAP_NAME(result, SurgSim::Math::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER);
	FEM3DPERFORMANCETEST_MAP_NAME(result, SurgSim::Math::INTEGRATIONSCHEME_LINEAR_MODIFIED_EXPLICIT_EULER);
	FEM3DPERFORMANCETEST_MAP_NAME(result, SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER);
	FEM3DPERFORMANCETEST_MAP_NAME(result, SurgSim::Math::INTEGRATIONSCHEME_LINEAR_IMPLICIT_EULER);
	FEM3DPERFORMANCETEST_MAP_NAME(result, SurgSim::Math::INTEGRATIONSCHEME_STATIC);
	FEM3DPERFORMANCETEST_MAP_NAME(result, SurgSim::Math::INTEGRATIONSCHEME_LINEAR_STATIC);
	FEM3DPERFORMANCETEST_MAP_NAME(result, SurgSim::Math::INTEGRATIONSCHEME_RUNGE_KUTTA_4);
	FEM3DPERFORMANCETEST_MAP_NAME(result, SurgSim::Math::INTEGRATIONSCHEME_LINEAR_RUNGE_KUTTA_4);
#undef FEM3DPERFORMANCETEST_MAP_NAME

	return result;
}

static std::unordered_map<SurgSim::Math::LinearSolver, std::string, std::hash<int>> getLinearSolverNames()
{
	std::unordered_map<SurgSim::Math::LinearSolver, std::string, std::hash<int>> result;

#define FEM3DPERFORMANCETEST_MAP_NAME(map, name) (map)[name] = #name
	FEM3DPERFORMANCETEST_MAP_NAME(result, SurgSim::Math::LINEARSOLVER_LU);
	FEM3DPERFORMANCETEST_MAP_NAME(result, SurgSim::Math::LINEARSOLVER_CONJUGATEGRADIENT);
#undef FEM3DPERFORMANCETEST_MAP_NAME

	return result;
}

static std::unordered_map<SurgSim::Math::IntegrationScheme, std::string, std::hash<int>> IntegrationSchemeNames
		= getIntegrationSchemeNames();

static std::unordered_map<SurgSim::Math::LinearSolver, std::string, std::hash<int>> LinearSolverNames
		= getLinearSolverNames();
}

namespace SurgSim
{
namespace Physics
{

class Fem3DPerformanceTestBase : public ::testing::Test
{
public:
	virtual void SetUp()
	{
		m_physicsManager = std::make_shared<SurgSim::Testing::MockPhysicsManager>();

		m_physicsManager->doInitialize();
		m_physicsManager->doStartUp();
	}

	void initializeRepresentation(std::shared_ptr<MockFem3DRepresentation> fem)
	{
		fem->initialize(std::make_shared<SurgSim::Framework::Runtime>());
		fem->wakeUp();
		std::shared_ptr<SurgSim::Math::LinearSparseSolveAndInverseCG> solver =
			std::dynamic_pointer_cast<SurgSim::Math::LinearSparseSolveAndInverseCG>(
				fem->getOdeSolver()->getLinearSolver());
		if (solver != nullptr)
		{
			solver->setMaxIterations(static_cast<SurgSim::Math::SparseMatrix::Index>(
										 maxIterationConstant * fem->getNumDof()));
			solver->setTolerance(tolerance);
		}

		m_physicsManager->executeAdditions(fem);
	}

	void initializeRepresentation(std::shared_ptr<DivisibleCubeRepresentation> fem)
	{
		fem->initialize(std::make_shared<SurgSim::Framework::Runtime>());
		fem->wakeUp();
		std::shared_ptr<SurgSim::Math::LinearSparseSolveAndInverseCG> solver =
			std::dynamic_pointer_cast<SurgSim::Math::LinearSparseSolveAndInverseCG>(
				fem->getOdeSolver()->getLinearSolver());
		if (solver != nullptr)
		{
			solver->setMaxIterations(static_cast<SurgSim::Math::SparseMatrix::Index>(
										 maxIterationConstant * fem->getNumDof()));
			solver->setTolerance(tolerance);
		}

		m_physicsManager->executeAdditions(fem);
	}

	void performTimingTest()
	{
		SurgSim::Framework::Timer totalTime;
		totalTime.beginFrame();

		SurgSim::Framework::Timer timer;
		timer.setMaxNumberOfFrames(frameCount);
		for (int i = 0; i < frameCount; i++)
		{
			timer.beginFrame();
			m_physicsManager->doUpdate(dt);
			timer.endFrame();
		}

		totalTime.endFrame();
		RecordProperty("Duration", boost::to_string(totalTime.getCumulativeTime()));
		RecordProperty("FrameRate", boost::to_string(timer.getAverageFrameRate()));
	}

protected:
	std::shared_ptr<SurgSim::Testing::MockPhysicsManager> m_physicsManager;
};

class IntegrationSchemeParamTest : public Fem3DPerformanceTestBase,
	public ::testing::WithParamInterface<std::tuple<SurgSim::Math::IntegrationScheme, SurgSim::Math::LinearSolver>>
{
};

class IntegrationSchemeAndCountParamTest
	: public Fem3DPerformanceTestBase,
	  public ::testing::WithParamInterface<std::tuple<SurgSim::Math::IntegrationScheme,
	  SurgSim::Math::LinearSolver, int>>
{
};

TEST_P(IntegrationSchemeParamTest, WoundTest)
{
	SurgSim::Math::IntegrationScheme integrationScheme;
	SurgSim::Math::LinearSolver linearSolver;
	std::tie(integrationScheme, linearSolver) = GetParam();
	RecordProperty("IntegrationScheme", IntegrationSchemeNames[integrationScheme]);
	RecordProperty("LinearSolver", LinearSolverNames[linearSolver]);

	auto fem = std::make_shared<SurgSim::Physics::MockFem3DRepresentation>("wound");
	fem->setFilename("Data/Fem3DPerformanceTest/wound_deformable.ply");
	fem->setIntegrationScheme(integrationScheme);
	fem->setLinearSolver(linearSolver);

	initializeRepresentation(fem);
	performTimingTest();
}

TEST_P(IntegrationSchemeAndCountParamTest, CubeTest)
{
	int numCubes;
	SurgSim::Math::IntegrationScheme integrationScheme;
	SurgSim::Math::LinearSolver linearSolver;
	std::tie(integrationScheme, linearSolver, numCubes) = GetParam();
	RecordProperty("IntegrationScheme", IntegrationSchemeNames[integrationScheme]);
	RecordProperty("CubeDivisions", boost::to_string(numCubes));
	RecordProperty("LinearSolver", LinearSolverNames[linearSolver]);

	auto fem = std::make_shared<DivisibleCubeRepresentation>("cube", numCubes);

	// We need to add some boundary conditions for the static solver to not run into a singular matrix
	std::const_pointer_cast<SurgSim::Math::OdeState>(fem->getInitialState())->addBoundaryCondition(0);
	std::const_pointer_cast<SurgSim::Math::OdeState>(fem->getInitialState())->addBoundaryCondition(1);
	std::const_pointer_cast<SurgSim::Math::OdeState>(fem->getInitialState())->addBoundaryCondition(2);

	fem->setIntegrationScheme(integrationScheme);
	fem->setLinearSolver(linearSolver);

	initializeRepresentation(fem);
	performTimingTest();
}

INSTANTIATE_TEST_CASE_P(Fem3DPerformanceTest,
						IntegrationSchemeParamTest,
						::testing::Combine(::testing::Values(SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER,
								SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EXPLICIT_EULER,
								SurgSim::Math::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER,
								SurgSim::Math::INTEGRATIONSCHEME_LINEAR_MODIFIED_EXPLICIT_EULER,
								SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER,
								SurgSim::Math::INTEGRATIONSCHEME_LINEAR_IMPLICIT_EULER,
								SurgSim::Math::INTEGRATIONSCHEME_STATIC,
								SurgSim::Math::INTEGRATIONSCHEME_LINEAR_STATIC,
								SurgSim::Math::INTEGRATIONSCHEME_RUNGE_KUTTA_4,
								SurgSim::Math::INTEGRATIONSCHEME_LINEAR_RUNGE_KUTTA_4),
								::testing::Values(SurgSim::Math::LINEARSOLVER_LU,
										SurgSim::Math::LINEARSOLVER_CONJUGATEGRADIENT)));

INSTANTIATE_TEST_CASE_P(
	Fem3DPerformanceTest,
	IntegrationSchemeAndCountParamTest,
	::testing::Combine(::testing::Values(SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER,
					   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EXPLICIT_EULER,
					   SurgSim::Math::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER,
					   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_MODIFIED_EXPLICIT_EULER,
					   SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER,
					   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_IMPLICIT_EULER,
					   SurgSim::Math::INTEGRATIONSCHEME_STATIC,
					   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_STATIC,
					   SurgSim::Math::INTEGRATIONSCHEME_RUNGE_KUTTA_4,
					   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_RUNGE_KUTTA_4),
					   ::testing::Values(SurgSim::Math::LINEARSOLVER_LU,
							   SurgSim::Math::LINEARSOLVER_CONJUGATEGRADIENT),
					   ::testing::Values(2, 3, 4, 5, 6, 7, 8)));

} // namespace Physics
} // namespace SurgSim
