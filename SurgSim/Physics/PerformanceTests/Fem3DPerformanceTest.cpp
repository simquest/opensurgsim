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
#include "SurgSim/Testing/MockPhysicsManager.h"

#include "viennacl/ocl/device.hpp"
#include "viennacl/ocl/platform.hpp"
#include "viennacl/ocl/backend.hpp"


using SurgSim::Math::Vector3d;

namespace
{
static const double dt = 0.001;
static const double maxIterationConstant = 0.1;
static const double tolerance = 1.0e-03;
static const int frameCount = 200;

static std::unordered_map<SurgSim::Math::IntegrationScheme, std::string, std::hash<int>> getIntegrationSchemeNames()
{
	std::unordered_map<SurgSim::Math::IntegrationScheme, std::string, std::hash<int>> result;

#define FEM3DPERFORMANCETEST_MAP_NAME(map, name) (map)[name] = #name
	FEM3DPERFORMANCETEST_MAP_NAME(result, SurgSim::Math::INTEGRATIONSCHEME_EULER_EXPLICIT);
	FEM3DPERFORMANCETEST_MAP_NAME(result, SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_EXPLICIT);
	FEM3DPERFORMANCETEST_MAP_NAME(result, SurgSim::Math::INTEGRATIONSCHEME_EULER_EXPLICIT_MODIFIED);
	FEM3DPERFORMANCETEST_MAP_NAME(result, SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_EXPLICIT_MODIFIED);
	FEM3DPERFORMANCETEST_MAP_NAME(result, SurgSim::Math::INTEGRATIONSCHEME_EULER_IMPLICIT);
	FEM3DPERFORMANCETEST_MAP_NAME(result, SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_IMPLICIT);
	FEM3DPERFORMANCETEST_MAP_NAME(result, SurgSim::Math::INTEGRATIONSCHEME_STATIC);
	FEM3DPERFORMANCETEST_MAP_NAME(result, SurgSim::Math::INTEGRATIONSCHEME_LINEAR_STATIC);
	FEM3DPERFORMANCETEST_MAP_NAME(result, SurgSim::Math::INTEGRATIONSCHEME_RUNGE_KUTTA_4);
	FEM3DPERFORMANCETEST_MAP_NAME(result, SurgSim::Math::INTEGRATIONSCHEME_LINEAR_RUNGE_KUTTA_4);
	FEM3DPERFORMANCETEST_MAP_NAME(result, SurgSim::Math::INTEGRATIONSCHEME_EULER_IMPLICIT_OPENCL);
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

static void initViennaCLContexts()
{
	// Picks the first device with the appropriate type to be used
	// use viennacl::ocl::switch_context(0); for the CPU and
	// viennacl::ocl::switch_context(1); for the GPU
	static bool isInitialized = false;
	if (isInitialized)
	{
		return;
	}

	int cpuContext = -1;
	int gpuContext = -1;
	size_t context = 0;
	auto platforms = viennacl::ocl::get_platforms();
	for (auto& platform : platforms)
	{
		for (auto& device : platform.devices())
		{
			if ((CL_DEVICE_TYPE_CPU & device.type()) != 0 && cpuContext < 0)
			{
				viennacl::ocl::setup_context(0, device);
				std::cout << "Using CPU:  ----------------\n";
				std::cout << device.info();
				cpuContext = 0;
			}

			if ((CL_DEVICE_TYPE_GPU & device.type()) != 0 && gpuContext < 0)
			{
				viennacl::ocl::setup_context(1, device);
				std::cout << "Using GPU:  ----------------\n";
				std::cout << device.info();
				gpuContext = 1;
			}
		}
	}

	isInitialized = true;
}
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
		initViennaCLContexts();
		viennacl::ocl::switch_context(0);

		m_physicsManager = std::make_shared<SurgSim::Testing::MockPhysicsManager>();

		m_physicsManager->doInitialize();
		m_physicsManager->doStartUp();

		m_runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	}

	void initializeRepresentation(std::shared_ptr<Fem3DRepresentation> fem)
	{
		fem->initialize(m_runtime);
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
		fem->initialize(std::make_shared<SurgSim::Framework::Runtime>("config.txt"));
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
		std::cout << "Test Start\n";
		SurgSim::Framework::Timer totalTime;
		totalTime.beginFrame();

		SurgSim::Framework::Timer timer;
		m_physicsManager->doUpdate(dt);
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
	std::shared_ptr<SurgSim::Framework::Runtime> m_runtime;
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

	auto fem = std::make_shared<SurgSim::Physics::Fem3DRepresentation>("wound");
	fem->loadFem("Geometry/wound_deformable.ply");
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
/*
INSTANTIATE_TEST_CASE_P(Fem3DPerformanceTest,
						IntegrationSchemeParamTest,
						::testing::Combine(::testing::Values(SurgSim::Math::INTEGRATIONSCHEME_EULER_EXPLICIT,
								SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_EXPLICIT,
								SurgSim::Math::INTEGRATIONSCHEME_EULER_EXPLICIT_MODIFIED,
								SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_EXPLICIT_MODIFIED,
								SurgSim::Math::INTEGRATIONSCHEME_EULER_IMPLICIT,
								SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_IMPLICIT,
								SurgSim::Math::INTEGRATIONSCHEME_STATIC,
								SurgSim::Math::INTEGRATIONSCHEME_LINEAR_STATIC,
								SurgSim::Math::INTEGRATIONSCHEME_RUNGE_KUTTA_4,
								SurgSim::Math::INTEGRATIONSCHEME_LINEAR_RUNGE_KUTTA_4,
								SurgSim::Math::INTEGRATIONSCHEME_EULER_IMPLICIT_OPENCL),
								::testing::Values(SurgSim::Math::LINEARSOLVER_LU,
										SurgSim::Math::LINEARSOLVER_CONJUGATEGRADIENT)));
*/
INSTANTIATE_TEST_CASE_P(
	Fem3DPerformanceTest,
	IntegrationSchemeAndCountParamTest,
	::testing::Combine(::testing::Values(
						   //SurgSim::Math::INTEGRATIONSCHEME_EULER_EXPLICIT,
						   //SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_EXPLICIT,
						   //SurgSim::Math::INTEGRATIONSCHEME_EULER_EXPLICIT_MODIFIED,
						   //SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_EXPLICIT_MODIFIED,
						   //SurgSim::Math::INTEGRATIONSCHEME_EULER_IMPLICIT,
						   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_IMPLICIT
						   //SurgSim::Math::INTEGRATIONSCHEME_STATIC,
						   //SurgSim::Math::INTEGRATIONSCHEME_LINEAR_STATIC,
						   //SurgSim::Math::INTEGRATIONSCHEME_RUNGE_KUTTA_4,
						   //SurgSim::Math::INTEGRATIONSCHEME_LINEAR_RUNGE_KUTTA_4,
						   //SurgSim::Math::INTEGRATIONSCHEME_EULER_IMPLICIT_OPENCL
					   ),
					   ::testing::Values(SurgSim::Math::LINEARSOLVER_LU,
							   SurgSim::Math::LINEARSOLVER_CONJUGATEGRADIENT,
							   SurgSim::Math::LINEARSOLVER_CONJUGATEGRADIENT_OPENCL
										),
					   ::testing::Values(8, 12, 14)));


INSTANTIATE_TEST_CASE_P(
	SingleTest,
	IntegrationSchemeAndCountParamTest,
	::testing::Combine(::testing::Values(SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_IMPLICIT),
					   ::testing::Values(SurgSim::Math::LINEARSOLVER_CONJUGATEGRADIENT_OPENCL),
					   ::testing::Values(14)));


} // namespace Physics
} // namespace SurgSim
