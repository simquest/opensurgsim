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
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentationPlyReaderDelegate.h"
#include "SurgSim/Physics/Fem3DElementCube.h"
#include "SurgSim/Testing/MockPhysicsManager.h"

using SurgSim::Math::Vector3d;

namespace
{
static const double dt = 0.001;
static const int frameCount = 100;

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

static std::unordered_map<SurgSim::Math::IntegrationScheme, std::string, std::hash<int>> IntegrationSchemeNames
	= getIntegrationSchemeNames();
}

namespace SurgSim
{
namespace Physics
{

class DivisbleCubeRepresentation : public Fem3DRepresentation
{
public:
	/// Constructor
	/// \param name	The name of the divisible cube representation.
	/// \param nodesPerAxis	The number of nodes per axis
	DivisbleCubeRepresentation(const std::string& name, size_t nodesPerAxis)
		: Fem3DRepresentation(name), m_numNodesPerAxis(nodesPerAxis)
	{
		// Compute the center point of the cube
		SurgSim::Math::Vector3d center = SurgSim::Math::Vector3d::Zero();

		// Compute the cube's corners for the Fem3d simulation
		double halfLength = static_cast<double>(nodesPerAxis);
		Vector3d X = Vector3d::UnitX();
		Vector3d Y = Vector3d::UnitY();
		Vector3d Z = Vector3d::UnitZ();
		m_cubeNodes[0] = center - halfLength * X - halfLength * Y - halfLength * Z;
		m_cubeNodes[1] = center + halfLength * X - halfLength * Y - halfLength * Z;
		m_cubeNodes[2] = center - halfLength * X + halfLength * Y - halfLength * Z;
		m_cubeNodes[3] = center + halfLength * X + halfLength * Y - halfLength * Z;
		m_cubeNodes[4] = center - halfLength * X - halfLength * Y + halfLength * Z;
		m_cubeNodes[5] = center + halfLength * X - halfLength * Y + halfLength * Z;
		m_cubeNodes[6] = center - halfLength * X + halfLength * Y + halfLength * Z;
		m_cubeNodes[7] = center + halfLength * X + halfLength * Y + halfLength * Z;

		auto initialState = std::make_shared<SurgSim::Math::OdeState>();
		fillUpDeformableState(initialState);
		setInitialState(initialState);
		addFemCubes(initialState);
	}

protected:
	/// Convert a node index from a 3d indexing to a 1d indexing
	/// \param i, j, k Indices along the X, Y and Z axis
	/// \return Unique index of the corresponding point (to access a linear array for example)
	size_t get1DIndexFrom3D(size_t i, size_t j, size_t k)
	{
		return m_numNodesPerAxis * m_numNodesPerAxis * i + m_numNodesPerAxis * j + k;
	}

	/// Fills up a given state with the cube's nodes, border nodes, and internal nodes
	/// \param[in,out] state	The state to be filled up
	void fillUpDeformableState(std::shared_ptr<SurgSim::Math::OdeState> state)
	{
		state->setNumDof(getNumDofPerNode(), m_numNodesPerAxis * m_numNodesPerAxis * m_numNodesPerAxis);
		SurgSim::Math::Vector& nodePositions = state->getPositions();

		for (size_t i = 0; i < m_numNodesPerAxis; i++)
		{
			// For a given index i, we intersect the cube with a (Y Z) plane, which defines a square on a (Y Z) plane
			Vector3d extremitiesX0[4] = {m_cubeNodes[0], m_cubeNodes[2], m_cubeNodes[4], m_cubeNodes[6]};
			Vector3d extremitiesX1[4] = {m_cubeNodes[1], m_cubeNodes[3], m_cubeNodes[5], m_cubeNodes[7]};
			Vector3d extremitiesXi[4];
			double coefI = static_cast<double>(i) / (static_cast<double>(m_numNodesPerAxis) - 1.0);

			for (size_t index = 0; index < 4; index++)
			{
				extremitiesXi[index] =
					extremitiesX0[index] * (1.0 - coefI) +
					extremitiesX1[index] *        coefI;
			}

			for (size_t j = 0; j < m_numNodesPerAxis; j++)
			{
				// For a given index j, we intersect the square with a (X Z) plane, which defines a line along (Z)
				Vector3d extremitiesY0[2] = {extremitiesXi[0], extremitiesXi[2]};
				Vector3d extremitiesY1[2] = {extremitiesXi[1], extremitiesXi[3]};
				Vector3d extremitiesYi[2];
				double coefJ = static_cast<double>(j) / (static_cast<double>(m_numNodesPerAxis) - 1.0);

				for (size_t index = 0; index < 2; index++)
				{
					extremitiesYi[index] =
						extremitiesY0[index] * (1.0 - coefJ) +
						extremitiesY1[index] *        coefJ;
				}

				for (size_t k = 0; k < m_numNodesPerAxis; k++)
				{
					// For a given index k, we intersect the line with a (X Y) plane, which defines a 3d point
					double coefK = static_cast<double>(k) / (static_cast<double>(m_numNodesPerAxis) - 1.0);
					Vector3d position3d = extremitiesYi[0] * (1.0 - coefK) + extremitiesYi[1] * coefK;
					SurgSim::Math::setSubVector(
						position3d, static_cast<unsigned int>(get1DIndexFrom3D(i, j, k)), 3, &nodePositions);
				}
			}
		}
	}

	/// Adds the Fem3D elements of small cubes
	/// \param state	The state for initialization.
	void addFemCubes(std::shared_ptr<SurgSim::Math::OdeState> state)
	{
		for (size_t i = 0; i < m_numNodesPerAxis - 1; i++)
		{
			for (size_t j = 0; j < m_numNodesPerAxis - 1; j++)
			{
				for (size_t k = 0; k < m_numNodesPerAxis - 1; k++)
				{
					std::array<unsigned int, 8> cubeNodeIds;
					cubeNodeIds[0] = static_cast<unsigned int>(get1DIndexFrom3D(i  , j  , k  ));
					cubeNodeIds[1] = static_cast<unsigned int>(get1DIndexFrom3D(i+1, j  , k  ));
					cubeNodeIds[2] = static_cast<unsigned int>(get1DIndexFrom3D(i  , j+1, k  ));
					cubeNodeIds[3] = static_cast<unsigned int>(get1DIndexFrom3D(i+1, j+1, k  ));
					cubeNodeIds[4] = static_cast<unsigned int>(get1DIndexFrom3D(i  , j  , k+1));
					cubeNodeIds[5] = static_cast<unsigned int>(get1DIndexFrom3D(i+1, j  , k+1));
					cubeNodeIds[6] = static_cast<unsigned int>(get1DIndexFrom3D(i  , j+1, k+1));
					cubeNodeIds[7] = static_cast<unsigned int>(get1DIndexFrom3D(i+1, j+1, k+1));

					std::array<unsigned int, 8> cube = {
						cubeNodeIds[0], cubeNodeIds[1], cubeNodeIds[3], cubeNodeIds[2],
						cubeNodeIds[4], cubeNodeIds[5], cubeNodeIds[7], cubeNodeIds[6]};

					// Add Fem3DElementCube for each cube
					std::shared_ptr<Fem3DElementCube> femElement = std::make_shared<Fem3DElementCube>(cube, *state);
					femElement->setMassDensity(980.0);   // 0.98 g/cm^-3 (2-part silicone rubber a.k.a. RTV6166)
					femElement->setPoissonRatio(0.499);  // From the paper (near 0.5)
					femElement->setYoungModulus(15.3e3); // 15.3 kPa (From the paper)
					femElement->initialize(*state);
					addFemElement(femElement);
				}
			}
		}
	}

private:
	// Number of point per dimensions
	size_t m_numNodesPerAxis;

	// Corner nodes of the original cube
	std::array<SurgSim::Math::Vector3d, 8> m_cubeNodes;
};

class Fem3DPerformanceTestBase : public ::testing::Test
{
public:
	virtual void SetUp()
	{
		m_physicsManager = std::make_shared<SurgSim::Testing::MockPhysicsManager>();

		m_physicsManager->doInitialize();
		m_physicsManager->doStartUp();
	}

	void initializeRepresentation(std::shared_ptr<Fem3DRepresentation> fem)
	{
		fem->initialize(std::make_shared<SurgSim::Framework::Runtime>());
		fem->wakeUp();
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
								   public ::testing::WithParamInterface<SurgSim::Math::IntegrationScheme>
{
};

class IntegrationSchemeAndCountParamTest
	: public Fem3DPerformanceTestBase,
	  public ::testing::WithParamInterface<std::tuple<SurgSim::Math::IntegrationScheme, int> >
{
};

TEST_P(IntegrationSchemeParamTest, WoundTest)
{
	SurgSim::Math::IntegrationScheme integrationScheme = GetParam();
	RecordProperty("IntegrationScheme", IntegrationSchemeNames[integrationScheme]);

	auto fem = std::make_shared<SurgSim::Physics::Fem3DRepresentation>("wound");
	fem->setFilename("Data/Fem3DPerformanceTest/wound_deformable.ply");
	fem->setIntegrationScheme(integrationScheme);

	initializeRepresentation(fem);
	performTimingTest();
}

TEST_P(IntegrationSchemeAndCountParamTest, CubeTest)
{
	int numCubes;
	SurgSim::Math::IntegrationScheme integrationScheme;
	std::tie(integrationScheme, numCubes) = GetParam();
	RecordProperty("IntegrationScheme", IntegrationSchemeNames[integrationScheme]);
	RecordProperty("CubeDivisions", boost::to_string(numCubes));

	auto fem = std::make_shared<DivisbleCubeRepresentation>("cube", numCubes);
	fem->setIntegrationScheme(integrationScheme);

	initializeRepresentation(fem);
	performTimingTest();
}

INSTANTIATE_TEST_CASE_P(Fem3DPerformanceTest,
						IntegrationSchemeParamTest,
						::testing::Values(SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER,
										  SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EXPLICIT_EULER,
										  SurgSim::Math::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER,
										  SurgSim::Math::INTEGRATIONSCHEME_LINEAR_MODIFIED_EXPLICIT_EULER,
										  SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER,
										  SurgSim::Math::INTEGRATIONSCHEME_LINEAR_IMPLICIT_EULER,
										  SurgSim::Math::INTEGRATIONSCHEME_STATIC,
										  SurgSim::Math::INTEGRATIONSCHEME_LINEAR_STATIC,
										  SurgSim::Math::INTEGRATIONSCHEME_RUNGE_KUTTA_4,
										  SurgSim::Math::INTEGRATIONSCHEME_LINEAR_RUNGE_KUTTA_4));

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
					   ::testing::Values(2, 3, 4, 5, 6, 7, 8)));

} // namespace Physics
} // namespace SurgSim
