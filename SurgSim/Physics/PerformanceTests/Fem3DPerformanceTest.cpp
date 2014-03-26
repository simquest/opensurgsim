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

#include <memory>

#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Timer.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentationPlyReaderDelegate.h"
#include "SurgSim/Physics/FemElement3DCube.h"
#include "SurgSim/Testing/PhysicsManager.h"

using SurgSim::Math::Vector3d;

namespace
{
static const double dt = 0.001;
static const double timeoutHz = 20;
static const int frameCount = 100;
static const double timeoutDuration = static_cast<double>(frameCount) / timeoutHz;
}

static std::shared_ptr<SurgSim::Physics::Fem3DRepresentation> loadFem(
	const std::string& fileName,
	SurgSim::Math::IntegrationScheme integrationScheme,
	double massDensity,
	double poissonRatio,
	double youngModulus)
{
	// The PlyReader and Fem3DRepresentationPlyReaderDelegate work together to load 3d fems.
	SurgSim::DataStructures::PlyReader reader(fileName);
	std::shared_ptr<SurgSim::Physics::Fem3DRepresentationPlyReaderDelegate> fem3dDelegate
		= std::make_shared<SurgSim::Physics::Fem3DRepresentationPlyReaderDelegate>();

	SURGSIM_ASSERT(reader.setDelegate(fem3dDelegate)) << "The input file " << fileName << " is malformed.";
	reader.parseFile();

	std::shared_ptr<SurgSim::Physics::Fem3DRepresentation> fem = fem3dDelegate->getFem();

	// The FEM requires the implicit Euler integration scheme to avoid "blowing up"
	fem->setIntegrationScheme(integrationScheme);

	// Physical parameters must be set for the finite elements in order to be valid for the simulation.
	for (size_t i = 0; i < fem->getNumFemElements(); i++)
	{
		fem->getFemElement(i)->setMassDensity(massDensity);
		fem->getFemElement(i)->setPoissonRatio(poissonRatio);
		fem->getFemElement(i)->setYoungModulus(youngModulus);
	}

	return fem;
}

namespace SurgSim
{
namespace Physics
{

class DivisbleCubeRepresentation : public Fem3DRepresentation
{
public:
	/// Constructor
	/// \param name	The name of the truth cube representation.
	/// \param corners The 8 corners of the truth cube
	DivisbleCubeRepresentation(const std::string& name, unsigned int nodesPerAxis)
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

		auto initialState = std::make_shared<DeformableRepresentationState>();
		fillUpDeformableState(initialState);
		setInitialState(initialState);
		addFemCubes(initialState);
	}

protected:
	/// Convert an node index from a 3d indexing to a 1d indexing
	/// \param i, j, k Indices along the X, Y and Z axis
	/// \return Unique index of the corresponding point (to access a linear array for example)
	unsigned int get1DIndexFrom3D(unsigned int i, unsigned int j, unsigned int k)
	{
		return m_numNodesPerAxis * m_numNodesPerAxis * i + m_numNodesPerAxis * j + k;
	}

	/// Fills up a given deformable state with the truth cube nodes
	/// border nodes and internal nodes (i.e. the beads)
	/// \param[in,out] state	The deformable state to be filled up
	void fillUpDeformableState(std::shared_ptr<DeformableRepresentationState> state)
	{
		state->setNumDof(getNumDofPerNode(), m_numNodesPerAxis * m_numNodesPerAxis * m_numNodesPerAxis);
		SurgSim::Math::Vector& nodePositions = state->getPositions();

		for (unsigned int i = 0; i < m_numNodesPerAxis; i++)
		{
			// For a given index i, we intersect the cube with a (Y Z) plane, which defines a square on a (Y Z) plane
			Vector3d extremitiesX0[4] = {m_cubeNodes[0], m_cubeNodes[2], m_cubeNodes[4], m_cubeNodes[6]};
			Vector3d extremitiesX1[4] = {m_cubeNodes[1], m_cubeNodes[3], m_cubeNodes[5], m_cubeNodes[7]};
			Vector3d extremitiesXi[4];
			double coefI = static_cast<double>(i) / (static_cast<double>(m_numNodesPerAxis) - 1.0);

			for (int index = 0; index < 4; index++)
			{
				extremitiesXi[index] =
					extremitiesX0[index] * (1.0 - coefI) +
					extremitiesX1[index] *        coefI;
			}

			for (unsigned int j = 0; j < m_numNodesPerAxis; j++)
			{
				// For a given index j, we intersect the square with a (X Z) plane, which defines a line along (Z)
				Vector3d extremitiesY0[2] = {extremitiesXi[0], extremitiesXi[2]};
				Vector3d extremitiesY1[2] = {extremitiesXi[1], extremitiesXi[3]};
				Vector3d extremitiesYi[2];
				double coefJ = static_cast<double>(j) / (static_cast<double>(m_numNodesPerAxis) - 1.0);

				for (int index = 0; index < 2; index++)
				{
					extremitiesYi[index] =
						extremitiesY0[index] * (1.0 - coefJ) +
						extremitiesY1[index] *        coefJ;
				}

				for (unsigned int k = 0; k < m_numNodesPerAxis; k++)
				{
					// For a given index k, we intersect the line with a (X Y) plane, which defines a 3d point
					double coefK = static_cast<double>(k) / (static_cast<double>(m_numNodesPerAxis) - 1.0);
					Vector3d position3d = extremitiesYi[0] * (1.0 - coefK) + extremitiesYi[1] * coefK;
					SurgSim::Math::setSubVector(position3d, get1DIndexFrom3D(i, j, k), 3, &nodePositions);
				}
			}
		}
	}

	/// Adds the Fem3D elements of small cubes
	/// \param state	The deformable state for initialization.
	void addFemCubes(std::shared_ptr<DeformableRepresentationState> state)
	{
		for (unsigned int i = 0; i < m_numNodesPerAxis - 1; i++)
		{
			for (unsigned int j = 0; j < m_numNodesPerAxis - 1; j++)
			{
				for (unsigned int k = 0; k < m_numNodesPerAxis - 1; k++)
				{
					std::array<unsigned int, 8> cubeNodeIds;
					cubeNodeIds[0] = get1DIndexFrom3D(i  , j  , k  );
					cubeNodeIds[1] = get1DIndexFrom3D(i+1, j  , k  );
					cubeNodeIds[2] = get1DIndexFrom3D(i  , j+1, k  );
					cubeNodeIds[3] = get1DIndexFrom3D(i+1, j+1, k  );
					cubeNodeIds[4] = get1DIndexFrom3D(i  , j  , k+1);
					cubeNodeIds[5] = get1DIndexFrom3D(i+1, j  , k+1);
					cubeNodeIds[6] = get1DIndexFrom3D(i  , j+1, k+1);
					cubeNodeIds[7] = get1DIndexFrom3D(i+1, j+1, k+1);

					std::array<unsigned int, 8> cube = {
						cubeNodeIds[0], cubeNodeIds[1], cubeNodeIds[3], cubeNodeIds[2],
						cubeNodeIds[4], cubeNodeIds[5], cubeNodeIds[7], cubeNodeIds[6]};

					// Add FemElement3DCube for each cube
					std::shared_ptr<FemElement3DCube> femElement = std::make_shared<FemElement3DCube>(cube, *state);
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
	unsigned int m_numNodesPerAxis;

	// Nodes of the original truth cube
	std::array<SurgSim::Math::Vector3d, 8> m_cubeNodes;
};

class Fem3DPerformanceTest : public ::testing::Test
{
public:
	virtual void SetUp()
	{
		m_physicsManager = std::make_shared<SurgSim::Testing::PhysicsManager>();

		m_physicsManager->doInitialize();
		m_physicsManager->doStartUp();
	}

	void initializeRepresentation(std::shared_ptr<Fem3DRepresentation> fem)
	{
		fem->initialize(std::make_shared<SurgSim::Framework::Runtime>());
		m_physicsManager->executeAdditions(fem);
	}

	void performTimingTest()
	{
		SurgSim::Framework::Timer timer;
		for (int i = 0; i < frameCount; i++)
		{
			timer.beginFrame();
			m_physicsManager->doUpdate(dt);
			timer.endFrame();

			RecordProperty("FrameRate", boost::to_string(timer.getAverageFrameRate()));
			if (timer.getCumulativeTime() > timeoutDuration)
			{
				return;
			}
		}
	}

protected:
	std::shared_ptr<SurgSim::Testing::PhysicsManager> m_physicsManager;
};

TEST_F(Fem3DPerformanceTest, WoundTest)
{
	auto fem = loadFem("Data/Fem3DPerformanceTest/wound_deformable.ply",
					   SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER,
					   1000.0,
					   0.45,
					   75e3);

	initializeRepresentation(fem);
	performTimingTest();
}

TEST_F(Fem3DPerformanceTest, Cube2Test)
{
	static const int numCubes = 2;
	auto fem = std::make_shared<DivisbleCubeRepresentation>("cube", numCubes);
	fem->setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER);

	initializeRepresentation(fem);
	performTimingTest();
}

TEST_F(Fem3DPerformanceTest, Cube3Test)
{
	static const int numCubes = 3;
	auto fem = std::make_shared<DivisbleCubeRepresentation>("cube", numCubes);
	fem->setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER);

	initializeRepresentation(fem);
	performTimingTest();
}

TEST_F(Fem3DPerformanceTest, Cube4Test)
{
	static const int numCubes = 4;
	auto fem = std::make_shared<DivisbleCubeRepresentation>("cube", numCubes);
	fem->setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER);

	initializeRepresentation(fem);
	performTimingTest();
}

TEST_F(Fem3DPerformanceTest, Cube5Test)
{
	static const int numCubes = 5;
	auto fem = std::make_shared<DivisbleCubeRepresentation>("cube", numCubes);
	fem->setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER);

	initializeRepresentation(fem);
	performTimingTest();
}

TEST_F(Fem3DPerformanceTest, Cube6Test)
{
	static const int numCubes = 6;
	auto fem = std::make_shared<DivisbleCubeRepresentation>("cube", numCubes);
	fem->setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER);

	initializeRepresentation(fem);
	performTimingTest();
}

TEST_F(Fem3DPerformanceTest, Cube7Test)
{
	static const int numCubes = 7;
	auto fem = std::make_shared<DivisbleCubeRepresentation>("cube", numCubes);
	fem->setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER);

	initializeRepresentation(fem);
	performTimingTest();
}

TEST_F(Fem3DPerformanceTest, Cube8Test)
{
	static const int numCubes = 8;
	auto fem = std::make_shared<DivisbleCubeRepresentation>("cube", numCubes);
	fem->setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER);

	initializeRepresentation(fem);
	performTimingTest();
}

TEST_F(Fem3DPerformanceTest, Cube9Test)
{
	static const int numCubes = 9;
	auto fem = std::make_shared<DivisbleCubeRepresentation>("cube", numCubes);
	fem->setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER);

	initializeRepresentation(fem);
	performTimingTest();
}

TEST_F(Fem3DPerformanceTest, Cube10Test)
{
	static const int numCubes = 10;
	auto fem = std::make_shared<DivisbleCubeRepresentation>("cube", numCubes);
	fem->setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER);

	initializeRepresentation(fem);
	performTimingTest();
}

TEST_F(Fem3DPerformanceTest, Cube11Test)
{
	static const int numCubes = 11;
	auto fem = std::make_shared<DivisbleCubeRepresentation>("cube", numCubes);
	fem->setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER);

	initializeRepresentation(fem);
	performTimingTest();
}

TEST_F(Fem3DPerformanceTest, Cube12Test)
{
	static const int numCubes = 12;
	auto fem = std::make_shared<DivisbleCubeRepresentation>("cube", numCubes);
	fem->setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER);

	initializeRepresentation(fem);
	performTimingTest();
}

} // namespace Physics
} // namespace SurgSim
