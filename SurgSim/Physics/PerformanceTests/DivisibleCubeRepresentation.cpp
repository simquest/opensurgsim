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

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem3DElementCube.h"
#include "SurgSim/Physics/Fem3DElementCorotationalTetrahedron.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/PerformanceTests/DivisibleCubeRepresentation.h"

using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Physics
{

DivisibleCubeRepresentation::DivisibleCubeRepresentation(const std::string& name, size_t nodesPerAxis)
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

bool DivisibleCubeRepresentation::initializeNoWakeUp()
{
	return doInitialize();
}

std::shared_ptr<SurgSim::Math::OdeSolver> DivisibleCubeRepresentation::getOdeSolver()
{
	return m_odeSolver;
}

/// Convert a node index from a 3d indexing to a 1d indexing
/// \param i, j, k Indices along the X, Y and Z axis
/// \return Unique index of the corresponding point (to access a linear array for example)
size_t DivisibleCubeRepresentation::get1DIndexFrom3D(size_t i, size_t j, size_t k)
{
	return m_numNodesPerAxis * m_numNodesPerAxis * i + m_numNodesPerAxis * j + k;
}

/// Fills up a given state with the cube's nodes, border nodes, and internal nodes
/// \param[in,out] state	The state to be filled up
void DivisibleCubeRepresentation::fillUpDeformableState(std::shared_ptr<SurgSim::Math::OdeState> state)
{
	SURGSIM_ASSERT(state != nullptr);
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
				SurgSim::Math::setSubVector(position3d, get1DIndexFrom3D(i, j, k), 3, &nodePositions);
			}
		}
	}
}

/// Adds the Fem3D elements of small cubes
/// \param state	The state for initialization.
void DivisibleCubeRepresentation::addFemCubes(std::shared_ptr<SurgSim::Math::OdeState> state)
{
	for (size_t i = 0; i < m_numNodesPerAxis - 1; i++)
	{
		for (size_t j = 0; j < m_numNodesPerAxis - 1; j++)
		{
			for (size_t k = 0; k < m_numNodesPerAxis - 1; k++)
			{

				std::array<size_t, 8> cubeNodeIds;
				cubeNodeIds[0] = get1DIndexFrom3D(i  , j  , k);
				cubeNodeIds[1] = get1DIndexFrom3D(i + 1, j  , k);
				cubeNodeIds[2] = get1DIndexFrom3D(i  , j + 1, k);
				cubeNodeIds[3] = get1DIndexFrom3D(i + 1, j + 1, k);
				cubeNodeIds[4] = get1DIndexFrom3D(i  , j  , k + 1);
				cubeNodeIds[5] = get1DIndexFrom3D(i + 1, j  , k + 1);
				cubeNodeIds[6] = get1DIndexFrom3D(i  , j + 1, k + 1);
				cubeNodeIds[7] = get1DIndexFrom3D(i + 1, j + 1, k + 1);

				std::array<size_t, 8> cube = {cubeNodeIds[0], cubeNodeIds[1], cubeNodeIds[3], cubeNodeIds[2],
											  cubeNodeIds[4], cubeNodeIds[5], cubeNodeIds[7], cubeNodeIds[6]
											 };

				//// Add Fem3DElementCube for each cube
				//std::shared_ptr<Fem3DElementCube> femElement = std::make_shared<Fem3DElementCube>(cube);
				//femElement->setMassDensity(980.0);   // 0.98 g/cm^-3 (2-part silicone rubber a.k.a. RTV6166)
				//femElement->setPoissonRatio(0.499);  // From the paper (near 0.5)
				//femElement->setYoungModulus(15.3e3); // 15.3 kPa (From the paper)
				//femElement->initialize(*state);
				//addFemElement(femElement);

				// Cube decomposition into 5 tetrahedrons
				// https://www.math.ucdavis.edu/~deloera/CURRENT_INTERESTS/cube.html
				std::array< std::array<size_t, 4>, 5> tetrahedrons = { {
					{ { cubeNodeIds[4], cubeNodeIds[7], cubeNodeIds[1], cubeNodeIds[2] } }, // CCW (47)cross(41) . (42) > 0
					{ { cubeNodeIds[4], cubeNodeIds[1], cubeNodeIds[7], cubeNodeIds[5] } }, // CCW (41)cross(47) . (45) > 0
					{ { cubeNodeIds[4], cubeNodeIds[2], cubeNodeIds[1], cubeNodeIds[0] } }, // CCW (42)cross(41) . (40) > 0
					{ { cubeNodeIds[4], cubeNodeIds[7], cubeNodeIds[2], cubeNodeIds[6] } }, // CCW (47)cross(42) . (46) > 0
					{ { cubeNodeIds[1], cubeNodeIds[2], cubeNodeIds[7], cubeNodeIds[3] } }  // CCW (12)cross(17) . (13) > 0
					}
				};

				for (int tetId = 0; tetId < 5; tetId++)
				{
					std::shared_ptr<Fem3DElementCorotationalTetrahedron> femElement = std::make_shared<Fem3DElementCorotationalTetrahedron>(tetrahedrons[tetId]);
					femElement->setMassDensity(980.0);   // 0.98 g/cm^-3 (2-part silicone rubber a.k.a. RTV6166)
					femElement->setPoissonRatio(0.499);  // From the paper (near 0.5)
					femElement->setYoungModulus(15.3e3); // 15.3 kPa (From the paper)
					femElement->initialize(*state);
					addFemElement(femElement);
				}
			}
		}
	}
}

} // namespace Physics
} // namespace SurgSim
