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

#include <math.h>

#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Graphics/OsgAxesRepresentation.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/PointCloudRepresentation.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/Fem3DElementCube.h"
#include "SurgSim/Physics/PhysicsManager.h"
#include "SurgSim/Physics/RenderTests/RenderTest.h"

#include "SurgSim/Framework/Logger.h"

using SurgSim::DataStructures::Vertices;
using SurgSim::Framework::SceneElement;
using SurgSim::Graphics::PointCloud;
using SurgSim::Graphics::PointCloudRepresentation;
using SurgSim::Graphics::OsgAxesRepresentation;
using SurgSim::Graphics::OsgPointCloudRepresentation;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;
using SurgSim::Physics::PhysicsManager;

using SurgSim::Framework::Logger;

namespace SurgSim
{
namespace Physics
{

struct TruthCubeData
{
	// Beads positions in uncompressed material
	std::vector<Vector3d> cubeData0;

	// Beads positions under 5% strain
	std::vector<Vector3d> cubeData1;

	// Beads positions under 12.5% strain
	std::vector<Vector3d> cubeData2;

	// Beads positions under 18.25% strain
	std::vector<Vector3d> cubeData3;
};

/// Parsing Truth Cube data from an external file
/// \param truthCubeData a container of cube data for all strains
/// \return True if the Truth Cube Data is successful loaded, otherwise false
bool parseTruthCubeData(std::shared_ptr<TruthCubeData> truthCubeData)
{
	const double mm2m = 1000.0; // Conversion constant from millimeter to meter
	const int numCommentLine = 7; // Number of comment lines to skip before accessing the actual data

	// Position of uncompressed data, 5% strain, 12.5% strain, 18.25% strain
	std::array<Vector3d, 4> position;
	std::string line;
	char comma;
	int i, j, k;
	int numLine = 0;

	const SurgSim::Framework::ApplicationData data("config.txt");
	std::string filename = data.findFile("uniaxial_positions.csv");
	std::ifstream datafile(filename);

	if (! datafile.good())
	{
		SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) << "Could not get uniaxial_positions.csv data file";
		return false;
	}

	while (std::getline(datafile, line))
	{
		if (++numLine > numCommentLine)
		{
			std::stringstream strstream(line);
			strstream >> i >> comma >> j >> comma >> k >> comma
				>> position[0].x() >> comma >> position[0].y() >> comma >> position[0].z() >> comma
				>> position[1].x() >> comma >> position[1].y() >> comma >> position[1].z() >> comma
				>> position[2].x() >> comma >> position[2].y() >> comma >> position[2].z() >> comma
				>> position[3].x() >> comma >> position[3].y() >> comma >> position[3].z();

			// Store strains separately, in meter unit and Y up (rotation 90 alond X)
			// Apparently, -Z seems to be up in the truth cube data set
			Quaterniond::AngleAxisType aa(M_PI / 2.0, Vector3d::UnitX());
			RigidTransform3d pose(aa);
			truthCubeData->cubeData0.push_back(pose * (position[0] / mm2m));
			truthCubeData->cubeData1.push_back(pose * (position[1] / mm2m));
			truthCubeData->cubeData2.push_back(pose * (position[2] / mm2m));
			truthCubeData->cubeData3.push_back(pose * (position[3] / mm2m));
		}
	}
	return true;
};

/// Search the node in the state that is the closest to a given 3d point
/// This is necessary because the structure of the nodes in the state and in the truth cube is not necessarily matching
/// The state is built with a structure aligned on +X +Y +Z, while the truth cube data are defined along +X +Y -Z and
/// rotated (PI/2 along X) to match Y up, so 3d indices don't match.
size_t searchForClosestNodeInState(std::shared_ptr<SurgSim::Math::OdeState> state, Vector3d p)
{
	size_t res = -1;
	double minDistance = std::numeric_limits<double>::max();

	SurgSim::Math::Vector& x = state->getPositions();
	for (size_t nodeId = 0; nodeId < state->getNumNodes(); ++nodeId)
	{
		Vector3d diff(SurgSim::Math::getSubVector(x, nodeId, 3) - p);
		if (diff.norm() < minDistance)
		{
			minDistance = diff.norm();
			res = nodeId;
		}
	}
	return res;
}

/// Truth cube representation (extension of a Fem3DRepresentation)
/// Defines a subdivided initial cube with cube FemElements
class TruthCubeRepresentation : public Fem3DRepresentation
{
public:
	/// Constructor
	/// \param name	The name of the truth cube representation.
	/// \param corners The 8 corners of the truth cube
	TruthCubeRepresentation(const std::string& name, std::array<SurgSim::Math::Vector3d, 8> corners) :
		Fem3DRepresentation(name)
	{
		m_numNodesPerAxis = 9;
		m_cubeNodes = corners;
	}

	/// Convert an node index from a 3d indexing to a 1d indexing
	/// \param i, j, k Indices along the X, Y and Z axis
	/// \return Unique index of the corresponding point (to access a linear array for example)
	size_t get1DIndexFrom3D(size_t i, size_t j, size_t k)
	{
		return m_numNodesPerAxis * m_numNodesPerAxis * i + m_numNodesPerAxis * j + k;
	}

	/// Convert a node index from a 1d indexing to a 3d indexing
	/// \param index of the node
	/// \param[in,out] i, j, k Corresponding indices along the X, Y and Z axis
	/// \param numNodesPerAxis the number of nodes per axis to be considered in the conversion
	void get3DIndexFrom1D(size_t index, size_t* i, size_t* j, size_t* k, size_t numNodesPerAxis)
	{
		size_t remainingIndex = index;

		(*i) = remainingIndex / (numNodesPerAxis * numNodesPerAxis);
		remainingIndex -= (*i) * (numNodesPerAxis * numNodesPerAxis);

		(*j) = remainingIndex / numNodesPerAxis;
		remainingIndex -= (*j) * numNodesPerAxis;

		(*k) = remainingIndex;
	}

	/// Gets the number of nodes per axis
	/// \return The number of nodes per axis.
	size_t getNumNodesPerAxis()
	{
		return m_numNodesPerAxis;
	}

	/// Gets the boundary conditions
	/// \return vector of dof (degrees of freedom) indices representing all boundary conditions
	std::vector<size_t> getBoundaryConditions()
	{
		return m_boundaryConditions;
	}

	/// Gets the boundary conditions displacement values
	/// \return The vector of displacement for all boundary conditions
	std::vector<double> getBoundaryConditionsDisplacement()
	{
		return m_boundaryConditionsDisplacement;
	}

	/// Fills up a given state with the truth cube nodes
	/// border nodes and internal nodes (i.e. the beads)
	/// \param[in,out] state	The state to be filled up
	void fillUpDeformableState(std::shared_ptr<SurgSim::Math::OdeState> state)
	{
		state->setNumDof(getNumDofPerNode(), m_numNodesPerAxis * m_numNodesPerAxis * m_numNodesPerAxis);
		SurgSim::Math::Vector& nodePositions = state->getPositions();

		for (int i = 0; i < m_numNodesPerAxis; i++)
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

			for (int j = 0; j < m_numNodesPerAxis; j++)
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

				for (int k=0; k < m_numNodesPerAxis; k++)
				{
					// For a given index k, we intersect the line with a (X Y) plane, which defines a 3d point
					double coefK = static_cast<double>(k) / (static_cast<double>(m_numNodesPerAxis) - 1.0);
					Vector3d position3d = extremitiesYi[0] * (1.0 - coefK) + extremitiesYi[1] * coefK;
					SurgSim::Math::setSubVector(position3d, get1DIndexFrom3D(i, j, k), 3, &nodePositions);
				}
			}
		}
	}

	/// Adjust the internal nodes position of the state to match the beads initial position
	/// \param state The state to adjust
	/// \param beadsInitialPositions The vector of beads initial position
	/// \note The ordering between the state and the beads vector might be different
	/// \note The algorithm will not rely on a matching indexing
	void adjustInitialBeadsPosition(std::shared_ptr<SurgSim::Math::OdeState> state,
		std::vector<Vector3d> beadsInitialPositions)
	{
		SurgSim::Math::Vector& x = state->getPositions();

		for (size_t index = 0; index < beadsInitialPositions.size(); ++index)
		{
			size_t nodeId = searchForClosestNodeInState(state, beadsInitialPositions[index]);
			SurgSim::Math::setSubVector(beadsInitialPositions[index], nodeId, 3, &x);
		}
	}

	/// Defines the boundary conditions for the truth cube
	/// \param displacementForTopLayer	The displacement of the boundary conditions for the top layer
	/// \note The bottom layer is completely fixed (all nodes, all dof)
	/// \note The top layer is completely fixed and compressed along Y
	void defineBoundaryCondition(double displacementForTopLayer)
	{
		for (int i = 0; i < m_numNodesPerAxis; i++)
		{
			for (int k = 0; k < m_numNodesPerAxis; k++)
			{
				// Add boundary condition for bottom layer (j = 0)
				int nodeId = get1DIndexFrom3D(i, 0, k);
				m_boundaryConditions.push_back(nodeId * 3 + 0);
				m_boundaryConditions.push_back(nodeId * 3 + 1);
				m_boundaryConditions.push_back(nodeId * 3 + 2);
				m_boundaryConditionsDisplacement.push_back(0.0);
				m_boundaryConditionsDisplacement.push_back(0.0);
				m_boundaryConditionsDisplacement.push_back(0.0);

				// Add boundary condition for top layer (j = m_numNodesPerAxis - 1)
				nodeId = get1DIndexFrom3D(i, m_numNodesPerAxis - 1, k);
				m_boundaryConditions.push_back(nodeId * 3 + 0);
				m_boundaryConditions.push_back(nodeId * 3 + 1);
				m_boundaryConditions.push_back(nodeId * 3 + 2);
				m_boundaryConditionsDisplacement.push_back(0.0);
				m_boundaryConditionsDisplacement.push_back(displacementForTopLayer);
				m_boundaryConditionsDisplacement.push_back(0.0);
			}
		}
	}

	/// Adds the Fem3D elements of small cubes
	/// \param state	The state for initialization.
	void addFemCubes(std::shared_ptr<SurgSim::Math::OdeState> state)
	{
		for (size_t i = 0; i < static_cast<size_t>(m_numNodesPerAxis - 1); i++)
		{
			for (size_t j = 0; j < static_cast<size_t>(m_numNodesPerAxis - 1); j++)
			{
				for (size_t k = 0; k < static_cast<size_t>(m_numNodesPerAxis - 1); k++)
				{
					std::array<size_t, 8> cubeNodeIds;
					cubeNodeIds[0] = get1DIndexFrom3D(i  , j  , k  );
					cubeNodeIds[1] = get1DIndexFrom3D(i+1, j  , k  );
					cubeNodeIds[2] = get1DIndexFrom3D(i  , j+1, k  );
					cubeNodeIds[3] = get1DIndexFrom3D(i+1, j+1, k  );
					cubeNodeIds[4] = get1DIndexFrom3D(i  , j  , k+1);
					cubeNodeIds[5] = get1DIndexFrom3D(i+1, j  , k+1);
					cubeNodeIds[6] = get1DIndexFrom3D(i  , j+1, k+1);
					cubeNodeIds[7] = get1DIndexFrom3D(i+1, j+1, k+1);

					std::array<size_t, 8> cube = {cubeNodeIds[0], cubeNodeIds[1], cubeNodeIds[3], cubeNodeIds[2],
												  cubeNodeIds[4], cubeNodeIds[5], cubeNodeIds[7], cubeNodeIds[6]};

					// Add Fem3DElementCube for each cube
					std::shared_ptr<Fem3DElementCube> femElement = std::make_shared<Fem3DElementCube>(cube);
					femElement->setMassDensity(980.0);   // 0.98 g/cm^-3 (2-part silicone rubber a.k.a. RTV6166)
					femElement->setPoissonRatio(0.499);  // From the paper (near 0.5)
					femElement->setYoungModulus(15.3e3); // 15.3 kPa (From the paper)
					femElement->initialize(*state);
					addFemElement(femElement);
				}
			}
		}
	}

	/// Update the current state based on some offset resulting from compressing the cube
	/// \param offset to apply.
	void applyDofCorrection(const SurgSim::Math::Vector& offset)
	{
		m_currentState->getPositions() += offset;
	}

	/// Get beads of the truth cube
	/// \return coordinate of beads
	/// \note The beads are all the internal nodes of the cube
	std::vector<std::vector<std::vector<Vector3d>>> getBeadsLocation()
	{
		std::vector<std::vector<std::vector<Vector3d>>> beadsLocation;
		auto positions = m_currentState->getPositions();

		beadsLocation.resize(m_numNodesPerAxis - 2);
		for (int i = 1; i < m_numNodesPerAxis - 1; i++)
		{
			beadsLocation[i-1].resize(m_numNodesPerAxis - 2);
			for (int j = 1; j < m_numNodesPerAxis - 1; j++)
			{
				beadsLocation[i-1][j-1].resize(m_numNodesPerAxis - 2);
				for (int k = 1; k < m_numNodesPerAxis - 1; k++)
				{
					beadsLocation[i-1][j-1][k-1] = SurgSim::Math::getSubVector(positions, get1DIndexFrom3D(i, j, k), 3);
				}
			}
		}
		return beadsLocation;
	}

private:
	typedef std::array<SurgSim::Math::Vector3d, 8> CubeNodesType;

	// Number of point per dimensions
	int m_numNodesPerAxis;

	/// Boundary condition (list of degrees of freedom to be fixed)
	std::vector<size_t> m_boundaryConditions;

	/// Boundary condition displacement (displacement to apply for each boundary condition)
	std::vector<double> m_boundaryConditionsDisplacement;

	// Nodes of the original truth cube
	CubeNodesType m_cubeNodes;
};


/// Build the constrained system for a given truth cube representation
/// \param truthCubeRepresentation	The Fem3D representation of the truth cube
/// \param[out] A the system matrix of size (numDof + numConstraint x numDof + numConstraint)
/// \param[out] B the system RHS vector of size (numDof + numConstraint)
/// \note Each row of the matrix H aims at fixing a node as a boundary condition.
/// \note For example, the constraint equation to fix the dof id 1 is in place (no displacement) is:
/// \note H(0 1 0 0...0).U(u0 u1 u2 u3...un) = 0 (or desired displacement)
void buildConstrainedSystem(std::shared_ptr<TruthCubeRepresentation> truthCubeRepresentation,
							SurgSim::Math::Matrix& A, SurgSim::Math::Vector& B)
{
	// The static system with constraints is defined as follow:
	// (K H^T).(U      ) = (F)
	// (H   0) (-lambda)   (E)

	size_t numConstraints = truthCubeRepresentation->getBoundaryConditions().size();
	size_t numDof = truthCubeRepresentation->getNumDof();
	A.resize(numDof + numConstraints, numDof + numConstraints);
	B.resize(numDof + numConstraints);
	SurgSim::Math::Matrix H(numConstraints, truthCubeRepresentation->getNumDof());
	H.setZero();
	B.setZero();

	// Build the temporary constraint matrix H along with the RHS vector B
	for (size_t i = 0; i < numConstraints; i++)
	{
		H(i, truthCubeRepresentation->getBoundaryConditions()[i]) = 1.0;
		B[numDof + i] = truthCubeRepresentation->getBoundaryConditionsDisplacement()[i];
	}

	A.setZero();
	// Copy K into A
	A.block(0,0, numDof, numDof) = truthCubeRepresentation->computeK(*truthCubeRepresentation->getInitialState());
	// Copy H into A
	A.block(numDof,0, numConstraints, numDof) = H;
	//Copy H^T into A
	A.block(0, numDof, numDof, numConstraints) = H.transpose();
}

/// Using static solver to find the displacement of truth cube
/// \param truthCubeRepresentation The Fem3D representation of truth cube
/// \return the vector of displacement for each dof (degree of freedom)
SurgSim::Math::Vector staticSolver(std::shared_ptr<TruthCubeRepresentation> truthCubeRepresentation)
{
	int numDof = truthCubeRepresentation->getNumDof();

	// Build the constrained system A.X = B
	SurgSim::Math::Matrix A;
	SurgSim::Math::Vector B;
	buildConstrainedSystem(truthCubeRepresentation, A, B);

	// Solve the constrained system A.X = B
	SurgSim::Math::Vector X = A.inverse() * B;

	// Extract the dof displacement vector from the solution X
	return X.segment(0, numDof);
}

/// Simulate the truth cube statically with the boundary conditions applied
/// \param truthCubeData The truth cube data (this is only useful to adjust the initial data)
/// \param truthCubeRepresentation The truth cube representation
/// \param displacement The displacement to apply along Y on the top layer (compression in meter)
void doSimulation(std::shared_ptr<TruthCubeData> truthCubeData,
				  std::shared_ptr<TruthCubeRepresentation> truthCubeRepresentation,
				  double displacement)
{
	// Create initial state
	// Note that the boundary conditions are NOT defined in this test in the state itself
	// This would simply modify the global stiffness matrix, which is not sufficient for this test
	// We prefer to keep the boundary conditions in a separate structure
	// (internal to TruthCubeRepresentation and apply the Lagrange multiplier technique to solve them).
	std::shared_ptr<SurgSim::Math::OdeState> initialState = std::make_shared<SurgSim::Math::OdeState>();
	truthCubeRepresentation->fillUpDeformableState(initialState);
	truthCubeRepresentation->adjustInitialBeadsPosition(initialState, truthCubeData->cubeData0);
	truthCubeRepresentation->setInitialState(initialState);

	// Create Fem3d cubes from the subdivision cubes
	truthCubeRepresentation->addFemCubes(initialState);

	// Setup boundary conditions and displacement
	truthCubeRepresentation->defineBoundaryCondition(displacement);

	// Wake Up the Representation
	truthCubeRepresentation->initialize(std::make_shared<SurgSim::Framework::Runtime>());
	truthCubeRepresentation->wakeUp();

	// Call staticSolver to find the offset values
	SurgSim::Math::Vector offset = staticSolver(truthCubeRepresentation);

	// Apply the correction to the simulated truth cube current state
	truthCubeRepresentation->applyDofCorrection(offset);
}

/// Copy simulation beads data into point cloud
/// \param truthCubeRepresentation	The simulated truth cube
/// \param representation	The representation of point cloud
void copySimulationBeadsIntoPointCloud(std::shared_ptr<TruthCubeRepresentation> truthCubeRepresentation,
									   std::shared_ptr<SurgSim::Graphics::OsgPointCloudRepresentation> representation)
{
	std::vector<std::vector<std::vector<Vector3d>>> beads = truthCubeRepresentation->getBeadsLocation();
	auto pointCloud = representation->getVertices();

	// Add deform to pointCloud
	for (size_t i = 0; i < beads.size(); i++)
	{
		for (size_t j = 0; j < beads[i].size(); j++)
		{
			for (size_t k = 0; k < beads[i][j].size(); k++)
			{
				pointCloud->addVertex(PointCloud::VertexType(beads[i][j][k]));
			}
		}
	}
}

// Copy experimental beads data into point cloud
/// \param truthCube	The experimental data for the truth cube
/// \param representation	The representation of point cloud
void copyExperimentalBeadsIntoPointCloud(std::vector<SurgSim::Math::Vector3d> truthCube,
							 std::shared_ptr<SurgSim::Graphics::OsgPointCloudRepresentation> representation)
{
	auto pointCloudCompressed = representation->getVertices();

	/// Loading the Truth Cube data into point cloud
	for (size_t i = 0; i < truthCube.size(); ++i)
	{
		pointCloudCompressed->addVertex(PointCloud::VertexType(truthCube[i]));
	}
}

/// Simple error analysis
/// \param cubeData The vector of all experimental beads 3d position
/// \param state The state of the simulated truth cube
/// \return The maximum error measured on each bead (in meter)
double analyzeError(std::vector<Vector3d> cubeData, std::shared_ptr<SurgSim::Math::OdeState> state)
{
	double maxError = 0.0;
	for (size_t cubeDataNodeId = 0; cubeDataNodeId < cubeData.size(); ++cubeDataNodeId)
	{
		size_t stateNodeId = searchForClosestNodeInState(state, cubeData[cubeDataNodeId]);
		auto diff = cubeData[cubeDataNodeId] - state->getPosition(stateNodeId);
		double error = diff.norm();
		if (error > maxError)
		{
			maxError = error;
		}
	}
	return maxError;
}

struct Fem3DVSTruthCubeRenderTests : public RenderTests
{
	void addComponent(std::shared_ptr<SurgSim::Framework::Component> component)
	{
		viewElement->addComponent(component);
	}

	void SetUp() override
	{
		RenderTests::SetUp();

		// Load truth cube data
		truthCubeData = std::make_shared<TruthCubeData>();
		parseTruthCubeData(truthCubeData);

		// Compute the center point of the cube
		SurgSim::Math::Vector3d center = SurgSim::Math::Vector3d::Zero();
		for (size_t nodeId = 0; nodeId < truthCubeData->cubeData0.size(); ++nodeId)
		{
			center += truthCubeData->cubeData0[nodeId];
		}
		center /= truthCubeData->cubeData0.size();

		// Compute the cube's corners for the Fem3d simulation
		double halfLength = 0.04;
		Vector3d X = Vector3d::UnitX();
		Vector3d Y = Vector3d::UnitY();
		Vector3d Z = Vector3d::UnitZ();
		cubeCorners[0] = center - halfLength * X - halfLength * Y - halfLength * Z;
		cubeCorners[1] = center + halfLength * X - halfLength * Y - halfLength * Z;
		cubeCorners[2] = center - halfLength * X + halfLength * Y - halfLength * Z;
		cubeCorners[3] = center + halfLength * X + halfLength * Y - halfLength * Z;
		cubeCorners[4] = center - halfLength * X - halfLength * Y + halfLength * Z;
		cubeCorners[5] = center + halfLength * X - halfLength * Y + halfLength * Z;
		cubeCorners[6] = center - halfLength * X + halfLength * Y + halfLength * Z;
		cubeCorners[7] = center + halfLength * X + halfLength * Y + halfLength * Z;
	}

	/// The truth cube data set
	std::shared_ptr<TruthCubeData> truthCubeData;

	/// The 8 corners of the cube defining the truth cube (useful to setup the Fem3D)
	std::array<Vector3d, 8> cubeCorners;
};

TEST_F(Fem3DVSTruthCubeRenderTests, rawDataTest)
{
	// Load the truth cube data
	std::shared_ptr<TruthCubeData> truthCubeData = std::make_shared<TruthCubeData>();
	parseTruthCubeData(truthCubeData);

	auto points0 = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation>("Data0");
	points0->setPointSize(4.0);
	points0->setColor(Vector4d(1.0, 1.0, 1.0, 1.0));
	copyExperimentalBeadsIntoPointCloud(truthCubeData->cubeData0, points0);
	addComponent(points0);

	auto points1 = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation>("Data1");
	points1->setPointSize(4.0);
	points1->setColor(Vector4d(1.0, 0.0, 0.0, 1.0));
	copyExperimentalBeadsIntoPointCloud(truthCubeData->cubeData1, points1);
	addComponent(points1);

	auto points2 = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation>("Data2");
	points2->setPointSize(4.0);
	points2->setColor(Vector4d(0.0, 1.0, 0.0, 1.0));
	copyExperimentalBeadsIntoPointCloud(truthCubeData->cubeData2, points2);
	addComponent(points2);

	auto points3 = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation>("Data3");
	points3->setPointSize(4.0);
	points3->setColor(Vector4d(0.0, 0.0, 1.0, 1.0));
	copyExperimentalBeadsIntoPointCloud(truthCubeData->cubeData3, points3);
	addComponent(points3);

	runTest(Vector3d(0.0, 0.0, 0.2), Vector3d::Zero(), 3000.0);
}

/// Simulate truth cube with 5% strain (4mm of displacement).
TEST_F(Fem3DVSTruthCubeRenderTests, Test5percentsStrain)
{
	/// Displacement of the top layer for this setup
	double displacement = -0.004;

	// Run the simulation
	std::shared_ptr<TruthCubeRepresentation> truthCubeRepresentation;
	truthCubeRepresentation = std::make_shared<TruthCubeRepresentation> ("TruthCube", cubeCorners);
	doSimulation(truthCubeData, truthCubeRepresentation, displacement);

	std::shared_ptr<SurgSim::Graphics::OsgPointCloudRepresentation> simulatedPoints;
	simulatedPoints = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation>("SimulatedPoints");
	copySimulationBeadsIntoPointCloud(truthCubeRepresentation, simulatedPoints);
	simulatedPoints->setPointSize(4.0);
	simulatedPoints->setColor(Vector4d(1.0, 0.0, 0.0, 1.0));
	addComponent(simulatedPoints);

	std::shared_ptr<SurgSim::Graphics::OsgPointCloudRepresentation> experimentalpoints;
	experimentalpoints = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation>("ExperimentalPoints");
	copyExperimentalBeadsIntoPointCloud(truthCubeData->cubeData1, experimentalpoints);
	experimentalpoints->setPointSize(4.0);
	experimentalpoints->setColor(Vector4d(1.0, 1.0, 1.0, 1.0));
	addComponent(experimentalpoints);

	double maxError = analyzeError(truthCubeData->cubeData1, truthCubeRepresentation->getCurrentState());
	std::cout << "The maximum error between simulated and experimental setup is " << maxError << " m" << std::endl;

	/// Run the thread
	runTest(Vector3d(0.0, 0.0, 0.2), Vector3d::Zero(), 3000.0);
}

/// Simulate truth cube with 12.5% strain (10mm of displacement).
TEST_F(Fem3DVSTruthCubeRenderTests, Test12percentsAndHalfStrain)
{
	/// Displacement of the top layer for this setup
	double displacement = -0.010;

	// Run the simulation
	std::shared_ptr<TruthCubeRepresentation> truthCubeRepresentation;
	truthCubeRepresentation = std::make_shared<TruthCubeRepresentation> ("TruthCube", cubeCorners);
	doSimulation(truthCubeData, truthCubeRepresentation, displacement);

	std::shared_ptr<SurgSim::Graphics::OsgPointCloudRepresentation> simulatedPoints;
	simulatedPoints = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation>("SimulatedPoints");
	copySimulationBeadsIntoPointCloud(truthCubeRepresentation, simulatedPoints);
	simulatedPoints->setPointSize(4.0);
	simulatedPoints->setColor(Vector4d(1.0, 0.0, 0.0, 1.0));
	addComponent(simulatedPoints);

	std::shared_ptr<SurgSim::Graphics::OsgPointCloudRepresentation> experimentalpoints;
	experimentalpoints = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation>("ExperimentalPoints");
	copyExperimentalBeadsIntoPointCloud(truthCubeData->cubeData2, experimentalpoints);
	experimentalpoints->setPointSize(4.0);
	experimentalpoints->setColor(Vector4d(1.0, 1.0, 1.0, 1.0));
	addComponent(experimentalpoints);

	double maxError = analyzeError(truthCubeData->cubeData2, truthCubeRepresentation->getCurrentState());
	std::cout << "The maximum error between simulated and experimental setup is " << maxError << " m" << std::endl;

	/// Run the thread
	runTest(Vector3d(0.0, 0.0, 0.2), Vector3d::Zero(), 3000.0);
}

/// Simulate truth cube with 18.25% strain (14.6mm of displacement).
TEST_F(Fem3DVSTruthCubeRenderTests, Test18percentsAndQuarterStrain)
{
	/// Displacement of the top layer for this setup
	double displacement = -0.0146;

	// Run the simulation
	std::shared_ptr<TruthCubeRepresentation> truthCubeRepresentation;
	truthCubeRepresentation = std::make_shared<TruthCubeRepresentation> ("TruthCube", cubeCorners);
	doSimulation(truthCubeData, truthCubeRepresentation, displacement);

	std::shared_ptr<SurgSim::Graphics::OsgPointCloudRepresentation> simulatedPoints;
	simulatedPoints = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation>("SimulatedPoints");
	copySimulationBeadsIntoPointCloud(truthCubeRepresentation, simulatedPoints);
	simulatedPoints->setPointSize(4.0);
	simulatedPoints->setColor(Vector4d(1.0, 0.0, 0.0, 1.0));
	addComponent(simulatedPoints);

	std::shared_ptr<SurgSim::Graphics::OsgPointCloudRepresentation> experimentalpoints;
	experimentalpoints = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation>("ExperimentalPoints");
	copyExperimentalBeadsIntoPointCloud(truthCubeData->cubeData3, experimentalpoints);
	experimentalpoints->setPointSize(4.0);
	experimentalpoints->setColor(Vector4d(1.0, 1.0, 1.0, 1.0));
	addComponent(experimentalpoints);

	double maxError = analyzeError(truthCubeData->cubeData3, truthCubeRepresentation->getCurrentState());
	std::cout << "The maximum error between simulated and experimental setup is " << maxError << " m" << std::endl;

	/// Run the thread
	runTest(Vector3d(0.0, 0.0, 0.2), Vector3d::Zero(), 3000.0);
}

}; // namespace Physics
}; // namespace SurgSim
