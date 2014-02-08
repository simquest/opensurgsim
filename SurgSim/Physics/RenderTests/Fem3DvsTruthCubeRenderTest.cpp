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
#include "SurgSim/Physics/RenderTests/RenderTest.h"

#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Graphics/PointCloudRepresentation.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/RigidTransform.h"

#include "SurgSim/Physics/PhysicsManager.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/FemElement3DCube.h"
#include "SurgSim/Physics/PhysicsManager.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Math/Quaternion.h"

using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Framework::SceneElement;
using SurgSim::Graphics::OsgPointCloudRepresentation;
using SurgSim::Physics::PhysicsManager;
using SurgSim::Framework::SceneElement;


namespace SurgSim
{
namespace Physics
{

typedef SurgSim::DataStructures::Vertices<void> CloudMesh;

/// Truth cube implementation by Fem3DRepresentation
class TruthCubeRepresentation : public Fem3DRepresentation
{
public:
	/// Constructor
	/// \param name	The name of the truth cube representation.
	/// \param size	The original length of the truth cube in m.
	/// \param level	The subdivision level.
	explicit TruthCubeRepresentation(const std::string& name, double size = 0.08, unsigned int level = 3) :
		m_name(name),
		m_size(size),
		m_levelSubdivision(level),
		m_numBoundayConditions(0),
		m_numDisplacements(0),
		Fem3DRepresentation(name)
	{
		const int dofPerNode = 3;

		m_numNodesPerAxis = pow(2, m_levelSubdivision) + 1;
		m_displacement.resize(m_numNodesPerAxis * m_numNodesPerAxis * m_numNodesPerAxis * dofPerNode);
		m_boundary.resize(m_numNodesPerAxis * m_numNodesPerAxis * m_numNodesPerAxis * dofPerNode);
		m_cubeNodes = createCube(m_size);
	}

	/// Destructor 
	~TruthCubeRepresentation()
	{
	}

	/// Gets the number of nodes per axis of truth cube
	/// \return The number of nodes per axis.
	unsigned int getNumNodesPerAxis()
	{
		return m_numNodesPerAxis;
	}

	/// Gets displacement values
	/// \return The vector of displacement.
	std::vector<double> getDisplacements()
	{
		return m_displacement;
	}

	/// Get number of displacement 
	/// \return the number of displacements have been set.
	int getNumofDisplacements()
	{
		return m_numDisplacements;
	}

	/// Get number of boundary conditions
	/// \return the number of boundary conditions
	int getNumofBoundaryConsitions()
	{
		return m_numBoundayConditions;
	}

	/// Sets state for the truth cube representation
	/// \param state	The deformable state. 
	void setDeformableState(std::shared_ptr<DeformableRepresentationState> state)
	{
		int nodeId = 0;
		for (int k = 0; k < m_numNodesPerAxis; k++)
		{
			for (int j = 0; j < m_numNodesPerAxis; j++)
			{
				for (int i = 0; i < m_numNodesPerAxis; i++)
				{
					state->getPositions()[nodeId * 3 + 0] = m_nodes[i][j][k][0];
					state->getPositions()[nodeId * 3 + 1] = m_nodes[i][j][k][1];
					state->getPositions()[nodeId * 3 + 2] = m_nodes[i][j][k][2];
					nodeId++;
				}
			}
		}
	}

	/// Sets the boundary condition for the truth cube
	/// \param displacement	The displacement of the boundary conditions
	void setBoundaryCondition(double displacement)
	{
		int nodeId = 0;
		for (int k = 0; k < m_numNodesPerAxis; k++)
		{
			for (int j = 0; j < m_numNodesPerAxis; j++)
			{
				for (int i = 0; i < m_numNodesPerAxis; i++)
				{
					// At bottom layer nodes
					if (j == 0)
					{
						// Add boundary condition
						m_boundary[nodeId * 3 + 0] = true;
						m_boundary[nodeId * 3 + 1] = true;
						m_boundary[nodeId * 3 + 2] = true;
						m_numBoundayConditions += 3;
					}
					// At top layer nodes
					if (j == m_numNodesPerAxis-1)
					{
						// Apply displacement value
						m_displacement[nodeId * 3 + 1] = displacement;
						m_numDisplacements++;
					}
					nodeId++;
				}
			}
		}
	}

	std::vector<bool> getBoundaryConditions()
	{
		return m_boundary;
	}

	/// Creates the subdivision truth cube nodes
	void createTruthCubeMesh()
	{
		m_nodes.resize(m_numNodesPerAxis);
		for (int i = 0; i < m_numNodesPerAxis; i++)
		{
			m_nodes[i].resize(m_numNodesPerAxis);
			Vector3d extremitiesX0[4] = {m_cubeNodes[0], m_cubeNodes[2], m_cubeNodes[4], m_cubeNodes[6]};
			Vector3d extremitiesX1[4] = {m_cubeNodes[1], m_cubeNodes[3], m_cubeNodes[5], m_cubeNodes[7]};

			Vector3d extremitiesXi[4];
			for (int index = 0; index < 4; index++)
			{
				extremitiesXi[index] = extremitiesX0[index] * (1.0 - i / (m_numNodesPerAxis - 1.0)) +
					extremitiesX1[index] * i / (m_numNodesPerAxis - 1.0);
			}

			for (int j = 0; j < m_numNodesPerAxis; j++)
			{
				Vector3d extremitiesY0[2] = {extremitiesXi[0], extremitiesXi[2]};
				Vector3d extremitiesY1[2] = {extremitiesXi[1], extremitiesXi[3]};

				Vector3d extremitiesYi[2];
				for (int index = 0; index < 2; index++)
				{
					extremitiesYi[index] = extremitiesY0[index] * (1.0 - j/(m_numNodesPerAxis - 1.0)) +
						extremitiesY1[index] * j / (m_numNodesPerAxis - 1.0);
				}
				m_nodes[i][j].resize(m_numNodesPerAxis);
				for (int k=0; k < m_numNodesPerAxis; k++)
				{
					m_nodes[i][j][k] = extremitiesYi[0] * (1.0 - k / (m_numNodesPerAxis - 1.0)) + extremitiesYi[1]
										* k / (m_numNodesPerAxis - 1.0);
				}
			}
		}
	}

	/// Adds the Fem3D elements of small cubes
	/// \param state	The deformable state for initialization.
	void addFemCubes(std::shared_ptr<DeformableRepresentationState> state)
	{
		for (int i = 0; i < m_numNodesPerAxis - 1; i++)
		{
			for (int j = 0; j < m_numNodesPerAxis - 1; j++)
			{
				for (int k = 0; k < m_numNodesPerAxis - 1; k++)
				{
					std::array<int, 8> cubeNodeIds;
					cubeNodeIds[0] = k * (m_numNodesPerAxis * m_numNodesPerAxis) + j * m_numNodesPerAxis + i;
					cubeNodeIds[1] = k * (m_numNodesPerAxis * m_numNodesPerAxis) + j * m_numNodesPerAxis + i + 1;
					cubeNodeIds[2] = k * (m_numNodesPerAxis * m_numNodesPerAxis) + (j + 1) * m_numNodesPerAxis + i;
					cubeNodeIds[3] = k * (m_numNodesPerAxis * m_numNodesPerAxis) + (j + 1) * m_numNodesPerAxis + i + 1;
					cubeNodeIds[4] = (k + 1) * (m_numNodesPerAxis * m_numNodesPerAxis) + j * m_numNodesPerAxis + i;
					cubeNodeIds[5] = (k + 1) * (m_numNodesPerAxis * m_numNodesPerAxis) + j * m_numNodesPerAxis + i + 1;
					cubeNodeIds[6] = (k + 1) * (m_numNodesPerAxis * m_numNodesPerAxis) +
										(j + 1) * m_numNodesPerAxis + i;
					cubeNodeIds[7] = (k + 1) * (m_numNodesPerAxis * m_numNodesPerAxis) +
										(j+1) * m_numNodesPerAxis + i + 1;

					std::array<unsigned int, 8> cube = {
						cubeNodeIds[0], cubeNodeIds[1], cubeNodeIds[3], cubeNodeIds[2],
						cubeNodeIds[4], cubeNodeIds[5], cubeNodeIds[7], cubeNodeIds[6]};

					// Add FemElement3DCube for each cube
					std::shared_ptr<FemElement3DCube> femElement = std::make_shared<FemElement3DCube>(cube, *state);
					femElement->setMassDensity(1250.0);
					femElement->setPoissonRatio(0.499);
					femElement->setYoungModulus(15.3e5);
					femElement->initialize(*state);
					this->addFemElement(femElement);

				}
			}
		}
	}

	/// Update dof based on correction values after compressing the cube
	/// \params offset The correction values.
	void applyDofCorrection(const SurgSim::Math::Vector& offset)
	{
		int nodeId = 0;
		for (int k = 0; k < m_numNodesPerAxis; k++)
		{
			for (int j = 0; j < m_numNodesPerAxis; j++)
			{
				for (int i = 0; i < m_numNodesPerAxis; i++)
				{
					m_nodes[i][j][k].x() += (offset)[nodeId+0];
					m_nodes[i][j][k].y() += (offset)[nodeId+1];
					m_nodes[i][j][k].z() += (offset)[nodeId+2];
					nodeId += 3;
				}
			}
		}
	}

	/// Get beads of the truth cube
	/// Note: the beads are actually all the internal nodes of the cube
	/// \return coordinate of beads
	std::vector<std::vector<std::vector<Vector3d>>> getBeads()
	{
		std::vector<std::vector<std::vector<Vector3d>>> beads;

		beads.resize(m_numNodesPerAxis - 2);
		for (int i = 0; i < m_numNodesPerAxis - 2; i++)
		{
			beads[i].resize(m_numNodesPerAxis - 2);
			for (int j = 0; j < m_numNodesPerAxis - 2; j++)
			{
				beads[i][j].resize(m_numNodesPerAxis - 2);
				for (int k = 0; k < m_numNodesPerAxis - 2; k++)
				{
						beads[i][j][k] = m_nodes[i + 1][j + 1][k + 1];
				}
			}
		}
		return beads;
	}

private:
	typedef std::array<SurgSim::Math::Vector3d, 8> CubeNodesType;

	/// Creates the truth cube nodes
	/// \param size	The size in m of the truth cube.
	CubeNodesType createCube(double size)
	{
		CubeNodesType cubeNodes;
		cubeNodes[0] = Vector3d(-0.5,-0.5,-0.5) * size; cubeNodes[1] = Vector3d(0.5,-0.5,-0.5) * size;
		cubeNodes[2] = Vector3d(-0.5, 0.5,-0.5) * size; cubeNodes[3] = Vector3d( 0.5, 0.5,-0.5) * size;
		cubeNodes[4] = Vector3d(-0.5,-0.5, 0.5) * size; cubeNodes[5] = Vector3d( 0.5,-0.5, 0.5) * size;
		cubeNodes[6] = Vector3d(-0.5, 0.5, 0.5) * size; cubeNodes[7] = Vector3d( 0.5, 0.5, 0.5) * size;

		return cubeNodes;
	}

	// Name of the truth cube representation
	std::string m_name;
	// Size in m of the original truth cube
	double m_size; 

	// Level of subdivision 
	unsigned int m_levelSubdivision;

	// Number of point per dimensions for each subdivision level
	int m_numNodesPerAxis; 

	// Number of boundary conditions
	int m_numBoundayConditions;

	// Number of displacements
	int m_numDisplacements;

	// Displacement
	std::vector<double> m_displacement;

	// BoundaryCondition
	std::vector<bool> m_boundary;

	// Nodes of the original truth cube 
	CubeNodesType m_cubeNodes;

	// Nodes of the subdivision cubes in 3D
	std::vector<std::vector<std::vector<Vector3d>>> m_nodes;
};

struct TruthCube
{
	// Data positions of uncompressed data
	std::vector<Vector3d> cubeData0;

	// Data positions of 5% strain
	std::vector<Vector3d> cubeData1;

	// Data positions of  12.5% strain
	std::vector<Vector3d> cubeData2;

	// Data positions of  18.25% strain
	std::vector<Vector3d> cubeData3;
};

/// Storage for experimental truth cube data.
std::shared_ptr<TruthCube> truthCube;

/// Parsing Truth Cube data from an external file
/// \param truthCube a container of cube data for all strains
/// \return True if the Truth Cube Data is successful loaded, otherwise false
bool parseTruthCubeData(std::shared_ptr<TruthCube> truthCube)
{
	// Position of uncompressed data, 5% strain, 12.5% strain, 18.25% strain
	std::array<Vector3d, 4> position;// position0, position1, position2, position3;

	const int numCommentLine = 7;
	std::string lineId;
	char comma;
	int i,j,k;
	int index = 0;

	// Conversion constant from millimeter to meter
	const double cm2m = 1000.0;

	const SurgSim::Framework::ApplicationData data("config.txt");

	std::string filename = data.findFile("uniaxial_positions.csv");

	std::ifstream datafile(filename);

	if (! datafile.good())
	{
		return false;
	}

	while (std::getline(datafile, lineId))
	{
		if (++index > numCommentLine)
		{
			std::stringstream strstream(lineId);
			strstream >> i >> comma >> j >> comma >> k >> comma
				>> position[0].x() >> comma >> position[0].y() >> comma >> position[0].z() >> comma
				>> position[1].x() >> comma >> position[1].y() >> comma >> position[1].z() >> comma
				>> position[2].x() >> comma >> position[2].y() >> comma >> position[2].z() >> comma
				>> position[3].x() >> comma >> position[3].y() >> comma >> position[3].z();

			// Re-scale, rotate and translate data of the truth cube
			// so that it would be ready to use in the simulation.
			for (unsigned int i = 0; i < position.size(); i++)
			{
				// Rescale into meter unit;
				position[i] /= cm2m;

				// Rotate CCW PI/2 around X-axis to correct the orientation.
				auto temp = position[i].y();
				position[i].y() = -1* position[i].z();
				position[i].z() = -1* temp;

				// Translate 20mm down on Y-Axis
				position[i].y()  -= 0.017;

				// Translate 10mm down on Z-Axis
				position[i].z()  += 0.013;
			}

			// Store proper strains for each cubeData
			truthCube->cubeData0.push_back(position[0]);
			truthCube->cubeData1.push_back(position[1]);
			truthCube->cubeData2.push_back(position[2]);
			truthCube->cubeData3.push_back(position[3]);
		}
	}
	return true;

};

/// Compute matrix H in staticSolver()
/// \param row	The number of row of H
/// \param col	The number of column of H.
/// \param truthcubeRepresentation	The Fem3D representation of the truth cube
/// \return the computed matrix H
SurgSim::Math::Matrix computeH(int row, int col, std::shared_ptr<TruthCubeRepresentation> truthCubeRepresentation)
{
	int j;
	int indexCol = 0;
	SurgSim::Math::Matrix H(row, col);
	H.setZero();
	for (int i = 0; i < row; i++)
	{ 
		for (j = indexCol; j < col; j++)
		{
			if (truthCubeRepresentation->getDisplacements()[j] != 0 ||
				truthCubeRepresentation->getBoundaryConditions()[j])
			{
				H(i, j) = 1.0;
				break;
			}
		}
		// update for next column index
		indexCol = j+1;
	}

	return H;
}

/// Compute vector E in staticSolver()
/// \param col	The number of col of E
/// \param truthcubeRepresentation	The Fem3D representation of the truth cube
/// \return the computed vector E
SurgSim::Math::Vector computeE(int col, std::shared_ptr<TruthCubeRepresentation> truthCubeRepresentation)
{
	SurgSim::Math::Vector E(col);
	E.setZero();
	std::vector<double> displacements = truthCubeRepresentation->getDisplacements();
	std::vector<bool> bc = truthCubeRepresentation->getBoundaryConditions();

	// Copy boundary conditions and displacement into E
	int displacementId = 0;
	for (unsigned int i = 0; i < truthCubeRepresentation->getNumDof(); i++)
	{
		if (displacements[i] || bc[i])
		{
			E(displacementId) = displacements[i];
			displacementId++;
		}
	}

	return E;
}

/// Compute extend matrix A in staticSolver()
/// \param numConstraints	The number of constraints of system
/// \param numDof	The number of dof of the truth cube
/// \return the extended matrix A
SurgSim::Math::Matrix computeA(int numConstraints, int numDof, 
							 SurgSim::Math::Matrix K, SurgSim::Math::Matrix H)
{
	SurgSim::Math::Matrix A(numDof+numConstraints, numDof+numConstraints);
	A.setZero();

	// Copy K into A
	A.block(0,0, numDof, numDof) = K;

	// Copy H into A
	A.block(numDof,0, numConstraints, numDof) = H;

	//Copy H^T into A
	A.block(0, numDof, numDof, numConstraints) = H.transpose();

	return A;
}


/// Using static solver to find the displacement of truth cube
/// \param truthcubeRepresentation	The Fem3D representation of truth cube
/// \return the list of displacements for each dofId
SurgSim::Math::Vector staticSolver(std::shared_ptr<TruthCubeRepresentation> truthCubeRepresentation)
{
	// Apply Lagrange multiplier method.
	// The static system with constraints is defined:
	// (K H^T).(U      ) = (F)
	// (H   0) (-lambda)   (E)
	// 
	// Extended matrix A = (K H^T)
	//                     (H   0)
	// Unknown vector  X = (      U)
	//                     (-lambda)
	// RHS             B = (F)
	//                     (E)
	// where:
	//		 - K is the stiffness matrix of size (numDof x numDof)
	//		 - H is constraint matrix of size (numConstraint x numDof)
	//		 - F is the initial force, F = 0
	//		 - E is displacements vector for each constraint in H
	//		 - U and lambda are the unknown variables.

	// numConstraints = number of boundary conditions  + number of displacements
	int numConstraints = truthCubeRepresentation->getNumofBoundaryConsitions() +
						truthCubeRepresentation->getNumofDisplacements();

	int numDof = truthCubeRepresentation->getNumDof();

	// Compute matrix H
	SurgSim::Math::Matrix H = computeH(numConstraints, numDof, truthCubeRepresentation);

	// Compute stiffness matrix K
	std::shared_ptr<DeformableRepresentationState> initialState = truthCubeRepresentation->getInitialState();
	SurgSim::Math::Matrix K = truthCubeRepresentation->computeK(*initialState);

	// Compute extended matrix A
	SurgSim::Math::Matrix A = computeA(numConstraints, numDof, K, H);

	// Compute the inverse of matrix A
	SurgSim::Math::Matrix Ainv = A.inverse();

	// Compute vector E
	SurgSim::Math::Vector E = computeE(numConstraints, truthCubeRepresentation);

	// Compute vector B
	SurgSim::Math::Vector b(numDof+numConstraints);
	b.setZero();
	b.segment(numDof, numConstraints) = E;

	// Create vector X
	SurgSim::Math::Vector X(numDof+numConstraints);
	X.setZero();

	// Compute the unknown variable X
	X = Ainv*b;

	// Compute vector U
	SurgSim::Math::Vector U = X.segment(0, numDof);

	return U;
}

/// Apply the compression with different strains on the truth cube
/// \param truthcubecRepresentation 	The representation of truth cube 
/// \param displacement The displacement
void doSimulation(std::shared_ptr<TruthCubeRepresentation> truthCubeRepresentation,
				  double displacement)
{
	// Create subdivision cubes mesh
	truthCubeRepresentation->createTruthCubeMesh();

	// Create initial state
	std::shared_ptr<DeformableRepresentationState> initialState = std::make_shared<DeformableRepresentationState>();
	initialState->setNumDof(truthCubeRepresentation->getNumDofPerNode(), 
		pow(truthCubeRepresentation->getNumNodesPerAxis(), 3));
	truthCubeRepresentation->setDeformableState(initialState);
	truthCubeRepresentation->setInitialState(initialState);

	// Create Fem3d cubes from the subdivision cubes
	truthCubeRepresentation->addFemCubes(initialState);

	// Setup boundary conditions and displacement
	truthCubeRepresentation->setBoundaryCondition(displacement);

	// Call staticSolver to find the offset values
	SurgSim::Math::Vector offset = staticSolver(truthCubeRepresentation);

	// Update the correction
	truthCubeRepresentation->applyDofCorrection(offset);
}

/// Copy simulation beads data into point cloud
/// \param truthCubePrepresentation	The representation of truth cube
/// \param representation	The representation of point cloud
void copySimulationBeadsIntoPointCloud(std::shared_ptr<TruthCubeRepresentation> truthCubeRepresentation,
							 std::shared_ptr<SurgSim::Graphics::OsgPointCloudRepresentation<void>> representation)
{
	std::vector<std::vector<std::vector<Vector3d>>> beads = truthCubeRepresentation->getBeads();
	auto pointCloud = representation->getVertices();

	// Add deform to pointCloud
	for (unsigned int i = 0; i < beads.size(); i++)
	{
		for (unsigned int j = 0; j < beads[i].size(); j++)
		{
			for (unsigned int k = 0; k < beads[i][j].size(); k++)
			{
				pointCloud->addVertex(CloudMesh::VertexType(beads[i][j][k]));
			}
		}
	}
}

// Copy experimental beads data into point cloud
/// \param truthCube	The experimental data for the truth cube
/// \param representation	The representation of point cloud
void copyExperimentalBeadsIntoPointCloud(std::vector<SurgSim::Math::Vector3d>& truthCube,
							 std::shared_ptr<SurgSim::Graphics::OsgPointCloudRepresentation<void>> representation)
{
	auto pointCloudCompressed = representation->getVertices();

	/// Loading the Truth Cube data into point cloud
	for (size_t i = 0; i < truthCube.size(); ++i)
	{
		pointCloudCompressed->addVertex(CloudMesh::VertexType(truthCube[i]));
	}

}

struct Fem3DVSTruthCubeRenderTests : public RenderTests
{

	void runTest(std::shared_ptr<SurgSim::Graphics::OsgPointCloudRepresentation<void>> expectData,
		std::shared_ptr<SurgSim::Graphics::OsgPointCloudRepresentation<void>> actualData)
	{
		/// Setup for expect data points
		expectData->setPointSize(4.0);
		RigidTransform3d expectPose = makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0, 0.0, 0.0));
		expectData->setInitialPose(expectPose);
		expectData->setColor(Vector4d(1.0, 0.0, 0.0, 0.0));

		/// Setup for actual data points
		actualData->setPointSize(4.0);
		RigidTransform3d actualPose = makeRigidTransform(Quaterniond::Identity(), Vector3d::Zero());
		actualData->setInitialPose(actualPose);
		actualData->setColor(Vector4d(0.0, 1.0, 0.0, 0.0));

		viewElement->addComponent(expectData);
		viewElement->addComponent(actualData);
		viewElement->enableManipulator(true);
		viewElement->setManipulatorParameters(Vector3d(0.0, 0.0, 0.2), Vector3d::Zero());

		/// Run the thread
		runtime->start();

		boost::this_thread::sleep(boost::posix_time::milliseconds(60000));
	}
};

/// Simulate truth cube with 4mm of displacement.
TEST_F(Fem3DVSTruthCubeRenderTests, CompressedCubeTest1)
{
	/// Displacement for this setup
	double displacement = -0.004;

	std::shared_ptr<TruthCubeRepresentation> truthCubeRepresentation = 
		std::make_shared<TruthCubeRepresentation> ("TruthCube");

	// Run simulation with specific displacement
	doSimulation(truthCubeRepresentation, displacement);

	auto actualpoints = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation<void>>
		("CompressedTruthCube");

	copySimulationBeadsIntoPointCloud(truthCubeRepresentation, actualpoints);

	// Initialize truthCube variable
	truthCube = std::make_shared<TruthCube>();

	// Parsing TruthCube data
	parseTruthCubeData(truthCube);

	auto expectedpoints = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation<void>>
		("ExperimentalTruthCube");

	copyExperimentalBeadsIntoPointCloud(truthCube->cubeData1, expectedpoints);

	/// Run the thread
	runTest(expectedpoints, actualpoints);

}

/// Simulate truth cube with 10mm of displacement.
TEST_F(Fem3DVSTruthCubeRenderTests, CompressedCubeTest2)
{

	/// Displacement for this setup
	double displacement = -0.010;

	std::shared_ptr<TruthCubeRepresentation> truthCubeRepresentation = 
		std::make_shared<TruthCubeRepresentation> ("TruthCube");

	// Run simulation with specific displacement
	doSimulation(truthCubeRepresentation, displacement);

	auto actualpoints = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation<void>>
		("CompressedTruthCube");

	copySimulationBeadsIntoPointCloud(truthCubeRepresentation, actualpoints);

	// Initialize truthCube variable
	truthCube = std::make_shared<TruthCube>();

	// Parsing TruthCube data
	parseTruthCubeData(truthCube);

	auto expectedpoints = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation<void>>
		("ExperimentalTruthCube");

	copyExperimentalBeadsIntoPointCloud(truthCube->cubeData2, expectedpoints);

	/// Run the thread
	runTest(expectedpoints, actualpoints);
}

/// Simulate truth cube with 14.6mm of displacement.
TEST_F(Fem3DVSTruthCubeRenderTests, CompressedCubeTest3)
{
	/// Displacement for this setup
	double displacement = -0.0146;

	std::shared_ptr<TruthCubeRepresentation> truthCubeRepresentation = 
		std::make_shared<TruthCubeRepresentation> ("TruthCube");

	// Run simulation with specific displacement
	doSimulation(truthCubeRepresentation, displacement);

	auto actualpoints = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation<void>>
		("CompressedTruthCube");

	copySimulationBeadsIntoPointCloud(truthCubeRepresentation, actualpoints);

	// Initialize truthCube variable
	truthCube = std::make_shared<TruthCube>();

	// Parsing TruthCube data
	parseTruthCubeData(truthCube);

	auto expectedpoints = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation<void>>
		("ExperimentalTruthCube");

	copyExperimentalBeadsIntoPointCloud(truthCube->cubeData3, expectedpoints);

	/// Run the thread
	runTest(expectedpoints, actualpoints);
}

}; // namespace Physics
}; // namespace SurgSim

