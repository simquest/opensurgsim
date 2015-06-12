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

/// \file Fem2DMechanicalValidationTests.cpp
/// This file tests the mechanical behaviors of the Fem2DRepresentation.

#include <gtest/gtest.h>

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem2DRepresentation.h"
#include "SurgSim/Physics/Fem2DElementTriangle.h"

using SurgSim::Math::Matrix;
using SurgSim::Math::Vector;
using SurgSim::Math::Vector3d;

namespace
{

static const double inchToMeter = 0.0254;
static const double ksiToPascal = 6894757.293; // Kip.in^-2 -> Pa = N.m^-2
static const double kipToNewton = 4448.221615; // Kip -> N

const double epsilonCantilever1 = 1e-8;
const double epsilonCantilever2 = 1e-8;
const double epsilonPlateWithSemiCirculatHole = 5e-5;
const double epsilonPlateBending = 2e-7;

}

namespace SurgSim
{

namespace Physics
{

/// Mechanical validation tests class
/// Our validation tests are based on the following Thesis:
/// "Development of Membrane, Plate and Flat Shell Elements in Java"
/// Kaushalkumar Kansara, May 2004
/// In this thesis, the same formulation has been chosen to simulate the plate bending deformation,
/// based on Batoz et al. work. Also, a large section of the thesis is dedicated to mechanical validation with
/// all necessary details to reproduce the simulation and comparison have been done against a commercial finite
/// element analysis program (SAP 2000).
class Fem2DMechanicalValidationTests : public ::testing::Test
{
private:
	std::shared_ptr<Fem2DRepresentation> m_fem;

	// Physical properties
	double m_nu;
	double m_E;

	// Geometric properties
	double m_thickness;

	// Force and displacement vectors
	SurgSim::Math::Vector m_F, m_U;

protected:
	void SetUp() override
	{
		m_fem = std::make_shared<Fem2DRepresentation>("Fem2D");
	}

	void applyBoundaryConditions()
	{
		auto boundaryConditions = m_fem->getInitialState()->getBoundaryConditions();
		for (auto bc = boundaryConditions.begin(); bc != boundaryConditions.end(); bc++)
		{
			m_F[*bc] = 0.0;
		}
	}

public:
	void setNodePositions(const std::vector<Vector3d>& nodes, const std::vector<size_t>& fixedNodes)
	{
		const size_t numDofPerNode = m_fem->getNumDofPerNode();
		std::shared_ptr<SurgSim::Math::OdeState> state = std::make_shared<SurgSim::Math::OdeState>();
		state->setNumDof(numDofPerNode, nodes.size());
		for (size_t nodeId = 0; nodeId < nodes.size(); nodeId++)
		{
			state->getPositions().segment<3>(numDofPerNode * nodeId) = nodes[nodeId];
		}
		for (auto fixedNodeId = fixedNodes.begin(); fixedNodeId != fixedNodes.end(); fixedNodeId++)
		{
			state->addBoundaryCondition(*fixedNodeId);
		}
		m_fem->setInitialState(state);
	}

	void addTriangle(const std::array<size_t, 3>& t, double youngModulus, double poissonRatio, double thickness)
	{
		std::shared_ptr<Fem2DElementTriangle> triangle = std::make_shared<Fem2DElementTriangle>(t);
		triangle->setYoungModulus(youngModulus);
		triangle->setPoissonRatio(poissonRatio);
		triangle->setMassDensity(1.0);  // In static mode, the mass density is not used, but it needs to be non null to
										// pass the initialize validation
		triangle->setThickness(thickness);
		m_fem->addFemElement(triangle);
	}

	// Make sure you call setNodePositions first to initialize the initialState
	void addPunctualLoad(size_t nodeId, const Vector3d& f)
	{
		// Apply load at extremity
		if (m_F.size() != static_cast<Vector::Index>(m_fem->getInitialState()->getNumDof()))
		{
			m_F.setZero(m_fem->getInitialState()->getNumDof());
		}
		m_F.segment(m_fem->getNumDofPerNode() * nodeId, 3) = f;
	}

	// Make sure you call setNodePositions first to initialize the initialState
	void addUniformSurfaceLoad(const Vector3d& forceInNewtonPerSquareMeter)
	{
		std::vector<Vector3d> forces;
		const size_t numNodes = m_fem->getInitialState()->getNumNodes();

		forces.resize(numNodes);
		for (auto f = std::begin(forces); f != std::end(forces); f++)
		{
			(*f).setZero();
		}
		for (size_t triangleId = 0; triangleId < m_fem->getNumFemElements(); triangleId++)
		{
			auto triangle = std::static_pointer_cast<Fem2DElementTriangle>(m_fem->getFemElement(triangleId));
			size_t nodeId0 = triangle->getNodeId(0);
			size_t nodeId1 = triangle->getNodeId(1);
			size_t nodeId2 = triangle->getNodeId(2);
			const Vector3d A = m_fem->getInitialState()->getPosition(nodeId0);
			const Vector3d B = m_fem->getInitialState()->getPosition(nodeId1);
			const Vector3d C = m_fem->getInitialState()->getPosition(nodeId2);
			const double Area = ((B - A).cross(C - A)).norm() / 2.0;
			Vector3d f = Area * forceInNewtonPerSquareMeter;
			// Uniform distribution, so the resulting force f is to be applied at the triangle center of mass:
			forces[nodeId0] += f / 3.0;
			forces[nodeId1] += f / 3.0;
			forces[nodeId2] += f / 3.0;
		}

		for (size_t nodeId = 0; nodeId < numNodes; nodeId++)
		{
			addPunctualLoad(nodeId, forces[nodeId]);
		}

		applyBoundaryConditions();
	}

	void solve()
	{
		m_fem->initialize(std::make_shared<SurgSim::Framework::Runtime>());
		m_fem->update(*(m_fem->getCurrentState()), Math::ODEEQUATIONUPDATE_K);
		Matrix K = m_fem->getK();
		m_fem->getCurrentState()->applyBoundaryConditionsToMatrix(&K);
		m_fem->getCurrentState()->applyBoundaryConditionsToVector(&m_F);
		m_U = K.inverse() * m_F;
	}

	double getUx(size_t nodeId) const
	{
		return m_U[m_fem->getNumDofPerNode() * nodeId];
	}

	double getUy(size_t nodeId) const
	{
		return m_U[m_fem->getNumDofPerNode() * nodeId + 1];
	}

	double getUz(size_t nodeId) const
	{
		return m_U[m_fem->getNumDofPerNode() * nodeId + 2];
	}
};

TEST_F(Fem2DMechanicalValidationTests, MembraneCantileverTest1)
{
	const double youngModulus = 30000 * ksiToPascal;
	const double poissonRatio = 0.25;
	const double thickness = 1.0  * inchToMeter;
	const double L = 48.0 * inchToMeter;
	const double h = 12.0 * inchToMeter;

	std::vector<Vector3d> nodes;
	nodes.push_back(Vector3d(-L / 2.0, - h / 2.0, 0.0));
	nodes.push_back(Vector3d(-L / 4.0, - h / 2.0, 0.0));
	nodes.push_back(Vector3d(     0.0, - h / 2.0, 0.0));
	nodes.push_back(Vector3d( L / 4.0, - h / 2.0, 0.0));
	nodes.push_back(Vector3d( L / 2.0, - h / 2.0, 0.0));
	nodes.push_back(Vector3d(-L / 2.0,   h / 2.0, 0.0));
	nodes.push_back(Vector3d(-L / 4.0,   h / 2.0, 0.0));
	nodes.push_back(Vector3d(     0.0,   h / 2.0, 0.0));
	nodes.push_back(Vector3d( L / 4.0,   h / 2.0, 0.0));
	nodes.push_back(Vector3d( L / 2.0,   h / 2.0, 0.0));

	std::vector<size_t> fixedNodes;
	fixedNodes.push_back(0);
	fixedNodes.push_back(5);

	setNodePositions(nodes, fixedNodes);

	const int numTriangles = 8;
	const std::array<std::array<size_t, 3>, numTriangles> trianglesNodeIds =
	{{
		{{0, 6, 5}}, {{0, 1, 6}}, {{1, 7, 6}}, {{1, 2, 7}},
		{{2, 8, 7}}, {{2, 3, 8}}, {{3, 9, 8}}, {{3, 4, 9}}
	}};

	for (size_t triangleId = 0; triangleId < numTriangles; triangleId++)
	{
		addTriangle(trianglesNodeIds[triangleId], youngModulus, poissonRatio, thickness);
	}

	addPunctualLoad(4, Vector3d(0, 20.0 * kipToNewton, 0));
	addPunctualLoad(9, Vector3d(0, 20.0 * kipToNewton, 0));

	solve();

	EXPECT_NEAR(-0.014159 * inchToMeter, getUx(9), epsilonCantilever1);
	EXPECT_NEAR( 0.090347 * inchToMeter, getUy(9), epsilonCantilever1);
	EXPECT_NEAR(-0.010825 * inchToMeter, getUx(7), epsilonCantilever1);
	EXPECT_NEAR( 0.030403 * inchToMeter, getUy(7), epsilonCantilever1);
}

TEST_F(Fem2DMechanicalValidationTests, MembraneCantileverTest2)
{
	const double youngModulus = 30000 * ksiToPascal;
	const double poissonRatio = 0.25;
	const double thickness = 1.0  * inchToMeter;
	const double L = 48.0 * inchToMeter;
	const double h = 12.0 * inchToMeter;

	std::vector<Vector3d> nodes;
	nodes.push_back(Vector3d(-L / 2.0, - h / 2.0, 0.0)); // 0
	nodes.push_back(Vector3d(-L / 4.0, - h / 2.0, 0.0)); // 1
	nodes.push_back(Vector3d(     0.0, - h / 2.0, 0.0)); // 2
	nodes.push_back(Vector3d( L / 4.0, - h / 2.0, 0.0)); // 3
	nodes.push_back(Vector3d( L / 2.0, - h / 2.0, 0.0)); // 4
	nodes.push_back(Vector3d(-L / 2.0,   h / 2.0, 0.0)); // 5
	nodes.push_back(Vector3d(-L / 4.0,   h / 2.0, 0.0)); // 6
	nodes.push_back(Vector3d(     0.0,   h / 2.0, 0.0)); // 7
	nodes.push_back(Vector3d( L / 4.0,   h / 2.0, 0.0)); // 8
	nodes.push_back(Vector3d( L / 2.0,   h / 2.0, 0.0)); // 9

	// Subdivision
	nodes.push_back((nodes[0] + nodes[1]) * 0.5); // 10
	nodes.push_back((nodes[0] + nodes[6]) * 0.5); // 11
	nodes.push_back((nodes[0] + nodes[5]) * 0.5); // 12
	nodes.push_back((nodes[5] + nodes[6]) * 0.5); // 13
	nodes.push_back((nodes[1] + nodes[6]) * 0.5); // 14
	nodes.push_back((nodes[1] + nodes[2]) * 0.5); // 15
	nodes.push_back((nodes[1] + nodes[7]) * 0.5); // 16
	nodes.push_back((nodes[6] + nodes[7]) * 0.5); // 17
	nodes.push_back(Vector3d::Zero()); // 18
	nodes.push_back((nodes[2] + nodes[3]) * 0.5); // 19
	nodes.push_back((nodes[2] + nodes[8]) * 0.5); // 20
	nodes.push_back((nodes[7] + nodes[8]) * 0.5); // 21
	nodes.push_back((nodes[3] + nodes[8]) * 0.5); // 22
	nodes.push_back((nodes[3] + nodes[4]) * 0.5); // 23
	nodes.push_back((nodes[3] + nodes[9]) * 0.5); // 24
	nodes.push_back((nodes[8] + nodes[9]) * 0.5); // 25
	nodes.push_back((nodes[4] + nodes[9]) * 0.5); // 26

	std::vector<size_t> fixedNodes;
	fixedNodes.push_back(0);
	fixedNodes.push_back(5);
	fixedNodes.push_back(12);

	setNodePositions(nodes, fixedNodes);

	const int numTriangles = 32;
	const std::array<std::array<size_t, 3>, numTriangles> trianglesNodeIds =
	{{
		{{0, 10, 12}}, {{10, 11, 12}}, {{12, 11, 13}}, {{12, 13, 5}},
		{{10, 1, 11}}, {{1, 14, 11}}, {{11, 14, 6}}, {{11, 6, 13}},
		{{1, 15, 14}}, {{15, 16, 14}}, {{14, 16, 17}}, {{14, 17, 6}},
		{{15, 2, 16}}, {{2, 18, 16}}, {{16, 18, 7}}, {{16, 7, 17}},
		{{2, 19, 18}}, {{19, 20, 18}}, {{18, 20, 21}}, {{18, 21, 7}},
		{{19, 3, 20}}, {{3, 22, 20}}, {{20, 22, 8}}, {{20, 8, 21}},
		{{3, 23, 22}}, {{23, 24, 22}}, {{22, 24, 25}}, {{22, 25, 8}},
		{{23, 4, 24}}, {{4, 26, 24}}, {{24, 26, 9}}, {{24, 9, 25}}
	}};

	for (size_t triangleId = 0; triangleId < numTriangles; triangleId++)
	{
		addTriangle(trianglesNodeIds[triangleId], youngModulus, poissonRatio, thickness);
	}

	addPunctualLoad( 4, Vector3d(0,  6.67 * kipToNewton, 0));
	addPunctualLoad( 9, Vector3d(0,  6.67 * kipToNewton, 0));
	addPunctualLoad(26, Vector3d(0, 26.67 * kipToNewton, 0));

	solve();

	EXPECT_NEAR(-0.034271 * inchToMeter, getUx(9), epsilonCantilever2);
	EXPECT_NEAR( 0.194456 * inchToMeter, getUy(9), epsilonCantilever2);
	EXPECT_NEAR(-0.025605 * inchToMeter, getUx(7), epsilonCantilever2);
	EXPECT_NEAR( 0.062971 * inchToMeter, getUy(7), epsilonCantilever2);
}

/// Generic algorithm to define the triangles in between 2 arrays of consecutive node indices:
/// beginIndex2
///  *---1---2---3---4  array2
///  | \ | \ | \ | \ |
///  *---1---2---3---4  array1
/// beginIndex1
template <size_t M>
void defineTriangleStrips(size_t beginIndex1, size_t beginIndex2, size_t number,
						  std::array<std::array<size_t, 3>, M>* triangleLists, size_t* triangleId)
{
	for (size_t i = 0; i < number - 1; i++)
	{
		std::array<size_t, 3> triangle1 = {{beginIndex1 + i, beginIndex1 + i + 1, beginIndex2 + i}};
		std::array<size_t, 3> triangle2 = {{beginIndex1 + i + 1, beginIndex2 + i + 1, beginIndex2 + i}};
		(*triangleLists)[(*triangleId)++] = triangle1;
		(*triangleLists)[(*triangleId)++] = triangle2;
	}
}

/// Helper function to define the nodes of the membrane plate with semi-circular hole test
/// In this test, nodes are defined around concentric circles. This function adds the nodes that are along the same
/// axis (going from the most interior circle to the most exterior circle) for a given angle.
/// \param angle The angle for which the nodes will be computed and added
/// \param numNodes The number of nodes to add along this axis (defined by the angle)
/// \param L The length of the plate
/// \param radius The radius of the most internal circle
/// \param [in,out] nodes The list of nodes in which the new nodes will be added
/// \note The spacing between 2 consecutive nodes is constant equal to (L / 2.0 - radius) / 5.0
void membranePlateWithSemiCircularHoleAddNodesForAngle(double angle, size_t numNodes, double L, double radius,
													   std::vector<Vector3d>* nodes)
{
	for (size_t nodeId = 0; nodeId < numNodes; nodeId++)
	{
		double distance = radius + nodeId * (L / 2.0 - radius) / 5.0;
		(*nodes).push_back(Vector3d(distance * cos(angle), distance * sin(angle), 0.0));
	}
}

TEST_F(Fem2DMechanicalValidationTests, MembranePlateWithSemiCircularHoleTest)
{
	const double youngModulus = 30000 * ksiToPascal;
	const double poissonRatio = 0.3;
	const double thickness = 0.45 * inchToMeter;
	const double radius = 3.0 * inchToMeter;
	const double L = 16.0 * inchToMeter;
	const double h = 6.0 * inchToMeter;

	double startAngle = -M_PI / 2.0;
	double deltaAngle = M_PI / 12.0;

	std::vector<Vector3d> nodes;
	double angle, distance;

	// Node 0..5
	angle = startAngle + 0.0 * deltaAngle;
	membranePlateWithSemiCircularHoleAddNodesForAngle(angle, 6, L, radius, &nodes);

	// Node 6..10
	angle = startAngle + 1.0 * deltaAngle;
	membranePlateWithSemiCircularHoleAddNodesForAngle(angle, 5, L, radius, &nodes);
	// Node 11 (we have distance.sin(angle) = -L/2)
	distance = - L / (2.0 * sin(angle));
	nodes.push_back(Vector3d(distance * cos(angle), distance * sin(angle), 0.0)); // 11

	// Node 12..17
	angle = startAngle + 2.0 * deltaAngle;
	membranePlateWithSemiCircularHoleAddNodesForAngle(angle, 6, L, radius, &nodes);
	// Node 18 (we have distance.sin(angle) = -L/2)
	distance = - L / (2.0 * sin(angle));
	nodes.push_back(Vector3d(distance * cos(angle), distance * sin(angle), 0.0)); // 18

	// Node 19..23
	angle = startAngle + 3.0 * deltaAngle;
	membranePlateWithSemiCircularHoleAddNodesForAngle(angle, 5, L, radius, &nodes);
	// Node 24 (we have distance.cos(angle) = h)
	distance = h / cos(angle);
	nodes.push_back(Vector3d(distance * cos(angle), distance * sin(angle), 0.0)); // 24
	// Node 25 (corner)
	nodes.push_back(Vector3d(h, -L / 2.0, 0.0)); // 25

	// Node 26..29
	angle = startAngle + 4.0 * deltaAngle;
	membranePlateWithSemiCircularHoleAddNodesForAngle(angle, 4, L, radius, &nodes);
	// Node 30 (we have distance.cos(angle) = h)
	distance = h / cos(angle);
	nodes.push_back(Vector3d(distance * cos(angle), distance * sin(angle), 0.0)); // 30

	// Node 31..33
	angle = startAngle + 5.0 * deltaAngle;
	membranePlateWithSemiCircularHoleAddNodesForAngle(angle, 3, L, radius, &nodes);
	// Node 34 (we have distance.cos(angle) = h)
	distance = h / cos(angle);
	nodes.push_back(Vector3d(distance * cos(angle), distance * sin(angle), 0.0)); // 34

	// Node 35..38
	angle = startAngle + 6.0 * deltaAngle; // (=0)
	membranePlateWithSemiCircularHoleAddNodesForAngle(angle, 4, L, radius, &nodes);

	// Node 39..41
	angle = startAngle + 7.0 * deltaAngle;
	membranePlateWithSemiCircularHoleAddNodesForAngle(angle, 3, L, radius, &nodes);
	// Node 42 (we have distance.cos(angle) = h)
	distance = h / cos(angle);
	nodes.push_back(Vector3d(distance * cos(angle), distance * sin(angle), 0.0)); // 42

	// Node 43..46
	angle = startAngle + 8.0 * deltaAngle;
	membranePlateWithSemiCircularHoleAddNodesForAngle(angle, 4, L, radius, &nodes);
	// Node 47 (we have distance.cos(angle) = h)
	distance = h / cos(angle);
	nodes.push_back(Vector3d(distance * cos(angle), distance * sin(angle), 0.0)); // 47

	// Node 48..52
	angle = startAngle + 9.0 * deltaAngle;
	membranePlateWithSemiCircularHoleAddNodesForAngle(angle, 5, L, radius, &nodes);
	// Node 53 (we have distance.cos(angle) = h)
	distance = h / cos(angle);
	nodes.push_back(Vector3d(distance * cos(angle), distance * sin(angle), 0.0)); // 53
	// Node 54 (corner)
	nodes.push_back(Vector3d(h, L / 2.0, 0.0)); // 54

	// Node 55..60
	angle = startAngle + 10.0 * deltaAngle;
	membranePlateWithSemiCircularHoleAddNodesForAngle(angle, 6, L, radius, &nodes);
	// Node 61 (we have distance.sin(angle) = L / 2)
	distance = L / (2.0 * sin(angle));
	nodes.push_back(Vector3d(distance * cos(angle), distance * sin(angle), 0.0)); // 61

	// Node 62..66
	angle = startAngle + 11.0 * deltaAngle;
	membranePlateWithSemiCircularHoleAddNodesForAngle(angle, 5, L, radius, &nodes);
	// Node 67 (we have distance.sin(angle) = L / 2)
	distance = L / (2.0 * sin(angle));
	nodes.push_back(Vector3d(distance * cos(angle), distance * sin(angle), 0.0)); // 67

	// Node 69..73
	angle = startAngle + 12.0 * deltaAngle;  // (= M_PI / 2)
	membranePlateWithSemiCircularHoleAddNodesForAngle(angle, 6, L, radius, &nodes);

	std::vector<size_t> fixedNodes;
	fixedNodes.push_back(54);
	fixedNodes.push_back(61);
	fixedNodes.push_back(67);
	fixedNodes.push_back(73);

	setNodePositions(nodes, fixedNodes);

	const int numTriangles = 110;
	std::array<std::array<size_t, 3>, numTriangles> trianglesNodeIds;
	size_t triangleId = 0;
	defineTriangleStrips(0, 6, 6, &trianglesNodeIds, &triangleId);
	defineTriangleStrips(6, 12, 6, &trianglesNodeIds, &triangleId);
	{
		std::array<size_t, 3> triangle = {{11, 18, 17}};
		trianglesNodeIds[triangleId++] = triangle;
	}
	defineTriangleStrips(12, 19, 7, &trianglesNodeIds, &triangleId);
	defineTriangleStrips(19, 26, 5, &trianglesNodeIds, &triangleId);
	{
		std::array<size_t, 3> triangle = {{23, 24, 30}};
		trianglesNodeIds[triangleId++] = triangle;
	}
	defineTriangleStrips(26, 31, 4, &trianglesNodeIds, &triangleId);
	{
		std::array<size_t, 3> triangle = {{29, 30, 34}};
		trianglesNodeIds[triangleId++] = triangle;
	}
	defineTriangleStrips(31, 35, 4, &trianglesNodeIds, &triangleId);
	defineTriangleStrips(35, 39, 4, &trianglesNodeIds, &triangleId);
	defineTriangleStrips(39, 43, 4, &trianglesNodeIds, &triangleId);
	{
		std::array<size_t, 3> triangle = {{42, 47, 46}};
		trianglesNodeIds[triangleId++] = triangle;
	}
	defineTriangleStrips(43, 48, 5, &trianglesNodeIds, &triangleId);
	{
		std::array<size_t, 3> triangle = {{47, 53, 52}};
		trianglesNodeIds[triangleId++] = triangle;
	}
	defineTriangleStrips(48, 55, 7, &trianglesNodeIds, &triangleId);
	defineTriangleStrips(55, 62, 6, &trianglesNodeIds, &triangleId);
	{
		std::array<size_t, 3> triangle = {{60, 61, 67}};
		trianglesNodeIds[triangleId++] = triangle;
	}
	defineTriangleStrips(62, 68, 6, &trianglesNodeIds, &triangleId);

	for (size_t triangleId = 0; triangleId < numTriangles; triangleId++)
	{
		addTriangle(trianglesNodeIds[triangleId], youngModulus, poissonRatio, thickness);
	}

	addPunctualLoad( 5, Vector3d(0, -1.0 * kipToNewton, 0));
	addPunctualLoad(11, Vector3d(0, -1.0 * kipToNewton, 0));
	addPunctualLoad(18, Vector3d(0, -1.0 * kipToNewton, 0));
	addPunctualLoad(25, Vector3d(0, -1.0 * kipToNewton, 0));

	solve();

	EXPECT_NEAR( 0.001545 * inchToMeter, getUx(0), epsilonPlateWithSemiCirculatHole);
	EXPECT_NEAR(-0.003132 * inchToMeter, getUy(0), epsilonPlateWithSemiCirculatHole);
	EXPECT_NEAR( 0.004213 * inchToMeter, getUx(5), epsilonPlateWithSemiCirculatHole);
	EXPECT_NEAR(-0.003355 * inchToMeter, getUy(5), epsilonPlateWithSemiCirculatHole);
}

TEST_F(Fem2DMechanicalValidationTests, PlateBendingSquarePlateMeshPatternATest)
{
	const double youngModulus = 3600 * ksiToPascal;
	const double poissonRatio = 0.2;
	const double thickness = 6.0  * inchToMeter;
	const double L = 144.0 * inchToMeter;
	const double deltaL = L / 8.0;

	std::vector<Vector3d> nodes;
	std::vector<size_t> fixedNodes;
	for (size_t Y = 0; Y < 9; Y++)
	{
		for (size_t X = 0; X < 9; X++)
		{
			nodes.push_back(Vector3d(-L / 2.0 + deltaL * X, - L / 2.0 + deltaL * Y, 0.0));
		}
	}
	for (size_t i = 0; i < 9; i++)
	{
		// 1st edge along X
		fixedNodes.push_back(i); // Nodes 0..8
		// last edge along X
		fixedNodes.push_back(9 * 8 + i); // Nodes 72..80
	}
	for (size_t i = 1; i < 8; i++)
	{
		// 1st edge along Y
		fixedNodes.push_back(9 * i); // Nodes 9, 18, 27, 36, 45, 54, 63
		// last edge along Y
		fixedNodes.push_back(9 * i + 8); // Nodes 17, 26, 35, 44, 53, 62
	}
	setNodePositions(nodes, fixedNodes);

	const int numTriangles = 128;
	std::array<std::array<size_t, 3>, numTriangles> trianglesNodeIds;
	size_t triangleId = 0;
	for(size_t Y = 0; Y < 8; Y++)
	{
		for(size_t X = 0; X < 8; X++)
		{
			std::array<size_t, 3> triangle1 = {{Y * 9 + X, Y * 9 + (X + 1), (Y + 1) * 9 + (X + 1)}};
			trianglesNodeIds[triangleId++] = triangle1;
			std::array<size_t, 3> triangle2 = {{Y * 9 + X, (Y + 1) * 9 + (X + 1), (Y + 1) * 9 + X}};
			trianglesNodeIds[triangleId++] = triangle2;
		}
	}
	for (size_t triangleId = 0; triangleId < numTriangles; triangleId++)
	{
		addTriangle(trianglesNodeIds[triangleId], youngModulus, poissonRatio, thickness);
	}

	addUniformSurfaceLoad(Vector3d(0.0, 0.0, -0.1) * ksiToPascal);

	solve();

	EXPECT_NEAR(-0.82920 * inchToMeter, getUz(40), epsilonPlateBending);
	EXPECT_NEAR(-0.304909 * inchToMeter, getUz(20), epsilonPlateBending);
}

TEST_F(Fem2DMechanicalValidationTests, PlateBendingSquarePlateMeshPatternBTest)
{
	const double youngModulus = 3600 * ksiToPascal;
	const double poissonRatio = 0.2;
	const double thickness = 6.0  * inchToMeter;
	const double L = 144.0 * inchToMeter;
	const double deltaL = L / 8.0;

	std::vector<Vector3d> nodes;
	std::vector<size_t> fixedNodes;
	for (size_t Y = 0; Y < 9; Y++)
	{
		for (size_t X = 0; X < 9; X++)
		{
			nodes.push_back(Vector3d(-L / 2.0 + deltaL * X, - L / 2.0 + deltaL * Y, 0.0));
		}
	}
	for (size_t i = 0; i < 9; i++)
	{
		// 1st edge along X
		fixedNodes.push_back(i); // Nodes 0..8
		// last edge along X
		fixedNodes.push_back(9 * 8 + i); // Nodes 72..80
	}
	for (size_t i = 1; i < 8; i++)
	{
		// 1st edge along Y
		fixedNodes.push_back(9 * i); // Nodes 9, 18, 27, 36, 45, 54, 63
		// last edge along Y
		fixedNodes.push_back(9 * i + 8); // Nodes 17, 26, 35, 44, 53, 62
	}
	setNodePositions(nodes, fixedNodes);

	const int numTriangles = 128;
	std::array<std::array<size_t, 3>, numTriangles> trianglesNodeIds;
	size_t triangleId = 0;
	for(size_t Y = 0; Y < 8; Y++)
	{
		for(size_t X = 0; X < 8; X++)
		{
			std::array<size_t, 3> triangle1 = {{Y * 9 + X, Y * 9 + (X + 1), (Y + 1) * 9 + X}};
			trianglesNodeIds[triangleId++] = triangle1;
			std::array<size_t, 3> triangle2 = {{Y * 9 + (X + 1), (Y + 1) * 9 + (X + 1), (Y + 1) * 9 + X}};
			trianglesNodeIds[triangleId++] = triangle2;
		}
	}
	for (size_t triangleId = 0; triangleId < numTriangles; triangleId++)
	{
		addTriangle(trianglesNodeIds[triangleId], youngModulus, poissonRatio, thickness);
	}

	addUniformSurfaceLoad(Vector3d(0.0, 0.0, -0.1) * ksiToPascal);

	solve();

	EXPECT_NEAR(-0.82920 * inchToMeter, getUz(40), epsilonPlateBending);
	EXPECT_NEAR(-0.30010 * inchToMeter, getUz(20), epsilonPlateBending);
}

TEST_F(Fem2DMechanicalValidationTests, CantileverPlateTest)
{
	const double youngModulus = 3600 * ksiToPascal;
	const double poissonRatio = 0.2;
	const double thickness = 6.0  * inchToMeter;
	const double L = 96.0 * inchToMeter;
	const double deltaL = L / 8;

	std::vector<Vector3d> nodes;
	std::vector<size_t> fixedNodes;
	for (size_t Y = 0; Y < 9; Y++)
	{
		for (size_t X = 0; X < 9; X++)
		{
			nodes.push_back(Vector3d(-L / 2.0 + deltaL * X, - L / 2.0 + deltaL * Y, 0.0));
		}
	}
	for (size_t i = 0; i < 9; i++)
	{
		// 1st edge along X
		fixedNodes.push_back(i); // Nodes 0..8
	}
	setNodePositions(nodes, fixedNodes);

	const int numTriangles = 128;
	std::array<std::array<size_t, 3>, numTriangles> trianglesNodeIds;
	size_t triangleId = 0;
	for(size_t Y = 0; Y < 8; Y++)
	{
		for(size_t X = 0; X < 8; X++)
		{
			std::array<size_t, 3> triangle1 = {{Y * 9 + X, Y * 9 + (X + 1), (Y + 1) * 9 + X}};
			trianglesNodeIds[triangleId++] = triangle1;
			std::array<size_t, 3> triangle2 = {{Y * 9 + (X + 1), (Y + 1) * 9 + (X + 1), (Y + 1) * 9 + X}};
			trianglesNodeIds[triangleId++] = triangle2;
		}
	}
	for (size_t triangleId = 0; triangleId < numTriangles; triangleId++)
	{
		addTriangle(trianglesNodeIds[triangleId], youngModulus, poissonRatio, thickness);
	}

	addUniformSurfaceLoad(Vector3d(0.0, 0.0, -0.01) * ksiToPascal);

	solve();

	// Expect 5% error max (in the Thesis, the author gets 4.96% error)
	const double epsilonNode76 = 1.68787 * inchToMeter * 0.05;
	EXPECT_NEAR(-1.68787 * inchToMeter, getUz(76), epsilonNode76);

	// Expect 5% error max (in the Thesis, the author gets 4.98% error)
	const double epsilonNode40 = 0.599412 * inchToMeter * 0.05;
	EXPECT_NEAR(-0.599412 * inchToMeter, getUz(40), epsilonNode40);
}

} // namespace Physics

} // namespace SurgSim
