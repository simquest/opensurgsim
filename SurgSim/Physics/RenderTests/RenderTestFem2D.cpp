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

///\file RenderTestFem2D.cpp render test for Fem2D

#include <memory>

#include "SurgSim/Blocks/TransferPhysicsToGraphicsMeshBehavior.h"
#include "SurgSim/Blocks/TransferPhysicsToPointCloudBehavior.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Graphics/Mesh.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem2DRepresentation.h"
#include "SurgSim/Physics/Fem2DElementTriangle.h"
#include "SurgSim/Physics/RenderTests/RenderTest.h"

using SurgSim::Blocks::TransferPhysicsToGraphicsMeshBehavior;
using SurgSim::Blocks::TransferPhysicsToPointCloudBehavior;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Graphics::OsgPointCloudRepresentation;
using SurgSim::Math::Vector3d;
using SurgSim::Physics::Fem2DRepresentation;
using SurgSim::Physics::Fem2DElementTriangle;

namespace
{

/// Create a Fem2D with a cylinder shape
/// \note This is defining a cylinder based on cylindrical coordinates M(length, angle)
/// \note The cylinder is composed of cross-sections with nodes added radially to each cross-section.
/// \note The nodes of 2 consecutives cross-sections are connected to form square-patches which in turn
/// \note are decomposed into 2 Fem2DElementTriangle.
void createFem2DCylinder(std::shared_ptr<Fem2DRepresentation> physicsRepresentation)
{
	// Mechanical properties
	const double youngModulus = 1e6;
	const double poissonRatio = 0.35;
	const double massDensity = 5000.0;
	// Geometrical properties
	const double length = 1.0;
	const double radius = 8e-2;
	const double thickness = 3e-2;
	// Number of cross-sections and their discretization in nodes
	const size_t numSections = 7;
	const size_t numNodesOnSection = 8;
	const size_t numNodes = numSections * numNodesOnSection;

	// Distance between 2 consecutive cross-section
	const double deltaL = length / (numSections - 1);
	// Angle between 2 consecutive nodes on a cross-section
	const double deltaAngle = 2.0 * M_PI / numNodesOnSection;

	std::shared_ptr<SurgSim::Math::OdeState> restState = std::make_shared<SurgSim::Math::OdeState>();
	restState->setNumDof(physicsRepresentation->getNumDofPerNode(), numNodes);

	// Sets the initial state (node positions and boundary conditions)
	SurgSim::Math::Vector& x = restState->getPositions();
	size_t numDofPerNode = physicsRepresentation->getNumDofPerNode();
	for (size_t sectionId = 0; sectionId < numSections; sectionId++)
	{
		for (size_t nodeIdOnSection = 0; nodeIdOnSection < numNodesOnSection; nodeIdOnSection++)
		{
			double angle = deltaAngle * nodeIdOnSection;
			x.segment<3>(numDofPerNode * (sectionId * numNodesOnSection + nodeIdOnSection)) =
				Vector3d(-length / 2.0 + sectionId * deltaL, radius * cos(angle), radius * sin(angle));
		}
	}
	// We fix the nodes on the 1st and last cross-sections
	const size_t section0 = 0;
	const size_t section1 = numSections - 1;
	for (size_t nodeId = 0; nodeId < numNodesOnSection; nodeId++)
	{
		restState->addBoundaryCondition(nodeId + numNodesOnSection * section0);
		restState->addBoundaryCondition(nodeId + numNodesOnSection * section1);
	}
	physicsRepresentation->setInitialState(restState);

	// Adds all the FemElements
	for (size_t sectionId = 0; sectionId < numSections - 1; sectionId++)
	{
		// For each cross-section, we connect the nodes of this cross-section to the nodes of the next cross-section

		for (size_t nodeIdOnSection = 0; nodeIdOnSection < numNodesOnSection; nodeIdOnSection++)
		{
			// On a given cross-section, each node will be connected to the next node
			// The last node is connected to the 1st node via a modulo in the node index calculation

			std::array<std::array<size_t, 2>, 2> nodeIds =
			{{
				{{
					sectionId * numNodesOnSection + nodeIdOnSection,
					sectionId * numNodesOnSection + (nodeIdOnSection + 1) % numNodesOnSection
				}}
				,
				{{
					(sectionId + 1) * numNodesOnSection + nodeIdOnSection,
					(sectionId + 1) * numNodesOnSection + (nodeIdOnSection + 1) % numNodesOnSection
				}}
			}};
			std::array<size_t, 3> triangle1NodeIds = {{nodeIds[0][0], nodeIds[0][1], nodeIds[1][1]}};
			std::shared_ptr<Fem2DElementTriangle> triangle1 = std::make_shared<Fem2DElementTriangle>(triangle1NodeIds);
			triangle1->setThickness(thickness);
			triangle1->setMassDensity(massDensity);
			triangle1->setPoissonRatio(poissonRatio);
			triangle1->setYoungModulus(youngModulus);
			physicsRepresentation->addFemElement(triangle1);

			std::array<size_t, 3> triangle2NodeIds = {{nodeIds[0][0], nodeIds[1][1], nodeIds[1][0]}};
			std::shared_ptr<Fem2DElementTriangle> triangle2 = std::make_shared<Fem2DElementTriangle>(triangle2NodeIds);
			triangle2->setThickness(thickness);
			triangle2->setMassDensity(massDensity);
			triangle2->setPoissonRatio(poissonRatio);
			triangle2->setYoungModulus(youngModulus);
			physicsRepresentation->addFemElement(triangle2);
		}
	}
}

// Generates a 2d fem comprised of a cylinder. The number of fem elements is determined by createFem2DCylinder.
std::shared_ptr<SurgSim::Framework::SceneElement> createFem2D(const std::string& name,
		const SurgSim::Math::RigidTransform3d& gfxPose,
		const SurgSim::Math::Vector4d& color,
		SurgSim::Math::IntegrationScheme integrationScheme)
{
	std::shared_ptr<Fem2DRepresentation> physicsRepresentation
		= std::make_shared<Fem2DRepresentation>("Physics Representation");

	// In this test, the physics representations are not transformed, only the graphics will be transformed
	createFem2DCylinder(physicsRepresentation);

	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(1e0);
	physicsRepresentation->setRayleighDampingStiffness(1e-3);

	std::shared_ptr<BasicSceneElement> femSceneElement = std::make_shared<BasicSceneElement>(name);
	femSceneElement->addComponent(physicsRepresentation);

	// Create a triangle mesh for visualizing the surface of the finite element model
	std::shared_ptr<SurgSim::Graphics::OsgMeshRepresentation> graphicsTriangleMeshRepresentation
		= std::make_shared<SurgSim::Graphics::OsgMeshRepresentation>("TriangleMesh Representation");
	graphicsTriangleMeshRepresentation->setLocalPose(gfxPose);
	auto mesh = graphicsTriangleMeshRepresentation->getMesh();
	// Create vertices
	for (size_t vertexId = 0; vertexId < physicsRepresentation->getInitialState()->getNumNodes(); vertexId++)
	{
		SurgSim::Graphics::Mesh::VertexType v(physicsRepresentation->getInitialState()->getPosition(vertexId));
		mesh->addVertex(v);
	}
	// Create triangles
	for (size_t triangleId = 0; triangleId < physicsRepresentation->getNumFemElements(); triangleId++)
	{
		auto nodeIdsVector = physicsRepresentation->getFemElement(triangleId)->getNodeIds();
		std::array<size_t, 3> nodeIds = {{nodeIdsVector[0], nodeIdsVector[1], nodeIdsVector[2]}};
		SurgSim::Graphics::Mesh::TriangleType t(nodeIds);
		mesh->addTriangle(t);
	}
	femSceneElement->addComponent(graphicsTriangleMeshRepresentation);

	// Create a behavior which transfers the position of the vertices in the FEM to locations in the triangle mesh
	auto physicsToMesh =
		std::make_shared<SurgSim::Blocks::TransferPhysicsToGraphicsMeshBehavior>("physics to triangle mesh");
	physicsToMesh->setSource(physicsRepresentation);
	physicsToMesh->setTarget(graphicsTriangleMeshRepresentation);
	femSceneElement->addComponent(physicsToMesh);

	std::shared_ptr<SurgSim::Graphics::PointCloudRepresentation> graphicsPointCloudRepresentation
			= std::make_shared<OsgPointCloudRepresentation>("PointCloud Representation");
	graphicsPointCloudRepresentation->setLocalPose(gfxPose);
	graphicsPointCloudRepresentation->setColor(color);
	graphicsPointCloudRepresentation->setPointSize(3.0f);
	graphicsPointCloudRepresentation->setLocalActive(true);
	femSceneElement->addComponent(graphicsPointCloudRepresentation);

	auto physicsToPointCloud =
		std::make_shared<TransferPhysicsToPointCloudBehavior>("Transfer from Physics to Graphics point cloud");
	physicsToPointCloud->setSource(physicsRepresentation);
	physicsToPointCloud->setTarget(graphicsPointCloudRepresentation);
	femSceneElement->addComponent(physicsToPointCloud);

	return femSceneElement;
}

}; // anonymous namespace

namespace SurgSim
{

namespace Physics
{

TEST_F(RenderTests, VisualTestFem2D)
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::Vector4d;

	const SurgSim::Math::Quaterniond quaternion(Eigen::AngleAxisd(-M_PI / 6.0, Vector3d(0.0, 1.0, 0.0)));

	scene->addSceneElement(
		createFem2D("Euler Explicit",                                          // name
					makeRigidTransform(quaternion, Vector3d(0.0, 0.6, 0.0)),   // graphics pose (rot., trans.)
					Vector4d(1, 0, 0, 1),                                      // color (r, g, b, a)
					SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_EXPLICIT));  // technique to update object

	scene->addSceneElement(
		createFem2D("Modified Euler Explicit",
					makeRigidTransform(quaternion, Vector3d(0.0, 0.3, 0.0)),
					Vector4d(0.5, 0, 0, 1),
					SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_EXPLICIT_MODIFIED));

	scene->addSceneElement(
		createFem2D("Runge Kutta 4",
					makeRigidTransform(quaternion, Vector3d(0.0, 0.0, 0.0)),
					Vector4d(0, 1, 0, 1),
					SurgSim::Math::INTEGRATIONSCHEME_LINEAR_RUNGE_KUTTA_4));

	scene->addSceneElement(
		createFem2D("Euler Implicit",
					makeRigidTransform(quaternion, Vector3d(0.0, -0.3, 0.0)),
					Vector4d(0, 0, 1, 1),
					SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_IMPLICIT));

	scene->addSceneElement(
		createFem2D("Static",
					makeRigidTransform(quaternion, Vector3d(0.0, -0.6, 0.0)),
					Vector4d(1, 1, 1, 1),
					SurgSim::Math::INTEGRATIONSCHEME_LINEAR_STATIC));

	runTest(Vector3d(0.0, 0.0, 2.0), Vector3d::Zero(), 5000.0);
}

}; // namespace Physics

}; // namespace SurgSim
