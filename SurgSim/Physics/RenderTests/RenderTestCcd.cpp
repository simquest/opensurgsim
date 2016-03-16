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

///\file RenderTestCcd.cpp render test for Continuous Collision Detection

#include <memory>

#include "SurgSim/DataStructures/SegmentMesh.h"
#include "SurgSim/Blocks/TransferPhysicsToGraphicsMeshBehavior.h"
#include "SurgSim/Blocks/TransferPhysicsToPointCloudBehavior.h"
#include "SurgSim/Blocks/TransferPhysicsToVerticesBehavior.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Graphics/Mesh.h"
#include "SurgSim/Graphics/OsgCurveRepresentation.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/SegmentMeshShape.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/DeformableCollisionRepresentation.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/Fem1DElementBeam.h"
#include "SurgSim/Physics/Fem2DRepresentation.h"
#include "SurgSim/Physics/Fem2DElementTriangle.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RenderTests/RenderTest.h"

using SurgSim::DataStructures::SegmentMeshPlain;
using SurgSim::Blocks::TransferPhysicsToGraphicsMeshBehavior;
using SurgSim::Blocks::TransferPhysicsToPointCloudBehavior;
using SurgSim::Blocks::TransferPhysicsToVerticesBehavior;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Graphics::OsgCurveRepresentation;
using SurgSim::Math::Vector3d;
using SurgSim::Physics::DeformableCollisionRepresentation;
using SurgSim::Physics::FixedRepresentation;
using SurgSim::Physics::Fem1DRepresentation;
using SurgSim::Physics::Fem1DElementBeam;
using SurgSim::Physics::Fem2DRepresentation;
using SurgSim::Physics::Fem2DElementTriangle;
using SurgSim::Physics::RigidCollisionRepresentation;

namespace
{

void loadModelFem1D(std::shared_ptr<Fem1DRepresentation> physicsRepresentation,
					std::shared_ptr<DeformableCollisionRepresentation> collisionRepresentation,
					size_t numNodes,
					const SurgSim::Math::RigidTransform3d& pose)
{
	std::shared_ptr<SurgSim::Math::OdeState> restState = std::make_shared<SurgSim::Math::OdeState>();
	restState->setNumDof(physicsRepresentation->getNumDofPerNode(), numNodes);

	auto mesh = std::make_shared<SegmentMeshPlain>();

	// Sets the initial state (node positions and boundary conditions)
	SurgSim::Math::Vector& x = restState->getPositions();
	Vector3d vertex;
	for (size_t nodeId = 0; nodeId < numNodes; nodeId++)
	{
		vertex = pose * Vector3d(static_cast<double>(nodeId) / static_cast<double>(numNodes - 1) - 0.5, 0.0, 0.0);

		SurgSim::Math::getSubVector(x, nodeId, physicsRepresentation->getNumDofPerNode()).segment<3>(0) = vertex;

		SegmentMeshPlain::VertexType meshVertex;
		meshVertex.position = vertex;
		mesh->addVertex(meshVertex);
	}

	physicsRepresentation->setInitialState(restState);

	// Adds all the FemElements
	for (size_t beamId = 0; beamId < numNodes - 1; beamId++)
	{
		std::array<size_t, 2> beamNodeIds = {{beamId, beamId + 1}};
		std::shared_ptr<Fem1DElementBeam> beam = std::make_shared<Fem1DElementBeam>(beamNodeIds);
		beam->setRadius(0.10);
		beam->setMassDensity(3000.0);
		beam->setPoissonRatio(0.45);
		beam->setYoungModulus(1e6);
		physicsRepresentation->addFemElement(beam);

		SegmentMeshPlain::EdgeType meshEdge(beamNodeIds);
		mesh->addEdge(meshEdge);
	}

	auto meshShape = std::make_shared<SurgSim::Math::SegmentMeshShape>(*mesh, 0.00001);
	collisionRepresentation->setShape(meshShape);
}

// Generates a 1d fem comprised of adjacent elements along a straight line.  The number of fem elements is determined
// by loadModelFem1D.
std::shared_ptr<SurgSim::Framework::SceneElement> createFem1D(const std::string& name,
	const SurgSim::Math::RigidTransform3d& pose,
	const SurgSim::Math::Vector4d& color,
	SurgSim::Math::IntegrationScheme integrationScheme)
{
	auto physicsRepresentation = std::make_shared<Fem1DRepresentation>("Physics Representation: " + name);
	auto collisionRepresentation = std::make_shared<DeformableCollisionRepresentation>("Collision " + name);
	collisionRepresentation->setCollisionDetectionType(SurgSim::Collision::COLLISION_DETECTION_TYPE_CONTINUOUS);

	loadModelFem1D(physicsRepresentation, collisionRepresentation, 10, pose);
	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);

	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(5e-2);
	physicsRepresentation->setRayleighDampingStiffness(5e-3);

	auto femSceneElement = std::make_shared<BasicSceneElement>(name);
	femSceneElement->addComponent(physicsRepresentation);
	femSceneElement->addComponent(collisionRepresentation);

	auto graphicsRepresentation = std::make_shared<OsgCurveRepresentation>("Graphics Representation: " + name);
	graphicsRepresentation->setColor(color);

	femSceneElement->addComponent(graphicsRepresentation);

	auto copier = std::make_shared<SurgSim::Blocks::TransferPhysicsToVerticesBehavior>("Copier");
	copier->setSource(physicsRepresentation);
	copier->setTarget(graphicsRepresentation);
	femSceneElement->addComponent(copier);

	return femSceneElement;
}

/// Create a Fem2D with a cylinder shape
/// \note This is defining a cylinder based on cylindrical coordinates M(length, angle)
/// \note The cylinder is composed of cross-sections with nodes added radially to each cross-section.
/// \note The nodes of 2 consecutive cross-sections are connected to form square-patches which in turn
/// \note are decomposed into 2 Fem2DElementTriangle.
void createFem2DCylinder(std::shared_ptr<Fem2DRepresentation> physicsRepresentation,
						 std::shared_ptr<DeformableCollisionRepresentation> collisionRepresentation,
						 const SurgSim::Math::RigidTransform3d& pose)
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

	auto mesh = std::make_shared<SurgSim::DataStructures::TriangleMeshPlain>();

	// Sets the initial state (node positions and boundary conditions)
	SurgSim::Math::Vector& x = restState->getPositions();
	size_t numDofPerNode = physicsRepresentation->getNumDofPerNode();
	Vector3d vertex;
	for (size_t sectionId = 0; sectionId < numSections; sectionId++)
	{
		for (size_t nodeIdOnSection = 0; nodeIdOnSection < numNodesOnSection; nodeIdOnSection++)
		{
			double angle = deltaAngle * nodeIdOnSection;
			vertex = pose * Vector3d(-length / 2.0 + sectionId * deltaL, radius * cos(angle), radius * sin(angle));
			x.segment<3>(numDofPerNode * (sectionId * numNodesOnSection + nodeIdOnSection)) = vertex;

			SurgSim::DataStructures::TriangleMeshPlain::VertexType meshVertex;
			meshVertex.position = vertex;
			mesh->addVertex(meshVertex);
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

			SurgSim::DataStructures::TriangleMeshPlain::TriangleType meshTriangle1(triangle1NodeIds);
			mesh->addTriangle(meshTriangle1);

			SurgSim::DataStructures::TriangleMeshPlain::TriangleType meshTriangle2(triangle2NodeIds);
			mesh->addTriangle(meshTriangle2);
		}
	}

	auto meshShape = std::make_shared<SurgSim::Math::MeshShape>(*mesh);
	collisionRepresentation->setShape(meshShape);
}

// Generates a 2d fem comprised of a cylinder. The number of fem elements is determined by createFem2DCylinder.
std::shared_ptr<SurgSim::Framework::SceneElement> createFem2D(const std::string& name,
															  const SurgSim::Math::RigidTransform3d& pose,
															  const SurgSim::Math::Vector4d& color,
															  SurgSim::Math::IntegrationScheme integrationScheme)
{
	std::shared_ptr<Fem2DRepresentation> physicsRepresentation
		= std::make_shared<Fem2DRepresentation>("Physics Representation" + name);
	auto collisionRepresentation = std::make_shared<DeformableCollisionRepresentation>("Collision " + name);
	collisionRepresentation->setCollisionDetectionType(SurgSim::Collision::COLLISION_DETECTION_TYPE_CONTINUOUS);

	// In this test, the physics representations are not transformed, only the graphics will be transformed
	createFem2DCylinder(physicsRepresentation, collisionRepresentation, pose);

	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);
	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(1e0);
	physicsRepresentation->setRayleighDampingStiffness(1e-3);

	std::shared_ptr<BasicSceneElement> femSceneElement = std::make_shared<BasicSceneElement>(name);
	femSceneElement->addComponent(physicsRepresentation);
	femSceneElement->addComponent(collisionRepresentation);

	// Create a triangle mesh for visualizing the surface of the finite element model
	std::shared_ptr<SurgSim::Graphics::OsgMeshRepresentation> graphicsTriangleMeshRepresentation
		= std::make_shared<SurgSim::Graphics::OsgMeshRepresentation>("TriangleMesh Representation");
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

	return femSceneElement;
}

} // anonymous namespace

namespace SurgSim
{

namespace Physics
{

TEST_F(RenderTests, SegmentMeshTriangleMeshCcd1)
{
	using SurgSim::Math::makeRigidTranslation;
	using SurgSim::Math::Vector4d;

	scene->addSceneElement(
		createFem1D("Euler Implicit",
		makeRigidTranslation(Vector3d(0.0, 0.2, 0.1)),
		Vector4d(0, 0, 1, 1),
		SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_IMPLICIT));

	const SurgSim::Math::Quaterniond quaternion(Eigen::AngleAxisd(-M_PI / 6.0, Vector3d(0.0, 1.0, 0.0)));

	scene->addSceneElement(
		createFem2D("Euler Implicit",
		SurgSim::Math::makeRigidTransform(quaternion, Vector3d(0.0, -0.4, 0.0)),
		Vector4d(0, 0, 1, 1),
		SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_IMPLICIT));

	runTest(Vector3d(0.0, 0.0, 2.0), Vector3d::Zero(), 5000.0);
}

TEST_F(RenderTests, SegmentMeshTriangleMeshCcd2)
{
	using SurgSim::Math::makeRigidTranslation;
	using SurgSim::Math::Vector4d;

	scene->addSceneElement(
		createFem1D("Euler Implicit",
		makeRigidTranslation(Vector3d(0.0, 0.2, 0.1)),
		Vector4d(0, 0, 1, 1),
		SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_IMPLICIT));

	const SurgSim::Math::Quaterniond quaternion(Eigen::AngleAxisd(-M_PI / 2.0, Vector3d(1.0, 0.0, 0.0)));
	SurgSim::Math::RigidTransform3d pose =
		SurgSim::Math::makeRigidTransform(quaternion, SurgSim::Math::Vector3d(0.0, -0.4, 0.0));

	auto rigidSceneElement = std::make_shared<SurgSim::Framework::BasicSceneElement>("RigidMesh");
	rigidSceneElement->setPose(pose);
	auto shape = std::make_shared<SurgSim::Math::MeshShape>();
	shape->load("box_bowl.ply");

	auto phxRigid = std::make_shared<FixedRepresentation>("Physics");
	phxRigid->setDensity(5800.0); // Cedar of Lebanon wood density 5800.0 Kg/m-3
	phxRigid->setShape(shape);
	rigidSceneElement->addComponent(phxRigid);

	auto collision = std::make_shared<RigidCollisionRepresentation>("Collision");
	collision->setCollisionDetectionType(SurgSim::Collision::COLLISION_DETECTION_TYPE_CONTINUOUS);
	phxRigid->setCollisionRepresentation(collision);
	collision->setShape(shape);
	rigidSceneElement->addComponent(collision);

	auto osgRepresentation = std::make_shared<SurgSim::Graphics::OsgMeshRepresentation>("OsgRepresentation");
	osgRepresentation->setShape(shape);
	rigidSceneElement->addComponent(osgRepresentation);

	scene->addSceneElement(rigidSceneElement);

	runTest(Vector3d(0.0, 0.0, 2.0), Vector3d::Zero(), 5000.0);
}

} // namespace Physics

} // namespace SurgSim
