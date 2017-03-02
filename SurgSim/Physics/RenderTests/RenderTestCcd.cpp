// This file is a part of the OpenSurgSim project.
// Copyright 2016, SimQuest Solutions Inc.
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
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/Fem2DElementTriangle.h"
#include "SurgSim/Physics/Fem2DRepresentation.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RenderTests/RenderTest.h"

using SurgSim::DataStructures::SegmentMeshPlain;
using SurgSim::Blocks::TransferPhysicsToGraphicsMeshBehavior;
using SurgSim::Blocks::TransferPhysicsToVerticesBehavior;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Graphics::OsgCurveRepresentation;
using SurgSim::Graphics::OsgMeshRepresentation;
using SurgSim::Math::Vector3d;
using SurgSim::Physics::DeformableCollisionRepresentation;
using SurgSim::Physics::Fem1DRepresentation;
using SurgSim::Physics::Fem2DElementTriangle;
using SurgSim::Physics::Fem2DRepresentation;
using SurgSim::Physics::FixedRepresentation;
using SurgSim::Physics::RigidCollisionRepresentation;

namespace
{
/// Create a Fem2D with a cylinder shape
/// \note This is defining a cylinder based on cylindrical coordinates M(length, angle)
/// \note The cylinder is composed of cross-sections with nodes added radially to each cross-section.
/// \note The nodes of 2 consecutive cross-sections are connected to form square-patches which in turn
/// \note are decomposed into 2 Fem2DElementTriangle.
void createFem2DCylinder(std::shared_ptr<Fem2DRepresentation> physicsRepresentation,
	std::shared_ptr<DeformableCollisionRepresentation> collisionRepresentation,
	const SurgSim::Math::RigidTransform3d& pose,
	const bool setBoundaryConditions = true, const double youngModulus = 1e6)
{
	// Mechanical properties
	//const double youngModulus = 1e6;
	const double poissonRatio = 0.35;
	const double massDensity = 5000.0;
	// Geometrical properties
	const double length = 0.5;
	const double radius = 4e-2;
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
	if (setBoundaryConditions)
	{
		// We fix the nodes on the 1st and last cross-sections
		const size_t section0 = 0;
		const size_t section1 = numSections - 1;
		for (size_t nodeId = 0; nodeId < numNodesOnSection; nodeId++)
		{
			restState->addBoundaryCondition(nodeId + numNodesOnSection * section0);
			restState->addBoundaryCondition(nodeId + numNodesOnSection * section1);
		}
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
std::shared_ptr<SurgSim::Framework::SceneElement> loadFem2D(const std::string& name,
	const SurgSim::Math::RigidTransform3d& pose,
	SurgSim::Math::IntegrationScheme integrationScheme,
	bool setBoundaryConditions = true, double youngModulus = 1e6)
{
	std::shared_ptr<Fem2DRepresentation> physicsRepresentation
		= std::make_shared<Fem2DRepresentation>("Physics Representation" + name);
	auto collisionRepresentation = std::make_shared<DeformableCollisionRepresentation>("Collision " + name);
	collisionRepresentation->setCollisionDetectionType(SurgSim::Collision::COLLISION_DETECTION_TYPE_CONTINUOUS);

	createFem2DCylinder(physicsRepresentation, collisionRepresentation, pose, setBoundaryConditions, youngModulus);

	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);
	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(1e-1);
	physicsRepresentation->setRayleighDampingStiffness(1e-4);

	std::shared_ptr<BasicSceneElement> femSceneElement = std::make_shared<BasicSceneElement>(name);
	femSceneElement->addComponent(physicsRepresentation);
	femSceneElement->addComponent(collisionRepresentation);

	// Create a triangle mesh for visualizing the surface of the finite element model
	std::shared_ptr<OsgMeshRepresentation> graphicsTriangleMeshRepresentation
		= std::make_shared<OsgMeshRepresentation>("TriangleMesh Representation");
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

// Load a fixed representation
std::shared_ptr<SurgSim::Framework::SceneElement> loadMesh(
	const std::string& name,
	const std::string& fileName,
	const SurgSim::Math::RigidTransform3d& pose,
	bool isFixed)
{
	auto sceneElement = std::make_shared<SurgSim::Framework::BasicSceneElement>(name);
	sceneElement->setPose(pose);

	auto shape = std::make_shared<SurgSim::Math::MeshShape>();
	shape->load(fileName);

	std::shared_ptr<SurgSim::Physics::RigidRepresentationBase> phx;
	if (isFixed)
	{
		phx = std::make_shared<SurgSim::Physics::FixedRepresentation>("Physics");
	}
	else
	{
		phx = std::make_shared<SurgSim::Physics::RigidRepresentation>("Physics");
		phx->setIsGravityEnabled(true);
		// http://www.engineeringtoolbox.com/wood-density-d_40.html
		phx->setDensity(5800.0); // Cedar of Lebanon wood density 5800.0 Kg/m-3
	}

	phx->setShape(shape);
	sceneElement->addComponent(phx);

	auto collision = std::make_shared<SurgSim::Physics::RigidCollisionRepresentation>("Collision");
	collision->setCollisionDetectionType(SurgSim::Collision::COLLISION_DETECTION_TYPE_CONTINUOUS);
	phx->setCollisionRepresentation(collision);
	collision->setShape(shape);
	sceneElement->addComponent(collision);

	std::shared_ptr<OsgMeshRepresentation> osgRepresentation =
		std::make_shared<OsgMeshRepresentation>("OsgRepresentation");
	osgRepresentation->setShape(shape);
	sceneElement->addComponent(osgRepresentation);

	return sceneElement;
}

} // anonymous namespace

namespace SurgSim
{
namespace Physics
{

class CcdRenderTest : public RenderTests
{
public:
	void SetUp() override
	{
		RenderTests::SetUp();

		SurgSim::Framework::Logger::getLoggerManager()->getLogger(physicsManager->getName())
			->setThreshold(SurgSim::Framework::LOG_LEVEL_DEBUG);
		physicsManager->setRate(150.0);
		physicsManager->setComputations(SurgSim::Physics::createCcdPipeline(false));

	}
};

TEST_F(CcdRenderTest, CcdTriangleMeshTriangleMeshDeformableRigid)
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::makeRigidTranslation;

	physicsManager->setRate(300.0);

	scene->addSceneElement(
		loadMesh("RigidMesh",
		"sphere0.025.ply",
		SurgSim::Math::makeRigidTranslation(Vector3d(0.2, 0.0, 0.03)),
		true
		));
	scene->addSceneElement(
		loadMesh("RigidMesh",
		"sphere0.025.ply",
		SurgSim::Math::makeRigidTranslation(Vector3d(-0.2, 0.0, 0.03)),
		true
		));
	scene->addSceneElement(
		loadMesh("RigidMesh",
		"sphere0.025.ply",
		SurgSim::Math::makeRigidTranslation(Vector3d(0.2, 0.0, -0.03)),
		true
		));
	scene->addSceneElement(
		loadMesh("RigidMesh",
		"sphere0.025.ply",
		SurgSim::Math::makeRigidTranslation(Vector3d(-0.2, 0.0, -0.03)),
		true
		));

	scene->addSceneElement(
		loadFem2D("DeformableCylinder",
		makeRigidTranslation(Vector3d(0,0.1,0)),
		SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_IMPLICIT, false));

	runTest(Vector3d(0.0, 0.2, 1.0), Vector3d(0.0, 0.0, 0.0), 50000.0);
}

TEST_F(CcdRenderTest, CcdTriangleMeshTriangleMeshDeformables)
{
	using SurgSim::Math::makeRigidTransform;

	physicsManager->setRate(300.0);

	scene->addSceneElement(
		loadFem2D("DeformableCylinder1",
		SurgSim::Math::RigidTransform3d::Identity(),
		SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_IMPLICIT));

	scene->addSceneElement(
		loadFem2D("DeformableCylinder2",
		makeRigidTransform(Vector3d(0, 0.1, 0), Vector3d(-1, 0.1, 0), Vector3d(0, 1.1, 0)),
		SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_IMPLICIT, true, 1e5));

	runTest(Vector3d(0.2, 0.2, 1.0), Vector3d(0.0, 0.0, 0.0), 50000.0);
}

} // namespace Physics

} // namespace SurgSim
