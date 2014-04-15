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

#include <memory>

#include "SurgSim/Blocks/TransferDeformableStateToVerticesBehavior.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/PointCloudRepresentation.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem2DRepresentation.h"
#include "SurgSim/Physics/FemElement2DTriangle.h"
#include "SurgSim/Physics/PhysicsManager.h"

using SurgSim::Blocks::TransferDeformableStateToVerticesBehavior;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Framework::SceneElement;
using SurgSim::Graphics::OsgPointCloudRepresentation;
using SurgSim::Math::Vector3d;
using SurgSim::Physics::DeformableRepresentationState;
using SurgSim::Physics::Fem2DRepresentation;
using SurgSim::Physics::FemElement2DTriangle;
using SurgSim::Physics::PhysicsManager;

///\file Example of how to put together a very simple demo of Fem2D

namespace
{

/// Load a cylinder shape as a Fem2D
void loadModelFem2D(std::shared_ptr<Fem2DRepresentation> physicsRepresentation)
{
	const double youngModulus = 1e6;
	const double poissonRatio = 0.35;
	const double massDensity = 5000.0;
	const double length = 1.0;
	const double radius = 1e-1;
	const double thickness = 3e-2;
	const size_t numSections = 7;
	const size_t numNodesOnSection = 8;
	const size_t numNodes = numSections * numNodesOnSection;
	const double deltaL = length / (numSections - 1);
	const double deltaAngle = 2.0 * M_PI / numNodesOnSection;

	std::shared_ptr<DeformableRepresentationState> restState = std::make_shared<DeformableRepresentationState>();
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
	for (size_t nodeId = 0; nodeId < numNodesOnSection; nodeId++)
	{
		for (size_t dofId = 0; dofId < numDofPerNode; dofId++)
		{
			restState->addBoundaryCondition(nodeId * numDofPerNode + dofId);
			restState->addBoundaryCondition((numNodes - 1 - nodeId) * numDofPerNode + dofId);
		}
	}
	physicsRepresentation->setInitialState(restState);

	// Adds all the FemElements
	for (size_t sectionId = 0; sectionId < numSections - 1; sectionId++)
	{
		for (size_t nodeIdOnSection = 0; nodeIdOnSection < numNodesOnSection - 1; nodeIdOnSection++)
		{
			std::array<std::array<unsigned int, 2>, 2> nodeIds =
			{{
				{{
					sectionId * numNodesOnSection + nodeIdOnSection,
					sectionId * numNodesOnSection + nodeIdOnSection + 1
				}}
				,
				{{
					(sectionId + 1) * numNodesOnSection + nodeIdOnSection,
					(sectionId + 1) * numNodesOnSection + nodeIdOnSection + 1
				}}
			}};
			std::array<unsigned int, 3> triangle1NodeIds = {{nodeIds[0][0], nodeIds[0][1], nodeIds[1][1]}};
			std::shared_ptr<FemElement2DTriangle> triangle1 = std::make_shared<FemElement2DTriangle>(triangle1NodeIds);
			triangle1->setThickness(thickness);
			triangle1->setMassDensity(massDensity);
			triangle1->setPoissonRatio(poissonRatio);
			triangle1->setYoungModulus(youngModulus);
			physicsRepresentation->addFemElement(triangle1);

			std::array<unsigned int, 3> triangle2NodeIds = {{nodeIds[0][0], nodeIds[1][1], nodeIds[1][0]}};
			std::shared_ptr<FemElement2DTriangle> triangle2 = std::make_shared<FemElement2DTriangle>(triangle2NodeIds);
			triangle2->setThickness(thickness);
			triangle2->setMassDensity(massDensity);
			triangle2->setPoissonRatio(poissonRatio);
			triangle2->setYoungModulus(youngModulus);
			physicsRepresentation->addFemElement(triangle2);
		}
		size_t nodeIdOnSection = numNodesOnSection - 1;
		std::array<std::array<unsigned int, 2>, 2> nodeIds = {{
			{{sectionId * numNodesOnSection + nodeIdOnSection, sectionId * numNodesOnSection + 0}}
			,
			{{(sectionId + 1) * numNodesOnSection + nodeIdOnSection, (sectionId + 1) * numNodesOnSection + 0}}
		}};
		std::array<unsigned int, 3> triangle1NodeIds = {{nodeIds[0][0], nodeIds[0][1], nodeIds[1][1]}};
		std::shared_ptr<FemElement2DTriangle> triangle1 = std::make_shared<FemElement2DTriangle>(triangle1NodeIds);
		triangle1->setThickness(thickness);
		triangle1->setMassDensity(massDensity);
		triangle1->setPoissonRatio(poissonRatio);
		triangle1->setYoungModulus(youngModulus);
		physicsRepresentation->addFemElement(triangle1);

		std::array<unsigned int, 3> triangle2NodeIds = {{nodeIds[0][0], nodeIds[1][1], nodeIds[1][0]}};
		std::shared_ptr<FemElement2DTriangle> triangle2 = std::make_shared<FemElement2DTriangle>(triangle2NodeIds);
		triangle2->setThickness(thickness);
		triangle2->setMassDensity(massDensity);
		triangle2->setPoissonRatio(poissonRatio);
		triangle2->setYoungModulus(youngModulus);
		physicsRepresentation->addFemElement(triangle2);
	}
}

std::shared_ptr<SurgSim::Graphics::ViewElement> createView(
	const std::string& name, int x, int y, int width, int height)
{
	using SurgSim::Graphics::OsgViewElement;

	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>(name);
	viewElement->getView()->setPosition(x, y);
	viewElement->getView()->setDimensions(width, height);

	return viewElement;
}

// Generates a 1d fem comprised of adjacent elements along a straight line.  The number of fem elements is determined
// by loadModelFem1D.
std::shared_ptr<SceneElement> createFem2D(const std::string& name,
		const SurgSim::Math::RigidTransform3d& gfxPose,
		const SurgSim::Math::Vector4d& color,
		SurgSim::Math::IntegrationScheme integrationScheme)
{
	std::shared_ptr<Fem2DRepresentation> physicsRepresentation
		= std::make_shared<Fem2DRepresentation>("Physics Representation: " + name);

	// In this example, the physics representations are not transformed, only the graphics will be transformed
	loadModelFem2D(physicsRepresentation);

	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(1e-2);
	physicsRepresentation->setRayleighDampingStiffness(1e-3);

	std::shared_ptr<BasicSceneElement> femSceneElement = std::make_shared<BasicSceneElement>(name);
	femSceneElement->addComponent(physicsRepresentation);

	// Create a triangle mesh for visualizing the surface of the finite element model
	std::shared_ptr<SurgSim::Graphics::OsgMeshRepresentation> graphicsTriangleMeshRepresentation
		= std::make_shared<SurgSim::Graphics::OsgMeshRepresentation>(name + " triangle mesh");
	graphicsTriangleMeshRepresentation->setInitialPose(gfxPose);
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
		std::array<unsigned int, 3> nodeIds = {{nodeIdsVector[0], nodeIdsVector[1], nodeIdsVector[2]}};
		SurgSim::Graphics::Mesh::TriangleType t(nodeIds);
		mesh->addTriangle(t);
	}
	femSceneElement->addComponent(graphicsTriangleMeshRepresentation);

	// Create a behavior which transfers the position of the vertices in the FEM to locations in the triangle mesh
	femSceneElement->addComponent(
		std::make_shared<SurgSim::Blocks::TransferDeformableStateToVerticesBehavior<SurgSim::Graphics::VertexData>>(
			name + " physics to triangle mesh",
			physicsRepresentation->getFinalState(),
			graphicsTriangleMeshRepresentation->getMesh()));

	std::shared_ptr<SurgSim::Graphics::PointCloudRepresentation<void>> graphicsPointCloudRepresentation
			= std::make_shared<OsgPointCloudRepresentation<void>>("Graphics Representation: " + name);
	graphicsPointCloudRepresentation->setInitialPose(gfxPose);
	graphicsPointCloudRepresentation->setColor(color);
	graphicsPointCloudRepresentation->setPointSize(3.0f);
	graphicsPointCloudRepresentation->setVisible(true);
	femSceneElement->addComponent(graphicsPointCloudRepresentation);

	femSceneElement->addComponent(std::make_shared<TransferDeformableStateToVerticesBehavior<void>>(
		"Transfer from Physics to Graphics point cloud: " + name,
		physicsRepresentation->getFinalState(),
		graphicsPointCloudRepresentation->getVertices()));

	return femSceneElement;
}

} // anonymous namespace


int main(int argc, char* argv[])
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::Vector4d;

	std::shared_ptr<SurgSim::Graphics::OsgManager> graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();
	std::shared_ptr<PhysicsManager> physicsManager = std::make_shared<PhysicsManager>();
	std::shared_ptr<SurgSim::Framework::BehaviorManager> behaviorManager
		= std::make_shared<SurgSim::Framework::BehaviorManager>();
	std::shared_ptr<SurgSim::Framework::Runtime> runtime = std::make_shared<SurgSim::Framework::Runtime>();

	runtime->addManager(physicsManager);
	runtime->addManager(graphicsManager);
	runtime->addManager(behaviorManager);

	std::shared_ptr<SurgSim::Graphics::OsgCamera> camera = graphicsManager->getDefaultCamera();
	std::shared_ptr<SurgSim::Framework::Scene> scene = runtime->getScene();

	const SurgSim::Math::Quaterniond quaternionIdentity = SurgSim::Math::Quaterniond::Identity();
	const SurgSim::Math::Quaterniond quaternion(Eigen::AngleAxisd(-M_PI / 6.0, Vector3d(0.0, 1.0, 0.0)));

	scene->addSceneElement(
		createFem2D("Euler Explicit",                                          // name
					makeRigidTransform(quaternion, Vector3d(0.0, 0.4, 0.0)),   // graphics pose (rot., trans.)
					Vector4d(1, 0, 0, 1),                                      // color (r, g, b, a)
					SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EXPLICIT_EULER));  // technique to update object

	scene->addSceneElement(
		createFem2D("Modified Euler Explicit",
					makeRigidTransform(quaternion, Vector3d(0.0, 0.0, 0.0)),
					Vector4d(0, 1, 0, 1),
					SurgSim::Math::INTEGRATIONSCHEME_LINEAR_MODIFIED_EXPLICIT_EULER));

	scene->addSceneElement(
		createFem2D("Euler Implicit",
					makeRigidTransform(quaternion, Vector3d(0.0, -0.4, 0.0)),
					Vector4d(0, 0, 1, 1),
					SurgSim::Math::INTEGRATIONSCHEME_LINEAR_IMPLICIT_EULER));

	scene->addSceneElement(createView("view1", 0, 0, 1023, 768));

	camera->setInitialPose(SurgSim::Math::makeRigidTransform(quaternionIdentity, Vector3d(0.0, 0.0, 2.0)));

	runtime->execute();

	return 0;
}
