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

#include "SurgSim/Blocks/BasicSceneElement.h"
#include "SurgSim/Blocks/TransferDeformableStateToVerticesBehavior.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/PointCloudRepresentation.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/FemElement1DBeam.h"
#include "SurgSim/Physics/PhysicsManager.h"

using SurgSim::Blocks::BasicSceneElement;
using SurgSim::Blocks::TransferDeformableStateToVerticesBehavior;
using SurgSim::Framework::SceneElement;
using SurgSim::Graphics::OsgPointCloudRepresentation;
using SurgSim::Math::Vector3d;
using SurgSim::Physics::DeformableRepresentationState;
using SurgSim::Physics::Fem1DRepresentation;
using SurgSim::Physics::FemElement1DBeam;
using SurgSim::Physics::PhysicsManager;

///\file Example of how to put together a very simple demo of Fem1D

namespace
{

void loadModelFem1D(std::shared_ptr<Fem1DRepresentation> physicsRepresentation, unsigned int numNodes)
{
	std::shared_ptr<DeformableRepresentationState> restState = std::make_shared<DeformableRepresentationState>();
	restState->setNumDof(physicsRepresentation->getNumDofPerNode(), numNodes);

	// Sets the initial state (node positions and boundary conditions)
	SurgSim::Math::Vector& x = restState->getPositions();
	for (unsigned int nodeId = 0; nodeId < numNodes; nodeId++)
	{
		SurgSim::Math::getSubVector(x, nodeId, physicsRepresentation->getNumDofPerNode()).segment<3>(0)
			= Vector3d(static_cast<double>(nodeId) / static_cast<double>(numNodes), 0.0, 0.0);
	}

	// Fix the start and end nodes
	restState->addBoundaryCondition(0 + 0);
	restState->addBoundaryCondition(0 + 1);
	restState->addBoundaryCondition(0 + 2);
	restState->addBoundaryCondition((numNodes - 1) * physicsRepresentation->getNumDofPerNode() + 0);
	restState->addBoundaryCondition((numNodes - 1) * physicsRepresentation->getNumDofPerNode() + 1);
	restState->addBoundaryCondition((numNodes - 1) * physicsRepresentation->getNumDofPerNode() + 2);

	physicsRepresentation->setInitialState(restState);

	// Adds all the FemElements
	for (unsigned int beamId = 0; beamId < numNodes - 1; beamId++)
	{
		std::array<unsigned int, 2> beamNodeIds = {{beamId, beamId + 1}};
		std::shared_ptr<FemElement1DBeam> beam = std::make_shared<FemElement1DBeam>(beamNodeIds);
		beam->setRadius(0.10);
		beam->setMassDensity(3000.0);
		beam->setPoissonRatio(0.45);
		beam->setYoungModulus(1e6);
		physicsRepresentation->addFemElement(beam);
	}
}

std::shared_ptr<SurgSim::Graphics::ViewElement> createView(const std::string& name, int x, int y, int width, int height)
{
	using SurgSim::Graphics::OsgViewElement;

	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>(name);
	viewElement->getView()->setPosition(x, y);
	viewElement->getView()->setDimensions(width, height);

	return viewElement;
}

// Generates a 1d fem comprised of adjacent elements along a straight line.  The number of fem elements is determined
// by loadModelFem1D.
std::shared_ptr<SceneElement> createFem1D(const std::string& name,
										  const SurgSim::Math::RigidTransform3d& gfxPose,
										  const SurgSim::Math::Vector4d& color,
										  SurgSim::Math::IntegrationScheme integrationScheme)
{
	std::shared_ptr<Fem1DRepresentation> physicsRepresentation
		= std::make_shared<Fem1DRepresentation>("Physics Representation: " + name);

	// In this example, the physics representations are not transformed, only the graphics will be transformed
	loadModelFem1D(physicsRepresentation, 10);

	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(5e-2);
	physicsRepresentation->setRayleighDampingStiffness(5e-3);

	std::shared_ptr<BasicSceneElement> femSceneElement = std::make_shared<BasicSceneElement>(name);
	femSceneElement->addComponent(physicsRepresentation);

	std::shared_ptr<SurgSim::Graphics::PointCloudRepresentation<void>> graphicsRepresentation
		= std::make_shared<OsgPointCloudRepresentation<void>>("Graphics Representation: " + name);
	graphicsRepresentation->setInitialPose(gfxPose);
	graphicsRepresentation->setColor(color);
	graphicsRepresentation->setPointSize(3.0f);
	graphicsRepresentation->setVisible(true);

	femSceneElement->addComponent(graphicsRepresentation);

	femSceneElement->addComponent(
		std::make_shared<TransferDeformableStateToVerticesBehavior<void>>("Transfer from Physics to Graphics: " + name,
																		  physicsRepresentation->getFinalState(),
																		  graphicsRepresentation->getVertices()));

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

	scene->addSceneElement(
		createFem1D("Euler Explicit",                                                  // name
					makeRigidTransform(quaternionIdentity, Vector3d(-3.0, 0.5, -3.0)), // graphics pose (rot., trans.)
					Vector4d(1, 0, 0, 1),                                              // color (r, g, b, a)
					SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER));                 // technique to update object

	scene->addSceneElement(
		createFem1D("Modified Euler Explicit",
					makeRigidTransform(quaternionIdentity, Vector3d(-0.5, 0.5, -3.0)),
					Vector4d(0, 1, 0, 1),
					SurgSim::Math::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER));

	scene->addSceneElement(
		createFem1D("Euler Implicit",
					makeRigidTransform(quaternionIdentity, Vector3d(2.0, 0.5, -3.0)),
					Vector4d(0, 0, 1, 1),
					SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER));

	scene->addSceneElement(createView("view1", 0, 0, 1023, 768));

	camera->setInitialPose(SurgSim::Math::makeRigidTransform(quaternionIdentity, Vector3d(0.0, 0.5, 5.0)));

	runtime->execute();

	return 0;
}
