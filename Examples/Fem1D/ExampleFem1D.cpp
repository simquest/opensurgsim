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
#include <boost/thread.hpp>

#include "SurgSim/Blocks/BasicSceneElement.h"
#include "SurgSim/Blocks/TransferDeformableStateToVerticesBehavior.h"
#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Framework/BehaviorManager.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/FemElement1DBeam.h"
#include "SurgSim/Physics/PhysicsManager.h"

using SurgSim::Blocks::BasicSceneElement;
using SurgSim::Blocks::TransferDeformableStateToVerticesBehavior;
using SurgSim::Framework::Logger;
using SurgSim::Framework::SceneElement;
using SurgSim::Graphics::OsgPointCloudRepresentation;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4f;
using SurgSim::Physics::DeformableRepresentationState;
using SurgSim::Physics::Fem1DRepresentation;
using SurgSim::Physics::FemElement1DBeam;
using SurgSim::Physics::PhysicsManager;

///\file Example of how to put together a very simple demo of Fem1D

namespace
{

void loadModelFem1D(std::shared_ptr<Fem1DRepresentation> physicsRepresentation, unsigned int numNodes)
{
	auto restState = std::make_shared<DeformableRepresentationState>();
	restState->setNumDof(physicsRepresentation->getNumDofPerNode(), numNodes);

	// Sets the initial state (node positions and boundary conditions)
	SurgSim::Math::Vector& x = restState->getPositions();
	for (unsigned int nodeId = 0; nodeId < numNodes; nodeId++)
	{
		SurgSim::Math::getSubVector(x, nodeId, physicsRepresentation->getNumDofPerNode()).segment<3>(0)
			= Vector3d(static_cast<double>(nodeId) / static_cast<double>(numNodes), 0.0, 0.0);
	}

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
		auto beam = std::make_shared<FemElement1DBeam>(beamNodeIds, *restState);
		beam->setCrossSectionCircular(0.10);
		beam->setMassDensity(3000.0);
		beam->setPoissonRatio(0.45);
		beam->setYoungModulus(1e6);
		physicsRepresentation->addFemElement(beam);
	}
}

std::shared_ptr<SurgSim::Graphics::ViewElement> createView(const std::string& name, int x, int y, int width, int height)
{
	using SurgSim::Graphics::OsgViewElement;

	auto viewElement = std::make_shared<OsgViewElement>(name);
	viewElement->getView()->setPosition(x, y);
	viewElement->getView()->setDimensions(width, height);

	return viewElement;
}

std::shared_ptr<SceneElement> createFem1D(const std::string& name,
										  const std::vector<SurgSim::Math::RigidTransform3d> gfxPoses,
										  SurgSim::Math::Vector4d color,
										  SurgSim::Math::IntegrationScheme integrationScheme)
{
	auto physicsRepresentation = std::make_shared<Fem1DRepresentation>(name + " Physics");

	// In this example, the physics representations are not transformed, only the graphics will be transformed
	loadModelFem1D(physicsRepresentation, 10);

	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(5e-2);
	physicsRepresentation->setRayleighDampingStiffness(5e-3);

	auto femSceneElement = std::make_shared<BasicSceneElement>(name);
	femSceneElement->addComponent(physicsRepresentation);

	unsigned int gfxObjectId = 0;
	for (const SurgSim::Math::RigidTransform3d& gfxPose : gfxPoses)
	{
		std::stringstream ss;
		ss << name + " Graphics object " << gfxObjectId;

		auto graphicsRepresentation = std::make_shared<OsgPointCloudRepresentation<void>>(ss.str());
		graphicsRepresentation->setInitialPose(gfxPose);
		graphicsRepresentation->setColor(color);
		graphicsRepresentation->setPointSize(3.0f);
		graphicsRepresentation->setVisible(true);

		femSceneElement->addComponent(graphicsRepresentation);

		ss.clear();
		ss << "Physics to Graphics (" << gfxObjectId << ") deformable points";

		femSceneElement->addComponent(
			std::make_shared<TransferDeformableStateToVerticesBehavior<void>>(
				ss.str(), physicsRepresentation->getFinalState(), graphicsRepresentation->getVertices())
			);

		gfxObjectId++;
	}

	return femSceneElement;
}

} // anonymous namespace


int main(int argc, char* argv[])
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::Vector4d;

	auto graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();
	auto physicsManager = std::make_shared<PhysicsManager>();
	auto behaviorManager = std::make_shared<SurgSim::Framework::BehaviorManager>();
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>();

	runtime->addManager(physicsManager);
	runtime->addManager(graphicsManager);
	runtime->addManager(behaviorManager);

	std::shared_ptr<SurgSim::Graphics::OsgCamera> camera = graphicsManager->getDefaultCamera();
	std::shared_ptr<SurgSim::Framework::Scene> scene = runtime->getScene();

	SurgSim::Math::Quaterniond qIdentity = SurgSim::Math::Quaterniond::Identity();
	SurgSim::Math::Vector3d translate(0, 0, -3);
	SurgSim::Math::IntegrationScheme integrationScheme = SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER;
	{
		std::vector<SurgSim::Math::RigidTransform3d> gfxPoses;

		gfxPoses.push_back(makeRigidTransform(qIdentity, translate + Vector3d(-3.0, 0.5, 0.0)));
		gfxPoses.push_back(makeRigidTransform(qIdentity, translate + Vector3d( 3.0, 0.5, 0.0)));
		scene->addSceneElement(createFem1D("Euler Explicit", gfxPoses, Vector4d(1, 0, 0, 1), integrationScheme));

		gfxPoses.clear();
		gfxPoses.push_back(makeRigidTransform(qIdentity, translate + Vector3d(-1.0, 0.5, 0.0)));
		gfxPoses.push_back(makeRigidTransform(qIdentity, translate + Vector3d( 3.0, 0.5, 0.0)));
		scene->addSceneElement(
			createFem1D("Modified Euler Explicit", gfxPoses, Vector4d(0, 1, 0, 1), integrationScheme));

		gfxPoses.clear();
		gfxPoses.push_back(makeRigidTransform(qIdentity, translate + Vector3d( 1.0, 0.5, 0.0)));
		gfxPoses.push_back(makeRigidTransform(qIdentity, translate + Vector3d( 3.0, 0.5, 0.0)));
		scene->addSceneElement(createFem1D("Euler Implicit", gfxPoses, Vector4d(0, 0, 1, 1), integrationScheme));
	}

	scene->addSceneElement(createView("view1", 0, 0, 1023, 768));

	camera->setInitialPose(SurgSim::Math::makeRigidTransform(qIdentity, Vector3d(0.0, 0.5, 5.0)));

	runtime->execute();

	return 0;
}
