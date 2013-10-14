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

#include <SurgSim/Blocks/BasicSceneElement.h>
#include <SurgSim/Framework/Behavior.h>
#include <SurgSim/Framework/BehaviorManager.h>
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Framework/SceneElement.h>
#include <SurgSim/Graphics/OsgCamera.h>
#include <SurgSim/Graphics/OsgManager.h>
#include <SurgSim/Graphics/OsgView.h>
#include <SurgSim/Graphics/OsgViewElement.h>
#include <SurgSim/Physics/PhysicsManager.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>
using SurgSim::Blocks::BasicSceneElement;
using SurgSim::Framework::Logger;
using SurgSim::Framework::SceneElement;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4f;
using SurgSim::Physics::PhysicsManager;

#include <SurgSim/Physics/MassSpringRepresentation.h>
#include <SurgSim/Physics/Mass.h>
#include <SurgSim/Physics/LinearSpring.h>
using SurgSim::Physics::MassSpringRepresentation;
using SurgSim::Physics::DeformableRepresentationState;

#include <SurgSim/Graphics/OsgPointCloudRepresentation.h>
using SurgSim::Graphics::OsgPointCloudRepresentation;

#include <SurgSim/Blocks/TransferDeformableStateToVerticesBehavior.h>
using SurgSim::Blocks::TransferDeformableStateToVerticesBehavior;

///\file Example of how to put together a very simple demo of mass springs

std::shared_ptr<SurgSim::Graphics::ViewElement> createView(const std::string& name, int x, int y, int width, int height)
{
	using SurgSim::Graphics::OsgViewElement;

	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>(name);
	viewElement->getView()->setPosition(x, y);
	viewElement->getView()->setDimensions(width, height);

	return viewElement;
}

std::shared_ptr<SceneElement> createMassSpring(const std::string& name, const SurgSim::Math::RigidTransform3d& pose,
	SurgSim::Math::Vector4d color, MassSpringRepresentation::IntegrationScheme integrationScheme)
{
	using SurgSim::Math::setSubVector;
	using SurgSim::Physics::Mass;
	using SurgSim::Physics::LinearSpring;

	const unsigned int numNodes = 20;

	std::shared_ptr<MassSpringRepresentation> physicsRepresentation =
		std::make_shared<MassSpringRepresentation>(name + " Physics");

	// Note: setInitialPose MUST be called before setInitialState to be effective !
	physicsRepresentation->setInitialPose(pose);

	std::shared_ptr<DeformableRepresentationState> state;
	state = std::make_shared<DeformableRepresentationState>();
	state->setNumDof(3 * numNodes);
	for (unsigned int i = 0; i < numNodes; i++)
	{
		Vector3d p(static_cast<double>(i)/static_cast<double>(numNodes), 0, 0);
		setSubVector(p, i, 3, &state->getPositions());
		physicsRepresentation->addMass(std::make_shared<Mass>(0.01));
	}
	state->addBoundaryCondition(0); // 1st node dof X
	state->addBoundaryCondition(1); // 1st node dof Y
	state->addBoundaryCondition(2); // 1st node dof Z
	for (unsigned int i = 0; i < numNodes - 1; i++)
	{
		std::shared_ptr<LinearSpring> spring = std::make_shared<LinearSpring>(i, i+1);
		spring->setDamping(0.0);
		spring->setStiffness(100.0);
		spring->setInitialLength(1.0/static_cast<double>(numNodes));
		physicsRepresentation->addSpring(spring);
	}
	physicsRepresentation->setInitialState(state);
	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(1e0);
	physicsRepresentation->setRayleighDampingStiffness(1e-2);

	std::shared_ptr<OsgPointCloudRepresentation<void>> graphicsRepresentation =
		std::make_shared<OsgPointCloudRepresentation<void>>(name + " Graphics");
	std::shared_ptr<SurgSim::DataStructures::Vertices<void>> vertices;
	vertices = std::make_shared<SurgSim::DataStructures::Vertices<void>>();
	graphicsRepresentation->setVertices(vertices);
	graphicsRepresentation->setColor(color);
	graphicsRepresentation->setPointSize(3.0f);
	graphicsRepresentation->setVisible(true);

	std::shared_ptr<SceneElement> massSpringElement = std::make_shared<BasicSceneElement>(name);
	massSpringElement->addComponent(physicsRepresentation);
	massSpringElement->addComponent(graphicsRepresentation);
	massSpringElement->addComponent(std::make_shared<TransferDeformableStateToVerticesBehavior<void>>
		("Physics to Graphics deformable points",
		physicsRepresentation->getFinalState(),
		physicsRepresentation->getNumDof() / physicsRepresentation->getNumMasses(),
		graphicsRepresentation->getVertices()));

	return massSpringElement;
}

int main(int argc, char* argv[])
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::Vector4d;

	std::shared_ptr<SurgSim::Graphics::OsgManager> graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();
	std::shared_ptr<PhysicsManager> physicsManager = std::make_shared<PhysicsManager>();
	std::shared_ptr<SurgSim::Framework::BehaviorManager> behaviorManager =
		std::make_shared<SurgSim::Framework::BehaviorManager>();
	std::shared_ptr<SurgSim::Framework::Runtime> runtime(new SurgSim::Framework::Runtime());

	runtime->addManager(physicsManager);
	runtime->addManager(graphicsManager);
	runtime->addManager(behaviorManager);

	std::shared_ptr<SurgSim::Framework::Scene> scene(new SurgSim::Framework::Scene());

	SurgSim::Math::Quaterniond qIdentity = SurgSim::Math::Quaterniond::Identity();

	scene->addSceneElement(createMassSpring("MassSpring 1D Euler Explicit",
		makeRigidTransform(qIdentity, Vector3d(-1.0, 0.0, 0.0)), Vector4d(1, 0, 0, 1),
		MassSpringRepresentation::INTEGRATIONSCHEME_EXPLICIT_EULER));

	scene->addSceneElement(createMassSpring("MassSpring 1D Modified Euler Explicit",
		makeRigidTransform(qIdentity, Vector3d(0.0, 0.0, 0.0)), Vector4d(0, 1, 0, 1),
		MassSpringRepresentation::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER));

	scene->addSceneElement(createMassSpring("MassSpring 1D Euler Implicit",
		makeRigidTransform(qIdentity, Vector3d(1.0, 0.0, 0.0)), Vector4d(0, 0, 1, 1),
		MassSpringRepresentation::INTEGRATIONSCHEME_IMPLICIT_EULER));

	scene->addSceneElement(createMassSpring("MassSpring 1D Euler Explicit",
		makeRigidTransform(qIdentity, Vector3d(0.0, 2.0, 0.0)), Vector4d(1, 0, 0, 1),
		MassSpringRepresentation::INTEGRATIONSCHEME_EXPLICIT_EULER));
	scene->addSceneElement(createMassSpring("MassSpring 1D Modified Euler Explicit",
		makeRigidTransform(qIdentity, Vector3d(0.0, 2.0, 0.0)), Vector4d(0, 1, 0, 1),
		MassSpringRepresentation::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER));
	scene->addSceneElement(createMassSpring("MassSpring 1D Euler Implicit",
		makeRigidTransform(qIdentity, Vector3d(0.0, 2.0, 0.0)), Vector4d(0, 0, 1, 1),
		MassSpringRepresentation::INTEGRATIONSCHEME_IMPLICIT_EULER));

	scene->addSceneElement(createView("view1", 0, 0, 1023, 768));

	graphicsManager->getDefaultCamera()->setInitialPose(
		SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0, 0.5, 5.0)));

	runtime->setScene(scene);

	runtime->execute();

	return 0;
}
