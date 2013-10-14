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

#include <SurgSim/Blocks/MassSpring1DRepresentation.h>
#include <SurgSim/Blocks/MassSpring2DRepresentation.h>
#include <SurgSim/Blocks/MassSpring3DRepresentation.h>
using SurgSim::Blocks::MassSpring1DRepresentation;
using SurgSim::Blocks::MassSpring2DRepresentation;
using SurgSim::Blocks::MassSpring3DRepresentation;

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

std::shared_ptr<SceneElement> createMassSpring1D(const std::string& name, const SurgSim::Math::RigidTransform3d& pose,
	SurgSim::Math::Vector4d color, MassSpringRepresentation::IntegrationScheme integrationScheme)
{
	std::shared_ptr<MassSpring1DRepresentation> physicsRepresentation =
		std::make_shared<MassSpring1DRepresentation>(name + " Physics");

	// Note: setInitialPose MUST be called before setInitialState to be effective !
	// When using MassSpringnDRepresentation, setInitialPose must be called before initnD !
	physicsRepresentation->setInitialPose(pose);

	std::vector<unsigned int> boundaryConditions;
	boundaryConditions.push_back(0);
	boundaryConditions.push_back(1);
	boundaryConditions.push_back(2);
	SurgSim::Math::Vector3d extremities[2] = { Vector3d(0,0,0), Vector3d(1,0,0) };
	unsigned int numNodesPerDim[1] = {6};
	physicsRepresentation->init1D(extremities,
		numNodesPerDim,
		boundaryConditions,
		0.1, // total mass (in Kg)
		100.0, // Stiffness stretching
		0.0, // Damping stretching
		10.0, // Stiffness bending
		0.0); // Damping bending

	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(1e-1);
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

std::shared_ptr<SceneElement> createMassSpring2D(const std::string& name, const SurgSim::Math::RigidTransform3d& pose,
	SurgSim::Math::Vector4d color, MassSpringRepresentation::IntegrationScheme integrationScheme)
{
	std::shared_ptr<MassSpring2DRepresentation> physicsRepresentation =
		std::make_shared<MassSpring2DRepresentation>(name + " Physics");

	// Note: setInitialPose MUST be called before setInitialState to be effective !
	// When using MassSpringnDRepresentation, setInitialPose must be called before initnD !
	physicsRepresentation->setInitialPose(pose);

	std::vector<unsigned int> boundaryConditions;
	boundaryConditions.push_back(0);
	boundaryConditions.push_back(1);
	boundaryConditions.push_back(2);
	SurgSim::Math::Vector3d extremities[2][2] = {
		{ Vector3d(0,0,0), Vector3d(1,0,0) },
		{ Vector3d(0,-1,0), Vector3d(1,-1,0) }
	};
	unsigned int numNodesPerDim[2] = {3, 3};
	physicsRepresentation->init2D(extremities,
		numNodesPerDim,
		boundaryConditions,
		0.1, // total mass (in Kg)
		100.0, // Stiffness stretching
		0.0, // Damping stretching
		10.0, // Stiffness bending
		0.0, // Damping bending
		10.0, // Stiffness face diagonal
		0.0); // Damping face diagonal

	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(1e-1);
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

std::shared_ptr<SceneElement> createMassSpring3D(const std::string& name, const SurgSim::Math::RigidTransform3d& pose,
	SurgSim::Math::Vector4d color, MassSpringRepresentation::IntegrationScheme integrationScheme)
{
	std::shared_ptr<MassSpring3DRepresentation> physicsRepresentation =
		std::make_shared<MassSpring3DRepresentation>(name + " Physics");

	// Note: setInitialPose MUST be called before setInitialState to be effective !
	// When using MassSpringnDRepresentation, setInitialPose must be called before initnD !
	physicsRepresentation->setInitialPose(pose);

	std::vector<unsigned int> boundaryConditions;
	boundaryConditions.push_back(0);
	boundaryConditions.push_back(1);
	boundaryConditions.push_back(2);
	SurgSim::Math::Vector3d extremities[2][2][2] = {
		{
			{ Vector3d(0,0,0), Vector3d(1,0,0) }
			,
			{ Vector3d(0,-1,0), Vector3d(1,-1,0) }
		}
		,
		{
			{ Vector3d(0,0,-1), Vector3d(1,0,-1) }
			,
			{ Vector3d(0,-1,-1), Vector3d(1,-1,-1) }
		},
	};
	unsigned int numNodesPerDim[3] = {3, 3, 3};
	physicsRepresentation->init3D(extremities,
		numNodesPerDim,
		boundaryConditions,
		0.1, // total mass (in Kg)
		100.0, // Stiffness stretching
		0.0, // Damping stretching
		10.0, // Stiffness bending
		0.0,  // Damping bending
		10.0, // Stiffness face diagonal
		0.0, // Damping face diagonal
		10.0, // Stiffness volume diagonal
		0.0); // Damping volume diagonal

	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(1e-1);
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
	SurgSim::Math::Vector3d translate(0,0,-3);

	// MassSpring1D
	{
		// All separate on the screen
		scene->addSceneElement(createMassSpring1D("MassSpring 1D Euler Explicit",
			makeRigidTransform(qIdentity, translate+Vector3d(-3.0, 3.0, 0.0)), Vector4d(1, 0, 0, 1),
			MassSpringRepresentation::INTEGRATIONSCHEME_EXPLICIT_EULER));
		scene->addSceneElement(createMassSpring1D("MassSpring 1D Modified Euler Explicit",
			makeRigidTransform(qIdentity, translate+Vector3d(-1.0, 3.0, 0.0)), Vector4d(0, 1, 0, 1),
			MassSpringRepresentation::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER));
		scene->addSceneElement(createMassSpring1D("MassSpring 1D Euler Implicit",
			makeRigidTransform(qIdentity, translate+Vector3d(1.0, 3.0, 0.0)), Vector4d(0, 0, 1, 1),
			MassSpringRepresentation::INTEGRATIONSCHEME_IMPLICIT_EULER));

		// All superposed on the screen
		scene->addSceneElement(createMassSpring1D("MassSpring 1D Euler Explicit",
			makeRigidTransform(qIdentity, translate+Vector3d(3.0, 3.0, 0.0)), Vector4d(1, 0, 0, 1),
			MassSpringRepresentation::INTEGRATIONSCHEME_EXPLICIT_EULER));
		scene->addSceneElement(createMassSpring1D("MassSpring 1D Modified Euler Explicit",
			makeRigidTransform(qIdentity, translate+Vector3d(3.0, 3.0, 0.0)), Vector4d(0, 1, 0, 1),
			MassSpringRepresentation::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER));
		scene->addSceneElement(createMassSpring1D("MassSpring 1D Euler Implicit",
			makeRigidTransform(qIdentity, translate+Vector3d(3.0, 3.0, 0.0)), Vector4d(0, 0, 1, 1),
			MassSpringRepresentation::INTEGRATIONSCHEME_IMPLICIT_EULER));
	}

	// MassSpring2D
	{
		// All separate on the screen
		scene->addSceneElement(createMassSpring2D("MassSpring 2D Euler Explicit",
			makeRigidTransform(qIdentity, translate+Vector3d(-3.0, 1.5, 0.0)), Vector4d(1, 0, 0, 1),
			MassSpringRepresentation::INTEGRATIONSCHEME_EXPLICIT_EULER));
		scene->addSceneElement(createMassSpring2D("MassSpring 2D Modified Euler Explicit",
			makeRigidTransform(qIdentity, translate+Vector3d(-1.0, 1.5, 0.0)), Vector4d(0, 1, 0, 1),
			MassSpringRepresentation::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER));
		scene->addSceneElement(createMassSpring2D("MassSpring 2D Euler Implicit",
			makeRigidTransform(qIdentity, translate+Vector3d(1.0, 1.5, 0.0)), Vector4d(0, 0, 1, 1),
			MassSpringRepresentation::INTEGRATIONSCHEME_IMPLICIT_EULER));

		// All superposed on the screen
		scene->addSceneElement(createMassSpring2D("MassSpring 2D Euler Explicit",
			makeRigidTransform(qIdentity, translate+Vector3d(3.0, 1.5, 0.0)), Vector4d(1, 0, 0, 1),
			MassSpringRepresentation::INTEGRATIONSCHEME_EXPLICIT_EULER));
		scene->addSceneElement(createMassSpring2D("MassSpring 2D Modified Euler Explicit",
			makeRigidTransform(qIdentity, translate+Vector3d(3.0, 1.5, 0.0)), Vector4d(0, 1, 0, 1),
			MassSpringRepresentation::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER));
		scene->addSceneElement(createMassSpring2D("MassSpring 2D Euler Implicit",
			makeRigidTransform(qIdentity, translate+Vector3d(3.0, 1.5, 0.0)), Vector4d(0, 0, 1, 1),
			MassSpringRepresentation::INTEGRATIONSCHEME_IMPLICIT_EULER));
	}

	// MassSpring3D
	{
		// All separate on the screen
		scene->addSceneElement(createMassSpring3D("MassSpring 3D Euler Explicit",
			makeRigidTransform(qIdentity, translate+Vector3d(-3.0, -0.5, 0.0)), Vector4d(1, 0, 0, 1),
			MassSpringRepresentation::INTEGRATIONSCHEME_EXPLICIT_EULER));
		scene->addSceneElement(createMassSpring3D("MassSpring 3D Modified Euler Explicit",
			makeRigidTransform(qIdentity, translate+Vector3d(-1.0, -0.5, 0.0)), Vector4d(0, 1, 0, 1),
			MassSpringRepresentation::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER));
		scene->addSceneElement(createMassSpring3D("MassSpring 3D Euler Implicit",
			makeRigidTransform(qIdentity, translate+Vector3d(1.0, -0.5, 0.0)), Vector4d(0, 0, 1, 1),
			MassSpringRepresentation::INTEGRATIONSCHEME_IMPLICIT_EULER));

		// All superposed on the screen
		scene->addSceneElement(createMassSpring3D("MassSpring 3D Euler Explicit",
			makeRigidTransform(qIdentity, translate+Vector3d(3.0, -0.5, 0.0)), Vector4d(1, 0, 0, 1),
			MassSpringRepresentation::INTEGRATIONSCHEME_EXPLICIT_EULER));
		scene->addSceneElement(createMassSpring3D("MassSpring 3D Modified Euler Explicit",
			makeRigidTransform(qIdentity, translate+Vector3d(3.0, -0.5, 0.0)), Vector4d(0, 1, 0, 1),
			MassSpringRepresentation::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER));
		scene->addSceneElement(createMassSpring3D("MassSpring 3D Euler Implicit",
			makeRigidTransform(qIdentity, translate+Vector3d(3.0, -0.5, 0.0)), Vector4d(0, 0, 1, 1),
			MassSpringRepresentation::INTEGRATIONSCHEME_IMPLICIT_EULER));
	}

	scene->addSceneElement(createView("view1", 0, 0, 1023, 768));

	graphicsManager->getDefaultCamera()->setInitialPose(
		SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0, 0.5, 5.0)));

	runtime->setScene(scene);

	runtime->execute();

	return 0;
}
