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

#include "SurgSim/Blocks/TransferDeformableStateToVerticesBehavior.h"

#include "SurgSim/Framework/BasicSceneElement.h"
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

#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/FemElement3DCube.h"
#include "SurgSim/Physics/FemElement3DTetrahedron.h"
#include "SurgSim/Physics/PhysicsManager.h"

#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Blocks::TransferDeformableStateToVerticesBehavior;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Framework::Logger;
using SurgSim::Framework::SceneElement;
using SurgSim::Graphics::OsgPointCloudRepresentation;
using SurgSim::Physics::DeformableRepresentationState;
using SurgSim::Physics::Fem3DRepresentation;
using SurgSim::Physics::FemElement;
using SurgSim::Physics::FemElement3DCube;
using SurgSim::Physics::FemElement3DTetrahedron;
using SurgSim::Physics::PhysicsManager;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4f;

///\file Example of how to put together a very simple demo of Fem3D,
/// using tetrahedron's elements or cube's elements

std::shared_ptr<SceneElement> createTetrahedronFem3D(const std::string& name,
		const SurgSim::Math::RigidTransform3d& pose, SurgSim::Math::Vector4d color,
		SurgSim::Math::IntegrationScheme integrationScheme)
{
	// Physics Representation
	std::shared_ptr<Fem3DRepresentation> physicsRepresentation;
	physicsRepresentation = std::make_shared<Fem3DRepresentation>(name + " Physics");
	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(5e-2);
	physicsRepresentation->setRayleighDampingStiffness(5e-3);

	std::array<Vector3d, 8> vertices = {{
			Vector3d(-0.5, -0.5, -0.5),
			Vector3d(0.5, -0.5, -0.5),
			Vector3d(-0.5,  0.5, -0.5),
			Vector3d(0.5,  0.5, -0.5),
			Vector3d(-0.5, -0.5,  0.5),
			Vector3d(0.5, -0.5,  0.5),
			Vector3d(-0.5,  0.5,  0.5),
			Vector3d(0.5,  0.5,  0.5)
		}
	};

	// Cube decomposition into 5 tetrahedrons
	// https://www.math.ucdavis.edu/~deloera/CURRENT_INTERESTS/cube.html
	std::array< std::array<unsigned int, 4>, 5> tetrahedrons = {{
			{{4, 7, 1, 2}}, // CCW (47)cross(41) . (42) > 0
			{{4, 1, 7, 5}}, // CCW (41)cross(47) . (45) > 0
			{{4, 2, 1, 0}}, // CCW (42)cross(41) . (40) > 0
			{{4, 7, 2, 6}}, // CCW (47)cross(42) . (46) > 0
			{{1, 2, 7, 3}}  // CCW (12)cross(17) . (13) > 0
		}
	};

	std::array<unsigned int, 4> boundaryConditionsNodeIdx = {{0, 1, 2, 3}};

	std::shared_ptr<DeformableRepresentationState> initialState = std::make_shared<DeformableRepresentationState>();
	initialState->setNumDof(physicsRepresentation->getNumDofPerNode(), 8);

	for (size_t i = 0; i != vertices.size(); i++)
	{
		initialState->getPositions().segment(i * 3, 3) = vertices[i];
	}

	for (auto index = boundaryConditionsNodeIdx.cbegin(); index != boundaryConditionsNodeIdx.cend(); ++index)
	{
		initialState->addBoundaryCondition((*index) * 3 + 0);
		initialState->addBoundaryCondition((*index) * 3 + 1);
		initialState->addBoundaryCondition((*index) * 3 + 2);
	}
	physicsRepresentation->setInitialState(initialState);

	for (auto tetrahedron = tetrahedrons.cbegin(); tetrahedron != tetrahedrons.cend(); ++tetrahedron)
	{
		std::shared_ptr<FemElement> element = std::make_shared<FemElement3DTetrahedron>(*tetrahedron);
		element->setMassDensity(8000.0);
		element->setPoissonRatio(0.45);
		element->setYoungModulus(1.0e6);
		physicsRepresentation->addFemElement(element);
	}

	// Graphics Representation
	std::shared_ptr<OsgPointCloudRepresentation<void>> graphicsRepresentation;
	graphicsRepresentation = std::make_shared<OsgPointCloudRepresentation<void>>(name + " Graphics object ");
	graphicsRepresentation->setInitialPose(pose);
	graphicsRepresentation->setColor(color);
	graphicsRepresentation->setPointSize(3.0f);
	graphicsRepresentation->setVisible(true);

	// Scene Element
	std::shared_ptr<SceneElement> femSceneElement = std::make_shared<BasicSceneElement>(name);
	femSceneElement->addComponent(physicsRepresentation);
	femSceneElement->addComponent(graphicsRepresentation);
	femSceneElement->addComponent(std::make_shared<TransferDeformableStateToVerticesBehavior<void>>(
									  "Physics to Graphics deformable points",
									  physicsRepresentation->getFinalState(),
									  graphicsRepresentation->getVertices())
								 );

	return femSceneElement;
}

std::shared_ptr<SceneElement> createCubeFem3D(const std::string& name,
		const SurgSim::Math::RigidTransform3d& pose,
		SurgSim::Math::Vector4d color, SurgSim::Math::IntegrationScheme integrationScheme)
{
	// Physics Representation
	std::shared_ptr<Fem3DRepresentation> physicsRepresentation;
	physicsRepresentation = std::make_shared<Fem3DRepresentation>(name + " Physics");
	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(5e-2);
	physicsRepresentation->setRayleighDampingStiffness(5e-3);

	std::array<Vector3d, 8> vertices = {{
			Vector3d(-0.5, -0.5, -0.5),
			Vector3d(0.5, -0.5, -0.5),
			Vector3d(-0.5,  0.5, -0.5),
			Vector3d(0.5,  0.5, -0.5),
			Vector3d(-0.5, -0.5,  0.5),
			Vector3d(0.5, -0.5,  0.5),
			Vector3d(-0.5,  0.5,  0.5),
			Vector3d(0.5,  0.5,  0.5)
		}
	};
	std::array<unsigned int, 8> cube = {{0, 1, 3, 2, 4, 5, 7, 6}};
	std::array<unsigned int, 4> boundaryConditionsNodeIdx = {{0, 1, 2, 3}};

	std::shared_ptr<DeformableRepresentationState> initialState = std::make_shared<DeformableRepresentationState>();
	initialState->setNumDof(physicsRepresentation->getNumDofPerNode(), 8);

	for (size_t i = 0; i != vertices.size(); i++)
	{
		initialState->getPositions().segment(i * 3, 3) = vertices[i];
	}

	for (auto index = boundaryConditionsNodeIdx.cbegin(); index != boundaryConditionsNodeIdx.cend(); ++index)
	{
		initialState->addBoundaryCondition((*index) * 3 + 0);
		initialState->addBoundaryCondition((*index) * 3 + 1);
		initialState->addBoundaryCondition((*index) * 3 + 2);
	}
	physicsRepresentation->setInitialState(initialState);

	std::shared_ptr<FemElement> element = std::make_shared<FemElement3DCube>(cube, *initialState);
	element->setMassDensity(8000.0);
	element->setPoissonRatio(0.45);
	element->setYoungModulus(1.0e6);
	physicsRepresentation->addFemElement(element);

	// Graphics Representation
	std::shared_ptr<OsgPointCloudRepresentation<void>> graphicsRepresentation;
	graphicsRepresentation = std::make_shared<OsgPointCloudRepresentation<void>>(name + " Graphics object ");
	graphicsRepresentation->setInitialPose(pose);
	graphicsRepresentation->setColor(color);
	graphicsRepresentation->setPointSize(3.0f);
	graphicsRepresentation->setVisible(true);

	// Scene Element
	std::shared_ptr<SceneElement> femSceneElement = std::make_shared<BasicSceneElement>(name);
	femSceneElement->addComponent(physicsRepresentation);
	femSceneElement->addComponent(graphicsRepresentation);
	femSceneElement->addComponent(std::make_shared<TransferDeformableStateToVerticesBehavior<void>>(
									  "Physics to Graphics deformable points",
									  physicsRepresentation->getFinalState(),
									  graphicsRepresentation->getVertices())
								 );

	return femSceneElement;
}

std::shared_ptr<SurgSim::Graphics::ViewElement> createView(const std::string& name, int x, int y, int width, int height)
{
	using SurgSim::Graphics::OsgViewElement;

	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>(name);
	viewElement->getView()->setPosition(x, y);
	viewElement->getView()->setDimensions(width, height);

	viewElement->enableManipulator(true);
	viewElement->setManipulatorParameters(Vector3d(0.0, 0.0, 7.0), Vector3d::Zero());

	return viewElement;
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

	auto scene = runtime->getScene();

	SurgSim::Math::Quaterniond qIdentity = SurgSim::Math::Quaterniond::Identity();

	std::cout << "Scene description:" << std::endl;
	std::cout << "Columns:" << std::endl;
	std::cout << "  The 3 columns are testing different ODE solvers" << std::endl;
	std::cout << "    Explicit Euler | Modified Explicit Euler | Implicit Euler" << std::endl;
	std::cout << "Rows:" << std::endl;
	std::cout << "  The top row is simulating a cube with a single cube element." << std::endl;
	std::cout << "  The bottom row is simulating a cube with 5 tetrahedron elements." << std::endl;

	// Cube with cube FemElement
	scene->addSceneElement(createCubeFem3D("CubeElement Euler Explicit",
										   makeRigidTransform(qIdentity, Vector3d(-2.0, 1.5, 0.0)),
										   Vector4d(1, 0, 0, 1),
										   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EXPLICIT_EULER));

	scene->addSceneElement(createCubeFem3D("CubeElement Modified Euler Explicit",
										   makeRigidTransform(qIdentity, Vector3d(0.0, 1.5, 0.0)),
										   Vector4d(0, 1, 0, 1),
										   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_MODIFIED_EXPLICIT_EULER));

	scene->addSceneElement(createCubeFem3D("CubeElement Fem 3D Euler Implicit",
										   makeRigidTransform(qIdentity, Vector3d(2.0, 1.5, 0.0)),
										   Vector4d(0, 0, 1, 1),
										   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_IMPLICIT_EULER));

	// Cube with tetrahedron FemElement
	scene->addSceneElement(createTetrahedronFem3D("TetrahedronElement Euler Explicit",
						   makeRigidTransform(qIdentity, Vector3d(-2.0, -0.5, 0.0)),
						   Vector4d(1, 0, 0, 1),
						   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EXPLICIT_EULER));

	scene->addSceneElement(createTetrahedronFem3D("TetrahedronElement Modified Euler Explicit",
						   makeRigidTransform(qIdentity, Vector3d(0.0, -0.5, 0.0)),
						   Vector4d(0, 1, 0, 1),
						   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_MODIFIED_EXPLICIT_EULER));

	scene->addSceneElement(createTetrahedronFem3D("TetrahedronElement Fem 3D Euler Implicit",
						   makeRigidTransform(qIdentity, Vector3d(2.0, -0.5, 0.0)),
						   Vector4d(0, 0, 1, 1),
						   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_IMPLICIT_EULER));

	scene->addSceneElement(createView("view1", 0, 0, 1023, 768));

	graphicsManager->getDefaultCamera()->setInitialPose(
		SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0, 0.0, 7.0)));

	runtime->execute();

	return 0;
}
