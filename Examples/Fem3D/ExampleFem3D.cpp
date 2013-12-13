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

#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/FemElement3DCube.h"
#include "SurgSim/Physics/FemElement3DTetrahedron.h"
#include "SurgSim/Physics/PhysicsManager.h"

#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Blocks::BasicSceneElement;
using SurgSim::Blocks::TransferDeformableStateToVerticesBehavior;
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
/// using tetrahedron's elements and cube's elements

namespace
{
// Cube nodes
//       2*-----------*3
//       /           /|
//    6*-----------*7 |      ^ y
//     |           |  |      |
//     |  0        |  *1     *->x
//     |           | /      /
//    4*-----------*5       z
std::array<SurgSim::Math::Vector3d, 8> cubeNodes =
{{
	Vector3d(-0.5,-0.5,-0.5), Vector3d( 0.5,-0.5,-0.5),
	Vector3d(-0.5, 0.5,-0.5), Vector3d( 0.5, 0.5,-0.5),
	Vector3d(-0.5,-0.5, 0.5), Vector3d( 0.5,-0.5, 0.5),
	Vector3d(-0.5, 0.5, 0.5), Vector3d( 0.5, 0.5, 0.5)
}};

// Cube decomposition into 5 tetrahedrons
// https://www.math.ucdavis.edu/~deloera/CURRENT_INTERESTS/cube.html
const unsigned int numTetrahedrons = 5;
std::array< std::array<unsigned int, 4>, numTetrahedrons> tetrahedrons =
{{
	{{4, 7, 1, 2}}, // CCW (47)cross(41) . (42) > 0
	{{4, 1, 7, 5}}, // CCW (41)cross(47) . (45) > 0
	{{4, 2, 1, 0}}, // CCW (42)cross(41) . (40) > 0
	{{4, 7, 2, 6}}, // CCW (47)cross(42) . (46) > 0
	{{1, 2, 7, 3}}  // CCW (12)cross(17) . (13) > 0
}};

const unsigned int numCubes = 1;
std::array< std::array<unsigned int, 8>, numCubes> cubes =
{{
	// 1st face CW, 2nd face CCW (Faces being seen from outside)
	{{0, 1, 3, 2, 4, 5, 7, 6}}
}};

// Boundary conditions (node indices)
const unsigned int numBoundaryConditionsNodeIdx = 4;
const std::array<unsigned int, numBoundaryConditionsNodeIdx> boundaryConditionsNodeIdx =
{{
	0, 1, 2, 3
}};

void loadFem3DRestState(std::shared_ptr<Fem3DRepresentation> physicsRepresentation)
{
	std::shared_ptr<DeformableRepresentationState> restState = std::make_shared<DeformableRepresentationState>();
	restState->setNumDof(physicsRepresentation->getNumDofPerNode(), 8);
	SurgSim::Math::Vector& x = restState->getPositions();

	// Sets the initial state (node positions and boundary conditions)
	for (int nodeId = 0; nodeId < 8; nodeId++)
	{
		SurgSim::Math::getSubVector(x, nodeId, 3) =  cubeNodes[nodeId];
	}
	for (unsigned int boundaryConditionId = 0;
		boundaryConditionId < numBoundaryConditionsNodeIdx;
		boundaryConditionId++)
	{
		// The boundary conditions in the state are the dof indices to be fixed
		restState->addBoundaryCondition(boundaryConditionsNodeIdx[boundaryConditionId] * 3 + 0);
		restState->addBoundaryCondition(boundaryConditionsNodeIdx[boundaryConditionId] * 3 + 1);
		restState->addBoundaryCondition(boundaryConditionsNodeIdx[boundaryConditionId] * 3 + 2);
	}
	physicsRepresentation->setInitialState(restState);
}

void setFemElementParameters(std::shared_ptr<Fem3DRepresentation> physicsRepresentation)
{
	// Sets all the FemElement's parameters
	for (unsigned int elementId = 0; elementId < physicsRepresentation->getNumFemElements(); elementId++)
	{
		std::shared_ptr<FemElement> element = physicsRepresentation->getFemElement(elementId);
		element->setMassDensity(8000.0);
		element->setPoissonRatio(0.45);
		element->setYoungModulus(1.0e6);
	}
}


void loadCubeModelFem3D(std::shared_ptr<Fem3DRepresentation> physicsRepresentation)
{
	loadFem3DRestState(physicsRepresentation);
	auto restState = physicsRepresentation->getInitialState();

	// Adds all the cube FemElements
	for (unsigned int elementId = 0; elementId < numCubes; elementId++)
	{
		std::shared_ptr<FemElement3DCube> element = nullptr;
		element = std::make_shared<FemElement3DCube>(cubes[elementId], *restState);
		physicsRepresentation->addFemElement(element);
	}
}

void loadTetrahedronModelFem3D(std::shared_ptr<Fem3DRepresentation> physicsRepresentation)
{
	loadFem3DRestState(physicsRepresentation);
	auto restState = physicsRepresentation->getInitialState();

	// Adds all the tetrahedrons FemElements
	for (unsigned int elementId = 0; elementId < numTetrahedrons; elementId++)
	{
		std::shared_ptr<FemElement3DTetrahedron> element = nullptr;
		element = std::make_shared<FemElement3DTetrahedron>(tetrahedrons[elementId], *restState);
		physicsRepresentation->addFemElement(element);
	}
}
};

std::shared_ptr<SceneElement> initializeFem3D(const std::string& name,
	std::shared_ptr<Fem3DRepresentation> physicsRepresentation,
	const std::vector<SurgSim::Math::RigidTransform3d> gfxPoses,
	SurgSim::Math::Vector4d color, SurgSim::Math::IntegrationScheme integrationScheme)
{
	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(5e-2);
	physicsRepresentation->setRayleighDampingStiffness(5e-3);

	std::shared_ptr<SceneElement> femSceneElement = std::make_shared<BasicSceneElement>(name);
	femSceneElement->addComponent(physicsRepresentation);

	unsigned int gfxObjectId = 0;
	for (auto gfxPose = std::begin(gfxPoses); gfxPose != std::end(gfxPoses); gfxPose++)
	{
		std::stringstream ss;
		ss << name + " Graphics object " << gfxObjectId;
		std::shared_ptr<OsgPointCloudRepresentation<void>> graphicsRepresentation =
			std::make_shared<OsgPointCloudRepresentation<void>>(ss.str());

		graphicsRepresentation->setInitialPose(*gfxPose);
		graphicsRepresentation->setColor(color);
		graphicsRepresentation->setPointSize(3.0f);
		graphicsRepresentation->setVisible(true);

		femSceneElement->addComponent(graphicsRepresentation);
		ss.clear();
		ss << "Physics to Graphics ("<< gfxObjectId <<") deformable points";
		femSceneElement->addComponent(std::make_shared<TransferDeformableStateToVerticesBehavior<void>>
			(ss.str(),
			physicsRepresentation->getFinalState(),
			graphicsRepresentation->getVertices()));

		gfxObjectId++;
	}

	return femSceneElement;
}

std::shared_ptr<SceneElement> createCubeFem3D(const std::string& name,
	const std::vector<SurgSim::Math::RigidTransform3d> gfxPoses,
	SurgSim::Math::Vector4d color, SurgSim::Math::IntegrationScheme integrationScheme)
{
	std::shared_ptr<Fem3DRepresentation> physicsRepresentation =
		std::make_shared<Fem3DRepresentation>(name + " Physics");

	// In this example, the physics representations are not transformed,
	// only the graphics one will apply a transform
	loadCubeModelFem3D(physicsRepresentation);

	setFemElementParameters(physicsRepresentation);

	return initializeFem3D(name, physicsRepresentation, gfxPoses, color, integrationScheme);
}

std::shared_ptr<SceneElement> createTetrahedronFem3D(const std::string& name,
	const std::vector<SurgSim::Math::RigidTransform3d> gfxPoses,
	SurgSim::Math::Vector4d color, SurgSim::Math::IntegrationScheme integrationScheme)
{
	std::shared_ptr<Fem3DRepresentation> physicsRepresentation =
		std::make_shared<Fem3DRepresentation>(name + " Physics");

	// In this example, the physics representations are not transformed,
	// only the graphics one will apply a transform
	loadTetrahedronModelFem3D(physicsRepresentation);

	setFemElementParameters(physicsRepresentation);

	return initializeFem3D(name, physicsRepresentation, gfxPoses, color, integrationScheme);
}

std::shared_ptr<SurgSim::Graphics::ViewElement> createView(const std::string& name, int x, int y, int width, int height)
{
	using SurgSim::Graphics::OsgViewElement;

	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>(name);
	viewElement->getView()->setPosition(x, y);
	viewElement->getView()->setDimensions(width, height);

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

	std::shared_ptr<SurgSim::Framework::Scene> scene(new SurgSim::Framework::Scene());

	SurgSim::Math::Quaterniond qIdentity = SurgSim::Math::Quaterniond::Identity();
	SurgSim::Math::Vector3d translate(0,0,-3);

	std::cout << "Scene description:" << std::endl;
	std::cout << "Columns:" << std::endl;
	std::cout << "  The first 3 columns are testing different ODE solvers" << std::endl;
	std::cout << "    Explicit Euler | Modified Explicit Euler | Implicit Euler" << std::endl;
	std::cout << "  The last column superposes all ODE solver results for comparison." << std::endl;
	std::cout << "Rows:" << std::endl;
	std::cout << "  The top row is simulating a cube with a single cube element." << std::endl;
	std::cout << "  The bottom row is simulating a cube with 5 tetrahedron elements." << std::endl;

	// Cube with cube FemElement
	{
		std::vector<SurgSim::Math::RigidTransform3d> gfxPoses;

		gfxPoses.push_back(makeRigidTransform(qIdentity, translate+Vector3d(-3.0, 1.0, 0.0)));
		gfxPoses.push_back(makeRigidTransform(qIdentity, translate+Vector3d( 3.0, 1.0, 0.0)));
		scene->addSceneElement(createCubeFem3D("CubeElement Euler Explicit",
			gfxPoses, Vector4d(1, 0, 0, 1),
			SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER));

		gfxPoses.clear();
		gfxPoses.push_back(makeRigidTransform(qIdentity, translate+Vector3d(-1.0, 1.0, 0.0)));
		gfxPoses.push_back(makeRigidTransform(qIdentity, translate+Vector3d( 3.0, 1.0, 0.0)));
		scene->addSceneElement(createCubeFem3D("CubeElement Modified Euler Explicit",
			gfxPoses, Vector4d(0, 1, 0, 1),
			SurgSim::Math::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER));

		gfxPoses.clear();
		gfxPoses.push_back(makeRigidTransform(qIdentity, translate+Vector3d( 1.0, 1.0, 0.0)));
		gfxPoses.push_back(makeRigidTransform(qIdentity, translate+Vector3d( 3.0, 1.0, 0.0)));
		scene->addSceneElement(createCubeFem3D("CubeElement Fem 3D Euler Implicit",
			gfxPoses, Vector4d(0, 0, 1, 1),
			SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER));
	}

	// Cube with tetrahedron FemElement
	{
		std::vector<SurgSim::Math::RigidTransform3d> gfxPoses;

		gfxPoses.push_back(makeRigidTransform(qIdentity, translate+Vector3d(-3.0, -1.0, 0.0)));
		gfxPoses.push_back(makeRigidTransform(qIdentity, translate+Vector3d( 3.0, -1.0, 0.0)));
		scene->addSceneElement(createTetrahedronFem3D("TetrahedronElement Euler Explicit",
			gfxPoses, Vector4d(1, 0, 0, 1),
			SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER));

		gfxPoses.clear();
		gfxPoses.push_back(makeRigidTransform(qIdentity, translate+Vector3d(-1.0, -1.0, 0.0)));
		gfxPoses.push_back(makeRigidTransform(qIdentity, translate+Vector3d( 3.0, -1.0, 0.0)));
		scene->addSceneElement(createTetrahedronFem3D("TetrahedronElement Modified Euler Explicit",
			gfxPoses, Vector4d(0, 1, 0, 1),
			SurgSim::Math::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER));

		gfxPoses.clear();
		gfxPoses.push_back(makeRigidTransform(qIdentity, translate+Vector3d( 1.0, -1.0, 0.0)));
		gfxPoses.push_back(makeRigidTransform(qIdentity, translate+Vector3d( 3.0, -1.0, 0.0)));
		scene->addSceneElement(createTetrahedronFem3D("TetrahedronElement Fem 3D Euler Implicit",
			gfxPoses, Vector4d(0, 0, 1, 1),
			SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER));
	}

	scene->addSceneElement(createView("view1", 0, 0, 1023, 768));

	graphicsManager->getDefaultCamera()->setInitialPose(
		SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0, 0.5, 5.0)));

	runtime->setScene(scene);

	runtime->execute();

	return 0;
}
