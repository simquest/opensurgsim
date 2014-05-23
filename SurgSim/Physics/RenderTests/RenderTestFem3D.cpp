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

///\file RenderTestFem3D.cpp render test for Fem3D

#include <memory>

#include "SurgSim/Blocks/TransferOdeStateToVerticesBehavior.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/Fem3DElementCube.h"
#include "SurgSim/Physics/Fem3DElementTetrahedron.h"
#include "SurgSim/Physics/RenderTests/RenderTest.h"

using SurgSim::Blocks::TransferOdeStateToVerticesBehavior;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Graphics::OsgPointCloudRepresentation;
using SurgSim::Physics::Fem3DRepresentation;
using SurgSim::Physics::FemElement;
using SurgSim::Physics::Fem3DElementCube;
using SurgSim::Physics::Fem3DElementTetrahedron;
using SurgSim::Math::Vector3d;

namespace
{

std::shared_ptr<SurgSim::Framework::SceneElement> createTetrahedronFem3D(const std::string& name,
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

	std::shared_ptr<SurgSim::Math::OdeState> initialState = std::make_shared<SurgSim::Math::OdeState>();
	initialState->setNumDof(physicsRepresentation->getNumDofPerNode(), 8);

	for (size_t i = 0; i != vertices.size(); i++)
	{
		initialState->getPositions().segment(i * 3, 3) = vertices[i];
	}

	for (auto index = boundaryConditionsNodeIdx.cbegin(); index != boundaryConditionsNodeIdx.cend(); ++index)
	{
		initialState->addBoundaryCondition(*index);
	}
	physicsRepresentation->setInitialState(initialState);

	for (auto tetrahedron = tetrahedrons.cbegin(); tetrahedron != tetrahedrons.cend(); ++tetrahedron)
	{
		std::shared_ptr<FemElement> element = std::make_shared<Fem3DElementTetrahedron>(*tetrahedron);
		element->setMassDensity(8000.0);
		element->setPoissonRatio(0.45);
		element->setYoungModulus(1.0e6);
		physicsRepresentation->addFemElement(element);
	}

	// Graphics Representation
	std::shared_ptr<OsgPointCloudRepresentation<void>> graphicsRepresentation;
	graphicsRepresentation = std::make_shared<OsgPointCloudRepresentation<void>>(name + " Graphics object ");
	graphicsRepresentation->setLocalPose(pose);
	graphicsRepresentation->setColor(color);
	graphicsRepresentation->setPointSize(3.0f);
	graphicsRepresentation->setVisible(true);

	// Scene Element
	std::shared_ptr<BasicSceneElement> femSceneElement = std::make_shared<BasicSceneElement>(name);
	femSceneElement->addComponent(physicsRepresentation);
	femSceneElement->addComponent(graphicsRepresentation);
	femSceneElement->addComponent(std::make_shared<TransferOdeStateToVerticesBehavior<void>>(
		"Physics to Graphics deformable points",
		physicsRepresentation->getFinalState(),
		graphicsRepresentation->getVertices()));

	return femSceneElement;
}

std::shared_ptr<SurgSim::Framework::SceneElement> createCubeFem3D(const std::string& name,
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

	std::shared_ptr<SurgSim::Math::OdeState> initialState = std::make_shared<SurgSim::Math::OdeState>();
	initialState->setNumDof(physicsRepresentation->getNumDofPerNode(), 8);

	for (size_t i = 0; i != vertices.size(); i++)
	{
		initialState->getPositions().segment(i * 3, 3) = vertices[i];
	}

	for (auto index = boundaryConditionsNodeIdx.cbegin(); index != boundaryConditionsNodeIdx.cend(); ++index)
	{
		initialState->addBoundaryCondition(*index);
	}
	physicsRepresentation->setInitialState(initialState);

	std::shared_ptr<FemElement> element = std::make_shared<Fem3DElementCube>(cube);
	element->setMassDensity(8000.0);
	element->setPoissonRatio(0.45);
	element->setYoungModulus(1.0e6);
	physicsRepresentation->addFemElement(element);

	// Graphics Representation
	std::shared_ptr<OsgPointCloudRepresentation<void>> graphicsRepresentation;
	graphicsRepresentation = std::make_shared<OsgPointCloudRepresentation<void>>(name + " Graphics object ");
	graphicsRepresentation->setLocalPose(pose);
	graphicsRepresentation->setColor(color);
	graphicsRepresentation->setPointSize(3.0f);
	graphicsRepresentation->setVisible(true);

	// Scene Element
	std::shared_ptr<BasicSceneElement> femSceneElement = std::make_shared<BasicSceneElement>(name);
	femSceneElement->addComponent(physicsRepresentation);
	femSceneElement->addComponent(graphicsRepresentation);
	femSceneElement->addComponent(std::make_shared<TransferOdeStateToVerticesBehavior<void>>(
		"Physics to Graphics deformable points",
		physicsRepresentation->getFinalState(),
		graphicsRepresentation->getVertices()));

	return femSceneElement;
}

}; // anonymous namespace

namespace SurgSim
{

namespace Physics
{

TEST_F(RenderTests, VisualTestFem3D)
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::Vector4d;

	SurgSim::Math::Quaterniond qIdentity = SurgSim::Math::Quaterniond::Identity();

	// Cube with cube FemElement
	scene->addSceneElement(createCubeFem3D("CubeElement Euler Explicit",
										   makeRigidTransform(qIdentity, Vector3d(-2.5, 2.0, 0.0)),
										   Vector4d(1, 0, 0, 1),
										   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EXPLICIT_EULER));

	scene->addSceneElement(createCubeFem3D("CubeElement Modified Euler Explicit",
										   makeRigidTransform(qIdentity, Vector3d(0.0, 2.0, 0.0)),
										   Vector4d(0, 1, 0, 1),
										   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_MODIFIED_EXPLICIT_EULER));

	scene->addSceneElement(createCubeFem3D("CubeElement Fem 3D Euler Implicit",
										   makeRigidTransform(qIdentity, Vector3d(2.5, 2.0, 0.0)),
										   Vector4d(0, 0, 1, 1),
										   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_IMPLICIT_EULER));

	// Cube with tetrahedron FemElement
	scene->addSceneElement(createTetrahedronFem3D("TetrahedronElement Euler Explicit",
						   makeRigidTransform(qIdentity, Vector3d(-2.5, -1.0, 0.0)),
						   Vector4d(1, 0, 0, 1),
						   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EXPLICIT_EULER));

	scene->addSceneElement(createTetrahedronFem3D("TetrahedronElement Modified Euler Explicit",
						   makeRigidTransform(qIdentity, Vector3d(0.0, -1.0, 0.0)),
						   Vector4d(0, 1, 0, 1),
						   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_MODIFIED_EXPLICIT_EULER));

	scene->addSceneElement(createTetrahedronFem3D("TetrahedronElement Fem 3D Euler Implicit",
						   makeRigidTransform(qIdentity, Vector3d(2.5, -1.0, 0.0)),
						   Vector4d(0, 0, 1, 1),
						   SurgSim::Math::INTEGRATIONSCHEME_LINEAR_IMPLICIT_EULER));

	runTest(Vector3d(0.0, 0.0, 7.0), Vector3d::Zero(), 5000.0);
}

}; // namespace Physics

}; // namespace SurgSim
