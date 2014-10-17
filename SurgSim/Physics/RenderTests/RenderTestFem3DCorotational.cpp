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

///\file RenderTestFem3DCorotational.cpp render test for Fem3D with corotational elements

#include <memory>

#include "SurgSim/Blocks/TransferPhysicsToPointCloudBehavior.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/Fem3DElementCorotationalTetrahedron.h"
#include "SurgSim/Physics/RenderTests/RenderTest.h"

using SurgSim::Blocks::TransferPhysicsToPointCloudBehavior;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Graphics::OsgPointCloudRepresentation;
using SurgSim::Physics::Fem3DRepresentation;
using SurgSim::Physics::FemElement;
using SurgSim::Physics::Fem3DElementCorotationalTetrahedron;
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
	physicsRepresentation->setRayleighDampingMass(1e-2);
	physicsRepresentation->setRayleighDampingStiffness(1e-3);

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
	std::array< std::array<size_t, 4>, 5> tetrahedrons = {{
			{{4, 7, 1, 2}}, // CCW (47)cross(41) . (42) > 0
			{{4, 1, 7, 5}}, // CCW (41)cross(47) . (45) > 0
			{{4, 2, 1, 0}}, // CCW (42)cross(41) . (40) > 0
			{{4, 7, 2, 6}}, // CCW (47)cross(42) . (46) > 0
			{{1, 2, 7, 3}}  // CCW (12)cross(17) . (13) > 0
		}
	};

	std::array<size_t, 2> boundaryConditionsNodeIdx = {{0, 1}};

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
		std::shared_ptr<FemElement> element = std::make_shared<Fem3DElementCorotationalTetrahedron>(*tetrahedron);
		element->setMassDensity(8000.0);
		element->setPoissonRatio(0.45);
		element->setYoungModulus(1.0e6);
		physicsRepresentation->addFemElement(element);
	}

	// Graphics Representation
	std::shared_ptr<OsgPointCloudRepresentation> graphicsRepresentation;
	graphicsRepresentation = std::make_shared<OsgPointCloudRepresentation>(name + " Graphics object ");
	graphicsRepresentation->setLocalPose(pose);
	graphicsRepresentation->setColor(color);
	graphicsRepresentation->setPointSize(3.0f);
	graphicsRepresentation->setLocalActive(true);

	// Scene Element
	std::shared_ptr<BasicSceneElement> femSceneElement = std::make_shared<BasicSceneElement>(name);
	femSceneElement->addComponent(physicsRepresentation);
	femSceneElement->addComponent(graphicsRepresentation);

	auto physicsToGraphics =
		std::make_shared<TransferPhysicsToPointCloudBehavior>("Physics to Graphics deformable points");
	physicsToGraphics->setSource(physicsRepresentation);
	physicsToGraphics->setTarget(graphicsRepresentation);
	femSceneElement->addComponent(physicsToGraphics);

	return femSceneElement;
}

}; // anonymous namespace

namespace SurgSim
{

namespace Physics
{

TEST_F(RenderTests, VisualTestFem3DCorotatioal)
{
	using SurgSim::Math::makeRigidTranslation;

	// Cube with corotational tetrahedron FemElement
	scene->addSceneElement(createTetrahedronFem3D("CorotationalTetrahedronElement Euler Explicit",
						   makeRigidTranslation(Vector3d(-4.0, 1.0, -1.0)),
						   SurgSim::Math::Vector4d(1, 0, 0, 1),
						   SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER));

	scene->addSceneElement(createTetrahedronFem3D("CorotatinoalTetrahedronElement Modified Euler Explicit",
						   makeRigidTranslation(Vector3d(-2.0, 1.0, -1.0)),
						   SurgSim::Math::Vector4d(0.5, 0, 0, 1),
						   SurgSim::Math::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER));

	scene->addSceneElement(createTetrahedronFem3D("CorotatinoalTetrahedronElement Runge Kutta 4",
						   makeRigidTranslation(Vector3d(0.0, 1.0, -1.0)),
						   SurgSim::Math::Vector4d(0, 1, 0, 1),
						   SurgSim::Math::INTEGRATIONSCHEME_RUNGE_KUTTA_4));

	scene->addSceneElement(createTetrahedronFem3D("CorotatinoalTetrahedronElement Fem 3D Euler Implicit",
						   makeRigidTranslation(Vector3d(2.0, 1.0, -1.0)),
						   SurgSim::Math::Vector4d(0, 0, 1, 1),
						   SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER));

	scene->addSceneElement(createTetrahedronFem3D("CorotatinoalTetrahedronElement Fem 3D Static",
						   makeRigidTranslation(Vector3d(4.0, 1.0, -1.0)),
						   SurgSim::Math::Vector4d(1, 1, 1, 1),
						   SurgSim::Math::INTEGRATIONSCHEME_STATIC));

	runTest(Vector3d(0.0, 0.0, 7.0), Vector3d::Zero(), 5000.0);
}

}; // namespace Physics

}; // namespace SurgSim
