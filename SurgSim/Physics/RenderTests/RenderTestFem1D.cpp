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

///\file RenderTestFem1D.cpp render test for Fem1D

#include <memory>

#include "SurgSim/Blocks/TransferPhysicsToVerticesBehavior.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Graphics/OsgCurveRepresentation.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/Fem1DElementBeam.h"
#include "SurgSim/Physics/RenderTests/RenderTest.h"
#include "SurgSim/Blocks/TransferPhysicsToVerticesBehavior.h"

using SurgSim::Blocks::TransferPhysicsToVerticesBehavior;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Graphics::OsgCurveRepresentation;
using SurgSim::Math::Vector3d;
using SurgSim::Physics::Fem1DRepresentation;
using SurgSim::Physics::Fem1DElementBeam;

namespace
{

void loadModelFem1D(std::shared_ptr<Fem1DRepresentation> physicsRepresentation, size_t numNodes)
{
	std::shared_ptr<SurgSim::Math::OdeState> restState = std::make_shared<SurgSim::Math::OdeState>();
	restState->setNumDof(physicsRepresentation->getNumDofPerNode(), numNodes);

	// Sets the initial state (node positions and boundary conditions)
	SurgSim::Math::Vector& x = restState->getPositions();
	for (size_t nodeId = 0; nodeId < numNodes; nodeId++)
	{
		SurgSim::Math::getSubVector(x, nodeId, physicsRepresentation->getNumDofPerNode()).segment<3>(0)
			= Vector3d(static_cast<double>(nodeId) / static_cast<double>(numNodes - 1) - 0.5, 0.0, 0.0);
	}

	// Fix the start and end nodes
	restState->addBoundaryCondition(0, 0);
	restState->addBoundaryCondition(0, 1);
	restState->addBoundaryCondition(0, 2);
	restState->addBoundaryCondition(numNodes - 1, 0);
	restState->addBoundaryCondition(numNodes - 1, 1);
	restState->addBoundaryCondition(numNodes - 1, 2);

	physicsRepresentation->setInitialState(restState);

	// Adds all the FemElements
	for (size_t beamId = 0; beamId < numNodes - 1; beamId++)
	{
		std::array<size_t, 2> beamNodeIds = {{beamId, beamId + 1}};
		std::shared_ptr<Fem1DElementBeam> beam = std::make_shared<Fem1DElementBeam>(beamNodeIds);
		beam->setRadius(0.10);
		beam->setMassDensity(3000.0);
		beam->setPoissonRatio(0.45);
		beam->setYoungModulus(1e6);
		physicsRepresentation->addFemElement(beam);
	}
}

// Generates a 1d fem comprised of adjacent elements along a straight line.  The number of fem elements is determined
// by loadModelFem1D.
std::shared_ptr<SurgSim::Framework::SceneElement> createFem1D(const std::string& name,
		const SurgSim::Math::RigidTransform3d& gfxPose,
		const SurgSim::Math::Vector4d& color,
		SurgSim::Math::IntegrationScheme integrationScheme)
{
	auto physicsRepresentation = std::make_shared<Fem1DRepresentation>("Physics Representation: " + name);

	// In this test, the physics representations are not transformed, only the graphics will be transformed
	loadModelFem1D(physicsRepresentation, 10);

	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(5e-2);
	physicsRepresentation->setRayleighDampingStiffness(5e-3);

	auto femSceneElement = std::make_shared<BasicSceneElement>(name);
	femSceneElement->addComponent(physicsRepresentation);

	auto graphicsRepresentation = std::make_shared<OsgCurveRepresentation>("Graphics Representation: " + name);
	graphicsRepresentation->setLocalPose(gfxPose);
	graphicsRepresentation->setColor(color);

	femSceneElement->addComponent(graphicsRepresentation);

	auto copier = std::make_shared<SurgSim::Blocks::TransferPhysicsToVerticesBehavior>("Copier");
	copier->setSource(physicsRepresentation);
	copier->setTarget(graphicsRepresentation);
	femSceneElement->addComponent(copier);

	return femSceneElement;
}

}; // anonymous namespace

namespace SurgSim
{

namespace Physics
{

TEST_F(RenderTests, VisualTestFem1D)
{
	using SurgSim::Math::makeRigidTranslation;
	using SurgSim::Math::Vector4d;

	scene->addSceneElement(
		createFem1D("Euler Explicit",                                           // name
					makeRigidTranslation(Vector3d(0.0, 0.5, 0.0)),              // graphics pose
					Vector4d(1, 0, 0, 1),                                       // color (r, g, b, a)
					SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_EXPLICIT));   // technique to update object

	scene->addSceneElement(
		createFem1D("Modified Euler Explicit",
					makeRigidTranslation(Vector3d(0.0, 0.25, 0.0)),
					Vector4d(0.5, 0, 0, 1),
					SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_EXPLICIT));

	scene->addSceneElement(
		createFem1D("Runge Kutta 4",
					makeRigidTranslation(Vector3d(0.0, 0.0, 0.0)),
					Vector4d(0, 1, 0, 1),
					SurgSim::Math::INTEGRATIONSCHEME_LINEAR_RUNGE_KUTTA_4));

	scene->addSceneElement(
		createFem1D("Euler Implicit",
					makeRigidTranslation(Vector3d(0.0, -0.25, 0.0)),
					Vector4d(0, 0, 1, 1),
					SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_IMPLICIT));

	scene->addSceneElement(
		createFem1D("Static",
					makeRigidTranslation(Vector3d(0.0, -0.5, 0.0)),
					Vector4d(1, 1, 1, 1),
					SurgSim::Math::INTEGRATIONSCHEME_LINEAR_STATIC));

	runTest(Vector3d(0.0, 0.0, 2.0), Vector3d::Zero(), 5000.0);
}

}; // namespace Physics

}; // namespace SurgSim
