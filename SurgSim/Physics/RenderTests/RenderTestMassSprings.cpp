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

///\file RenderTestMassSprings.cpp render test for MassSprings

#include <memory>

#include "SurgSim/Blocks/MassSpring1DRepresentation.h"
#include "SurgSim/Blocks/MassSpring2DRepresentation.h"
#include "SurgSim/Blocks/MassSpring3DRepresentation.h"
#include "SurgSim/Blocks/TransferPhysicsToPointCloudBehavior.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/RenderTests/RenderTest.h"

using SurgSim::Blocks::MassSpring1DRepresentation;
using SurgSim::Blocks::MassSpring2DRepresentation;
using SurgSim::Blocks::MassSpring3DRepresentation;
using SurgSim::Blocks::TransferPhysicsToPointCloudBehavior;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Graphics::OsgPointCloudRepresentation;
using SurgSim::Math::Vector3d;

namespace
{

std::shared_ptr<SurgSim::Framework::SceneElement> createMassSpring1D(const std::string& name,
		const SurgSim::Math::RigidTransform3d& gfxPose,
		SurgSim::Math::Vector4d color, SurgSim::Math::IntegrationScheme integrationScheme)
{
	std::shared_ptr<MassSpring1DRepresentation> physicsRepresentation =
		std::make_shared<MassSpring1DRepresentation>(name + " Physics");

	// In this test, the physics representations are not transformed,
	// only the graphics one will apply a transform

	std::vector<size_t> nodeBoundaryConditions;
	nodeBoundaryConditions.push_back(0);
	// Adding in this boundary condition is not necessary for the physics of the system, but it allows us
	// to better control the condition number for the linear system that the OdeStaticSolver
	// generates. This results in a more stable and more accurate test.
	nodeBoundaryConditions.push_back(1);

	// MassSpring1D with a straight line would define springs only along 1 direction, which would result in
	// stiffness matrix of rank n/3. The Z axis can be constrained entirely and the simulation
	// be done in exclusively in 2D (X,Y), but for the 2nd dimension to be defined properly in the stiffness matrix
	// we need a shape that does not have any straight lines anywhere. This will ensure that the problem is
	// well defined in 2D. This problem only arise in static resolution (OdeSolverStatic).
	std::vector<SurgSim::Math::Vector3d> nodes;
	nodes.push_back(Vector3d(-0.5, 0.0, 0));
	nodes.push_back(Vector3d(-0.3, -0.5, 0));
	nodes.push_back(Vector3d(-0.1, -0.4, 0));
	nodes.push_back(Vector3d(0.1, -0.5, 0));
	nodes.push_back(Vector3d(0.3, 0.0, 0));
	physicsRepresentation->init1D(nodes,
								  nodeBoundaryConditions,
								  0.1, // total mass (in Kg)
								  5.0, // Stiffness stretching
								  0.5, // Damping stretching
								  3.0, // Stiffness bending
								  0.5); // Damping bending

	// MassSpring1D defines springs only on the XY plane, so the stiffness matrix will contains
	// rows and columns of 0 for all Z axis. Therefore we need to constrain all Z dof to entirely
	// define the problem (stiffness matrix invertible in static resolution (OdeSolverStatic)).
	for (size_t nodeId = 1; nodeId < nodes.size(); ++nodeId)
	{
		SurgSim::Math::OdeState* state =
			const_cast<SurgSim::Math::OdeState*>(physicsRepresentation->getInitialState().get());
		state->addBoundaryCondition(nodeId, 2);
	}

	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(1e0);
	physicsRepresentation->setRayleighDampingStiffness(3e-2);

	std::shared_ptr<BasicSceneElement> massSpringElement = std::make_shared<BasicSceneElement>(name);
	massSpringElement->addComponent(physicsRepresentation);

	std::shared_ptr<OsgPointCloudRepresentation> graphicsRepresentation =
		std::make_shared<OsgPointCloudRepresentation>("Graphics object");
	graphicsRepresentation->setLocalPose(gfxPose);
	graphicsRepresentation->setColor(color);
	graphicsRepresentation->setPointSize(3.0f);
	graphicsRepresentation->setLocalActive(true);
	massSpringElement->addComponent(graphicsRepresentation);

	auto physicsToGraphics =
		std::make_shared<TransferPhysicsToPointCloudBehavior>("Physics to Graphics deformable points");
	physicsToGraphics->setSource(physicsRepresentation);
	physicsToGraphics->setTarget(graphicsRepresentation);

	massSpringElement->addComponent(physicsToGraphics);
	return massSpringElement;

}

std::shared_ptr<SurgSim::Framework::SceneElement> createMassSpring2D(const std::string& name,
		const SurgSim::Math::RigidTransform3d& gfxPose,
		SurgSim::Math::Vector4d color, SurgSim::Math::IntegrationScheme integrationScheme)
{
	std::shared_ptr<MassSpring2DRepresentation> physicsRepresentation =
		std::make_shared<MassSpring2DRepresentation>(name + " Physics");

	// In this test, the physics representations are not transformed,
	// only the graphics one will apply a transform

	// The regular structure of the MassSpring2D (in X,Y plane) makes the stiffness matrix close to singular
	// (Z axis aside) even with 1 fixed node. We run this test with 2 fixed nodes to ensure that a static resolution
	// will be stable (OdeSolverStatic).
	std::vector<size_t> nodeBoundaryConditions;
	nodeBoundaryConditions.push_back(0);
	nodeBoundaryConditions.push_back(1);
	std::array<std::array<SurgSim::Math::Vector3d, 2>, 2> extremities =
	{
		{
			{{ Vector3d(-0.5, 0.5, 0), Vector3d(0.5, 0.5, 0) }},
			{{ Vector3d(-0.5, -0.5, 0), Vector3d(0.5, -0.5, 0) }}
		}
	};
	size_t numNodesPerDim[2] = {3, 3};
	physicsRepresentation->init2D(extremities,
								  numNodesPerDim,
								  nodeBoundaryConditions,
								  0.1, // total mass (in Kg)
								  5.0, // Stiffness stretching
								  0.5, // Damping stretching
								  1.0, // Stiffness bending
								  0.5, // Damping bending
								  1.0, // Stiffness face diagonal
								  0.5); // Damping face diagonal

	// MassSpring2D defines springs only on the XY plane, so the stiffness matrix will contains
	// rows and columns of 0 for all Z axis. Therefore we need to constrain all Z dof to entirely
	// define the problem (stiffness matrix invertible in static resolution (OdeSolverStatic)).
	for (size_t nodeId = 2; nodeId < numNodesPerDim[0] * numNodesPerDim[1]; ++nodeId)
	{
		SurgSim::Math::OdeState* state =
			const_cast<SurgSim::Math::OdeState*>(physicsRepresentation->getInitialState().get());
		state->addBoundaryCondition(nodeId, 2);
	}

	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(1e0);
	physicsRepresentation->setRayleighDampingStiffness(3e-2);

	std::shared_ptr<BasicSceneElement> massSpringElement = std::make_shared<BasicSceneElement>(name);
	massSpringElement->addComponent(physicsRepresentation);

	std::shared_ptr<OsgPointCloudRepresentation> graphicsRepresentation =
		std::make_shared<OsgPointCloudRepresentation>("Graphics object");
	graphicsRepresentation->setLocalPose(gfxPose);
	graphicsRepresentation->setColor(color);
	graphicsRepresentation->setPointSize(3.0f);
	graphicsRepresentation->setLocalActive(true);
	massSpringElement->addComponent(graphicsRepresentation);
	auto physicsToGraphics =
		std::make_shared<TransferPhysicsToPointCloudBehavior>("Physics to Graphics deformable points");
	physicsToGraphics->setSource(physicsRepresentation);
	physicsToGraphics->setTarget(graphicsRepresentation);
	massSpringElement->addComponent(physicsToGraphics);

	return massSpringElement;
}

std::shared_ptr<SurgSim::Framework::SceneElement> createMassSpring3D(const std::string& name,
		const SurgSim::Math::RigidTransform3d& gfxPose,
		SurgSim::Math::Vector4d color, SurgSim::Math::IntegrationScheme integrationScheme)
{
	std::shared_ptr<MassSpring3DRepresentation> physicsRepresentation =
		std::make_shared<MassSpring3DRepresentation>(name + " Physics");

	// In this test, the physics representations are not transformed,
	// only the graphics one will apply a transform

	std::vector<size_t> nodeBoundaryConditions;
	nodeBoundaryConditions.push_back(0);
	nodeBoundaryConditions.push_back(1);
	// Adding in these two boundary conditions is not necessary for the physics of the system, but it allows us
	// to better control the condition number for the linear system that the OdeStaticSolver
	// generates. This results in a more stable and more accurate test.
	nodeBoundaryConditions.push_back(2);
	nodeBoundaryConditions.push_back(3);
	std::array<std::array<std::array<SurgSim::Math::Vector3d, 2>, 2>, 2> extremities =
	{
		{
			{{
					{{ Vector3d(-0.5, 0.5, 0.5), Vector3d(0.5, 0.5, 0.5) }}
					,
					{{ Vector3d(-0.5, -0.5, 0.5), Vector3d(0.5, -0.5, 0.5) }}
				}
			}
			,
			{{
					{{ Vector3d(-0.5, 0.5, -0.5), Vector3d(0.5, 0.5, -0.5) }}
					,
					{{ Vector3d(-0.5, -0.5, -0.5), Vector3d(0.5, -0.5, -0.5) }}
				}
			},
		}
	};
	size_t numNodesPerDim[3] = {3, 3, 3};
	physicsRepresentation->init3D(extremities,
								  numNodesPerDim,
								  nodeBoundaryConditions,
								  0.1, // total mass (in Kg)
								  5.0, // Stiffness stretching
								  0.5, // Damping stretching
								  1.0, // Stiffness bending
								  0.5,  // Damping bending
								  1.0, // Stiffness face diagonal
								  0.5, // Damping face diagonal
								  1.0, // Stiffness volume diagonal
								  0.5); // Damping volume diagonal

	// MassSpring 3D defines springs in all directions (X, Y, Z, XY, XZ, YZ, XYZ) so the stiffness matrix
	// is entirely defined (no rows or columns of 0).
	// Therefore, setting up one node as boundary condition is sufficient to make the static problem invertible.

	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(1e0);
	physicsRepresentation->setRayleighDampingStiffness(3e-2);

	std::shared_ptr<BasicSceneElement> massSpringElement = std::make_shared<BasicSceneElement>(name);
	massSpringElement->addComponent(physicsRepresentation);

	std::shared_ptr<OsgPointCloudRepresentation> graphicsRepresentation =
		std::make_shared<OsgPointCloudRepresentation>("Graphics object");
	graphicsRepresentation->setLocalPose(gfxPose);
	graphicsRepresentation->setColor(color);
	graphicsRepresentation->setPointSize(3.0f);
	graphicsRepresentation->setLocalActive(true);
	massSpringElement->addComponent(graphicsRepresentation);

	auto physicsToGraphics =
		std::make_shared<TransferPhysicsToPointCloudBehavior>("Physics to Graphics deformable points");
	physicsToGraphics->setSource(physicsRepresentation);
	physicsToGraphics->setTarget(graphicsRepresentation);
	massSpringElement->addComponent(physicsToGraphics);

	return massSpringElement;
}

}; // anonymous namespace

namespace SurgSim
{

namespace Physics
{

TEST_F(RenderTests, VisualTestMassSprings)
{
	using SurgSim::Math::makeRigidTranslation;
	using SurgSim::Math::Vector4d;

	// MassSpring1D
	{
		scene->addSceneElement(createMassSpring1D("MassSpring 1D Euler Explicit",
							   makeRigidTranslation(Vector3d(-3.0, 3.0, 0.0)), Vector4d(1, 0, 0, 1),
							   SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER));

		scene->addSceneElement(createMassSpring1D("MassSpring 1D Modified Euler Explicit",
							   makeRigidTranslation(Vector3d(-1.25, 3.0, 0.0)), Vector4d(0.5, 0, 0, 1),
							   SurgSim::Math::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER));

		scene->addSceneElement(createMassSpring1D("MassSpring 1D Runge Kutta 4",
							   makeRigidTranslation(Vector3d(0.5, 3.0, 0.0)), Vector4d(0, 1, 0, 1),
							   SurgSim::Math::INTEGRATIONSCHEME_RUNGE_KUTTA_4));

		scene->addSceneElement(createMassSpring1D("MassSpring 1D Euler Implicit",
							   makeRigidTranslation(Vector3d(2.25, 3.0, 0.0)), Vector4d(0, 0, 1, 1),
							   SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER));

		scene->addSceneElement(createMassSpring1D("MassSpring 1D Static",
							   makeRigidTranslation(Vector3d(4.0, 3.0, 0.0)), Vector4d(1, 1, 1, 1),
							   SurgSim::Math::INTEGRATIONSCHEME_STATIC));
	}

	// MassSpring2D
	{
		scene->addSceneElement(createMassSpring2D("MassSpring 2D Euler Explicit",
							   makeRigidTranslation(Vector3d(-3.0, 0.5, 0.0)), Vector4d(1, 0, 0, 1),
							   SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER));

		scene->addSceneElement(createMassSpring2D("MassSpring 2D Modified Euler Explicit",
							   makeRigidTranslation(Vector3d(-1.25, 0.5, 0.0)), Vector4d(0.5, 0, 0, 1),
							   SurgSim::Math::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER));

		scene->addSceneElement(createMassSpring2D("MassSpring 2D Runge Kutta 4",
							   makeRigidTranslation(Vector3d(0.5, 0.5, 0.0)), Vector4d(0, 1, 0, 1),
							   SurgSim::Math::INTEGRATIONSCHEME_RUNGE_KUTTA_4));

		scene->addSceneElement(createMassSpring2D("MassSpring 2D Euler Implicit",
							   makeRigidTranslation(Vector3d(2.25, 0.5, 0.0)), Vector4d(0, 0, 1, 1),
							   SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER));

		scene->addSceneElement(createMassSpring2D("MassSpring 2D Static",
							   makeRigidTranslation(Vector3d(4.0, 0.5, 0.0)), Vector4d(1, 1, 1, 1),
							   SurgSim::Math::INTEGRATIONSCHEME_STATIC));
	}

	// MassSpring3D
	{
		scene->addSceneElement(createMassSpring3D("MassSpring 3D Euler Explicit",
							   makeRigidTranslation(Vector3d(-3.0, -1.5, 0.0)), Vector4d(1, 0, 0, 1),
							   SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER));

		scene->addSceneElement(createMassSpring3D("MassSpring 3D Modified Euler Explicit",
							   makeRigidTranslation(Vector3d(-1.25, -1.5, 0.0)), Vector4d(0.5, 0, 0, 1),
							   SurgSim::Math::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER));

		scene->addSceneElement(createMassSpring3D("MassSpring 3D Runge Kutta 4",
							   makeRigidTranslation(Vector3d(0.5, -1.5, 0.0)), Vector4d(0, 1, 0, 1),
							   SurgSim::Math::INTEGRATIONSCHEME_RUNGE_KUTTA_4));

		scene->addSceneElement(createMassSpring3D("MassSpring 3D Euler Implicit",
							   makeRigidTranslation(Vector3d(2.25, -1.5, 0.0)), Vector4d(0, 0, 1, 1),
							   SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER));

		scene->addSceneElement(createMassSpring3D("MassSpring 3D Static",
							   makeRigidTranslation(Vector3d(4.0, -1.5, 0.0)), Vector4d(1, 1, 1, 1),
							   SurgSim::Math::INTEGRATIONSCHEME_STATIC));
	}

	runTest(Vector3d(0.0, 0.0, 8.5), Vector3d::Zero(), 15000.0);
}

}; // namespace Physics

}; // namespace SurgSim
