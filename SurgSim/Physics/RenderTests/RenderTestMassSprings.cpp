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
#include "SurgSim/Blocks/TransferDeformableStateToVerticesBehavior.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/RenderTests/RenderTest.h"

using SurgSim::Blocks::MassSpring1DRepresentation;
using SurgSim::Blocks::MassSpring2DRepresentation;
using SurgSim::Blocks::MassSpring3DRepresentation;
using SurgSim::Blocks::TransferDeformableStateToVerticesBehavior;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Graphics::OsgPointCloudRepresentation;
using SurgSim::Math::Vector3d;


namespace
{

std::shared_ptr<SurgSim::Framework::SceneElement> createMassSpring1D(const std::string& name,
		const std::vector<SurgSim::Math::RigidTransform3d> gfxPoses,
		SurgSim::Math::Vector4d color, SurgSim::Math::IntegrationScheme integrationScheme)
{
	std::shared_ptr<MassSpring1DRepresentation> physicsRepresentation =
		std::make_shared<MassSpring1DRepresentation>(name + " Physics");

	// In this test, the physics representations are not transformed,
	// only the graphics one will apply a transform

	std::vector<unsigned int> boundaryConditions;
	boundaryConditions.push_back(0);
	boundaryConditions.push_back(1);
	boundaryConditions.push_back(2);
	std::array<SurgSim::Math::Vector3d, 2> extremities = {{ Vector3d(0, 0, 0), Vector3d(1, 0, 0) }};
	unsigned int numNodesPerDim[1] = {6};
	physicsRepresentation->init1D(extremities,
								  numNodesPerDim,
								  boundaryConditions,
								  0.1, // total mass (in Kg)
								  5.0, // Stiffness stretching
								  0.5, // Damping stretching
								  1.0, // Stiffness bending
								  0.5); // Damping bending

	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(1e-1);
	physicsRepresentation->setRayleighDampingStiffness(1e-2);

	std::shared_ptr<BasicSceneElement> massSpringElement = std::make_shared<BasicSceneElement>(name);
	massSpringElement->addComponent(physicsRepresentation);

	unsigned int gfxObjectId = 0;
	for (auto gfxPose = std::begin(gfxPoses); gfxPose != std::end(gfxPoses); gfxPose++)
	{
		std::stringstream ss;
		ss << name + " Graphics object " << gfxObjectId;
		std::shared_ptr<OsgPointCloudRepresentation<void>> graphicsRepresentation =
					std::make_shared<OsgPointCloudRepresentation<void>>(ss.str());

		graphicsRepresentation->setLocalPose(*gfxPose);
		graphicsRepresentation->setColor(color);
		graphicsRepresentation->setPointSize(3.0f);
		graphicsRepresentation->setVisible(true);

		massSpringElement->addComponent(graphicsRepresentation);
		ss.clear();
		ss << "Physics to Graphics (" << gfxObjectId << ") deformable points";
		massSpringElement->addComponent(std::make_shared<TransferDeformableStateToVerticesBehavior<void>>
										(ss.str(),
										 physicsRepresentation->getFinalState(),
										 graphicsRepresentation->getVertices()));

		gfxObjectId++;
	}

	return massSpringElement;

}

std::shared_ptr<SurgSim::Framework::SceneElement> createMassSpring2D(const std::string& name,
		const std::vector<SurgSim::Math::RigidTransform3d> gfxPoses,
		SurgSim::Math::Vector4d color, SurgSim::Math::IntegrationScheme integrationScheme)
{
	std::shared_ptr<MassSpring2DRepresentation> physicsRepresentation =
		std::make_shared<MassSpring2DRepresentation>(name + " Physics");

	// In this test, the physics representations are not transformed,
	// only the graphics one will apply a transform

	std::vector<unsigned int> boundaryConditions;
	boundaryConditions.push_back(0);
	boundaryConditions.push_back(1);
	boundaryConditions.push_back(2);
	std::array<std::array<SurgSim::Math::Vector3d, 2>, 2> extremities =
	{
		{
			{{ Vector3d(0, 0, 0), Vector3d(1, 0, 0) }},
			{{ Vector3d(0, -1, 0), Vector3d(1, -1, 0) }}
		}
	};
	unsigned int numNodesPerDim[2] = {3, 3};
	physicsRepresentation->init2D(extremities,
								  numNodesPerDim,
								  boundaryConditions,
								  0.1, // total mass (in Kg)
								  5.0, // Stiffness stretching
								  0.5, // Damping stretching
								  1.0, // Stiffness bending
								  0.5, // Damping bending
								  1.0, // Stiffness face diagonal
								  0.5); // Damping face diagonal

	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(1e-1);
	physicsRepresentation->setRayleighDampingStiffness(1e-2);

	std::shared_ptr<BasicSceneElement> massSpringElement = std::make_shared<BasicSceneElement>(name);
	massSpringElement->addComponent(physicsRepresentation);

	unsigned int gfxObjectId = 0;
	for (auto gfxPose = std::begin(gfxPoses); gfxPose != std::end(gfxPoses); gfxPose++)
	{
		std::stringstream ss;
		ss << name + " Graphics object " << gfxObjectId;
		std::shared_ptr<OsgPointCloudRepresentation<void>> graphicsRepresentation =
					std::make_shared<OsgPointCloudRepresentation<void>>(ss.str());

		graphicsRepresentation->setLocalPose(*gfxPose);
		graphicsRepresentation->setColor(color);
		graphicsRepresentation->setPointSize(3.0f);
		graphicsRepresentation->setVisible(true);

		massSpringElement->addComponent(graphicsRepresentation);
		ss.clear();
		ss << "Physics to Graphics (" << gfxObjectId << ") deformable points";
		massSpringElement->addComponent(std::make_shared<TransferDeformableStateToVerticesBehavior<void>>
										(ss.str(),
										 physicsRepresentation->getFinalState(),
										 graphicsRepresentation->getVertices()));

		gfxObjectId++;
	}

	return massSpringElement;
}

std::shared_ptr<SurgSim::Framework::SceneElement> createMassSpring3D(const std::string& name,
		const std::vector<SurgSim::Math::RigidTransform3d> gfxPoses,
		SurgSim::Math::Vector4d color, SurgSim::Math::IntegrationScheme integrationScheme)
{
	std::shared_ptr<MassSpring3DRepresentation> physicsRepresentation =
		std::make_shared<MassSpring3DRepresentation>(name + " Physics");

	// In this test, the physics representations are not transformed,
	// only the graphics one will apply a transform

	std::vector<unsigned int> boundaryConditions;
	boundaryConditions.push_back(0);
	boundaryConditions.push_back(1);
	boundaryConditions.push_back(2);
	std::array<std::array<std::array<SurgSim::Math::Vector3d, 2>, 2>, 2> extremities =
	{
		{
			{{
					{{ Vector3d(0, 0, 0), Vector3d(1, 0, 0) }}
					,
					{{ Vector3d(0, -1, 0), Vector3d(1, -1, 0) }}
				}
			}
			,
			{{
					{{ Vector3d(0, 0, -1), Vector3d(1, 0, -1) }}
					,
					{{ Vector3d(0, -1, -1), Vector3d(1, -1, -1) }}
				}
			},
		}
	};
	unsigned int numNodesPerDim[3] = {3, 3, 3};
	physicsRepresentation->init3D(extremities,
								  numNodesPerDim,
								  boundaryConditions,
								  0.1, // total mass (in Kg)
								  5.0, // Stiffness stretching
								  0.5, // Damping stretching
								  1.0, // Stiffness bending
								  0.5,  // Damping bending
								  1.0, // Stiffness face diagonal
								  0.5, // Damping face diagonal
								  1.0, // Stiffness volume diagonal
								  0.5); // Damping volume diagonal

	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(1e-1);
	physicsRepresentation->setRayleighDampingStiffness(1e-2);

	std::shared_ptr<BasicSceneElement> massSpringElement = std::make_shared<BasicSceneElement>(name);
	massSpringElement->addComponent(physicsRepresentation);

	unsigned int gfxObjectId = 0;
	for (auto gfxPose = std::begin(gfxPoses); gfxPose != std::end(gfxPoses); gfxPose++)
	{
		std::stringstream ss;
		ss << name + " Graphics object " << gfxObjectId;
		std::shared_ptr<OsgPointCloudRepresentation<void>> graphicsRepresentation =
					std::make_shared<OsgPointCloudRepresentation<void>>(ss.str());

		graphicsRepresentation->setLocalPose(*gfxPose);
		graphicsRepresentation->setColor(color);
		graphicsRepresentation->setPointSize(3.0f);
		graphicsRepresentation->setVisible(true);

		massSpringElement->addComponent(graphicsRepresentation);
		ss.clear();
		ss << "Physics to Graphics (" << gfxObjectId << ") deformable points";
		massSpringElement->addComponent(std::make_shared<TransferDeformableStateToVerticesBehavior<void>>
										(ss.str(),
										 physicsRepresentation->getFinalState(),
										 graphicsRepresentation->getVertices()));

		gfxObjectId++;
	}

	return massSpringElement;
}

}; // anonymous namespace

namespace SurgSim
{

namespace Physics
{

TEST_F(RenderTests, VisualTestMassSprings)
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::Vector4d;

	SurgSim::Math::Quaterniond qIdentity = SurgSim::Math::Quaterniond::Identity();

	// MassSpring1D
	{
		std::vector<SurgSim::Math::RigidTransform3d> gfxPoses;
		gfxPoses.push_back(makeRigidTransform(qIdentity, Vector3d(-3.0, 3.0, 0.0)));
		gfxPoses.push_back(makeRigidTransform(qIdentity, Vector3d(3.0, 3.0, 0.0)));
		scene->addSceneElement(createMassSpring1D("MassSpring 1D Euler Explicit",
							   gfxPoses, Vector4d(1, 0, 0, 1),
							   SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER));

		gfxPoses.clear();
		gfxPoses.push_back(makeRigidTransform(qIdentity, Vector3d(-1.0, 3.0, 0.0)));
		gfxPoses.push_back(makeRigidTransform(qIdentity, Vector3d(3.0, 3.0, 0.0)));
		scene->addSceneElement(createMassSpring1D("MassSpring 1D Modified Euler Explicit",
							   gfxPoses, Vector4d(0, 1, 0, 1),
							   SurgSim::Math::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER));

		gfxPoses.clear();
		gfxPoses.push_back(makeRigidTransform(qIdentity, Vector3d(1.0, 3.0, 0.0)));
		gfxPoses.push_back(makeRigidTransform(qIdentity, Vector3d(3.0, 3.0, 0.0)));
		scene->addSceneElement(createMassSpring1D("MassSpring 1D Euler Implicit",
							   gfxPoses, Vector4d(0, 0, 1, 1),
							   SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER));
	}

	// MassSpring2D
	{
		std::vector<SurgSim::Math::RigidTransform3d> gfxPoses;
		gfxPoses.push_back(makeRigidTransform(qIdentity, Vector3d(-3.0, 1.5, 0.0)));
		gfxPoses.push_back(makeRigidTransform(qIdentity, Vector3d(3.0, 1.5, 0.0)));
		scene->addSceneElement(createMassSpring2D("MassSpring 2D Euler Explicit",
							   gfxPoses, Vector4d(1, 0, 0, 1),
							   SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER));

		gfxPoses.clear();
		gfxPoses.push_back(makeRigidTransform(qIdentity, Vector3d(-1.0, 1.5, 0.0)));
		gfxPoses.push_back(makeRigidTransform(qIdentity, Vector3d(3.0, 1.5, 0.0)));
		scene->addSceneElement(createMassSpring2D("MassSpring 2D Modified Euler Explicit",
							   gfxPoses, Vector4d(0, 1, 0, 1),
							   SurgSim::Math::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER));

		gfxPoses.clear();
		gfxPoses.push_back(makeRigidTransform(qIdentity, Vector3d(1.0, 1.5, 0.0)));
		gfxPoses.push_back(makeRigidTransform(qIdentity, Vector3d(3.0, 1.5, 0.0)));
		scene->addSceneElement(createMassSpring2D("MassSpring 2D Euler Implicit",
							   gfxPoses, Vector4d(0, 0, 1, 1),
							   SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER));
	}

	// MassSpring3D
	{
		std::vector<SurgSim::Math::RigidTransform3d> gfxPoses;
		gfxPoses.push_back(makeRigidTransform(qIdentity, Vector3d(-3.0, -0.5, 0.0)));
		gfxPoses.push_back(makeRigidTransform(qIdentity, Vector3d(3.0, -0.5, 0.0)));
		scene->addSceneElement(createMassSpring3D("MassSpring 3D Euler Explicit",
							   gfxPoses, Vector4d(1, 0, 0, 1),
							   SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER));

		gfxPoses.clear();
		gfxPoses.push_back(makeRigidTransform(qIdentity, Vector3d(-1.0, -0.5, 0.0)));
		gfxPoses.push_back(makeRigidTransform(qIdentity, Vector3d(3.0, -0.5, 0.0)));
		scene->addSceneElement(createMassSpring3D("MassSpring 3D Modified Euler Explicit",
							   gfxPoses, Vector4d(0, 1, 0, 1),
							   SurgSim::Math::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER));

		gfxPoses.clear();
		gfxPoses.push_back(makeRigidTransform(qIdentity, Vector3d(1.0, -0.5, 0.0)));
		gfxPoses.push_back(makeRigidTransform(qIdentity, Vector3d(3.0, -0.5, 0.0)));
		scene->addSceneElement(createMassSpring3D("MassSpring 3D Euler Implicit",
							   gfxPoses, Vector4d(0, 0, 1, 1),
							   SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER));
	}

	runTest(Vector3d(0.0, 0.0, 8.0), Vector3d::Zero(), 3000.0);
}

}; // namespace Physics

}; // namespace SurgSim
