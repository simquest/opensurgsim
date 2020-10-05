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

///\file RenderTestMassSpringFrictionalSliding.cpp render test for MassSpringConstraintFrictionalSliding

#include <memory>

#include "SurgSim/Blocks/TransferPhysicsToGraphicsMeshBehavior.h"
#include "SurgSim/Blocks/TransferPhysicsToVerticesBehavior.h"
#include "SurgSim/Collision/ElementContactFilter.h"
#include "SurgSim/Collision/SegmentMeshTriangleMeshContact.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Graphics/OsgCurveRepresentation.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/SegmentMeshShape.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/ConstraintComponent.h"
#include "SurgSim/Physics/DeformableCollisionRepresentation.h"
#include "SurgSim/Physics/Fem2DRepresentation.h"
#include "SurgSim/Physics/MassSpringRepresentation.h"
#include "SurgSim/Physics/RenderTests/RenderTest.h"
#include "SurgSim/Physics/SlidingConstraint.h"

using SurgSim::Blocks::TransferPhysicsToGraphicsMeshBehavior;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Framework::checkAndConvert;
using SurgSim::Graphics::OsgCurveRepresentation;
using SurgSim::Graphics::OsgMeshRepresentation;
using SurgSim::Math::makeRigidTranslation;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;
using SurgSim::Physics::DeformableCollisionRepresentation;
using SurgSim::Physics::Fem2DRepresentation;
using SurgSim::Physics::MassSpringRepresentation;

namespace
{

// Generates a 2d fem comprised of a square.
std::shared_ptr<SurgSim::Framework::SceneElement> loadFem2D(const std::string& name,
	const SurgSim::Math::RigidTransform3d& pose,
	SurgSim::Math::IntegrationScheme integrationScheme)
{
	std::string fileName = "square_low_res.ply";
	auto femSceneElement = std::make_shared<BasicSceneElement>(name);

	auto collisionRepresentation = std::make_shared<DeformableCollisionRepresentation>("Collision " + name);
	auto meshShape = std::make_shared<SurgSim::Math::MeshShape>();
	meshShape->load(fileName);
	collisionRepresentation->setShape(meshShape);
	femSceneElement->addComponent(collisionRepresentation);

	auto physicsRepresentation = std::make_shared<Fem2DRepresentation>("Physics");
	physicsRepresentation->setLocalPose(pose);
	physicsRepresentation->loadFem(fileName);
	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);
	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(1e1);
	physicsRepresentation->setRayleighDampingStiffness(1e-1);
	femSceneElement->addComponent(physicsRepresentation);

	auto graphicsTriangleMeshRepresentation = std::make_shared<OsgMeshRepresentation>("TriangleMesh Representation");
	graphicsTriangleMeshRepresentation->loadMesh(fileName);
	femSceneElement->addComponent(graphicsTriangleMeshRepresentation);

	auto physicsToMesh = std::make_shared<TransferPhysicsToGraphicsMeshBehavior>("physics to triangle mesh");
	physicsToMesh->setSource(physicsRepresentation);
	physicsToMesh->setTarget(graphicsTriangleMeshRepresentation);
	femSceneElement->addComponent(physicsToMesh);

	return femSceneElement;
}

std::shared_ptr<SurgSim::Framework::SceneElement> createMassSpring1D(const std::string& name,
	const SurgSim::Math::RigidTransform3d& pose, Vector4d color,
	SurgSim::Math::IntegrationScheme integrationScheme)
{
	auto physicsRepresentation = std::make_shared<MassSpringRepresentation>("Physics");
	physicsRepresentation->setLocalPose(pose);
	std::string fileName = "MassSpring1D.ply";
	physicsRepresentation->loadMassSpring(fileName);
	physicsRepresentation->setIntegrationScheme(integrationScheme);
	physicsRepresentation->setRayleighDampingMass(1e1);
	physicsRepresentation->setRayleighDampingStiffness(1e-4);

	auto massSpringElement = std::make_shared<BasicSceneElement>(name);
	massSpringElement->addComponent(physicsRepresentation);

	auto graphicsRepresentation = std::make_shared<OsgCurveRepresentation>("Graphics Representation");
	graphicsRepresentation->setColor(color);
	graphicsRepresentation->setAntiAliasing(true);
	graphicsRepresentation->setWidth(3.0);
	massSpringElement->addComponent(graphicsRepresentation);

	auto copier = std::make_shared<SurgSim::Blocks::TransferPhysicsToVerticesBehavior>("Copier");
	copier->setSource(physicsRepresentation);
	copier->setTarget(graphicsRepresentation);
	massSpringElement->addComponent(copier);
	
	auto collisionRepresentation = std::make_shared<DeformableCollisionRepresentation>("Collision");
	auto shape = std::make_shared<SurgSim::Math::SegmentMeshShape>();
	shape->load(fileName);
	collisionRepresentation->setShape(shape);
	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);
	massSpringElement->addComponent(collisionRepresentation);
	
	return massSpringElement;
}

void addConstraint(std::shared_ptr<SurgSim::Framework::SceneElement> element,
	std::shared_ptr<SurgSim::Physics::MassSpringRepresentation> massSpring,
	std::shared_ptr<SurgSim::Physics::Fem2DRepresentation> fem2d, size_t massSpringIndex)
{
	std::vector<size_t> ignores;
	ignores.push_back(massSpringIndex);
	ignores.push_back(massSpringIndex + 1);
	ignores.push_back(massSpringIndex - 1);
	auto filter = std::make_shared<SurgSim::Collision::ElementContactFilter>("filter");
	filter->setFilter(fem2d->getCollisionRepresentation(), ignores);
	filter->setRepresentation(massSpring->getCollisionRepresentation());
	element->addComponent(filter);

	auto constraintComponent = std::make_shared<SurgSim::Physics::ConstraintComponent>("SlidingConstraint");
	auto constraintData = std::make_shared<SurgSim::Physics::SlidingConstraintData>();
	constraintData->setFrictionCoefficient(0.25);
	SurgSim::DataStructures::Location first;
	first.index = massSpringIndex;
	SurgSim::DataStructures::Location second;
	auto segmentMeshShape = std::dynamic_pointer_cast<SurgSim::Math::SegmentMeshShape>(
		massSpring->getCollisionRepresentation()->getShape());
	SURGSIM_ASSERT(segmentMeshShape != nullptr);
	auto triangleMeshShape = std::dynamic_pointer_cast<SurgSim::Math::MeshShape>(
		fem2d->getCollisionRepresentation()->getShape());
	SURGSIM_ASSERT(triangleMeshShape != nullptr);
	SurgSim::Collision::SegmentMeshTriangleMeshContact contact;
	auto contacts = contact.calculateDcdContact(
		*segmentMeshShape, massSpring->getPose() * massSpring->getCollisionRepresentation()->getLocalPose(),
		*triangleMeshShape, fem2d->getPose() * fem2d->getCollisionRepresentation()->getLocalPose());
	SURGSIM_ASSERT(contacts.size() > 0);
	
	Vector3d punctureDirection = Vector3d::UnitY();
	auto slidingConstraint = std::make_shared<SurgSim::Physics::SlidingConstraint>(SurgSim::Physics::FRICTIONAL_SLIDING,
		constraintData, massSpring, contacts.front()->penetrationPoints.first,
		fem2d, contacts.front()->penetrationPoints.second, punctureDirection);
	constraintComponent->setConstraint(slidingConstraint);
	element->addComponent(constraintComponent);
}

};

namespace SurgSim
{
namespace Physics
{

TEST_F(RenderTests, VisualTestMassSpringFrictionalSliding)
{
	//SurgSim::Framework::Logger::getLogger("Physics/MassSpringRepresentation")->setThreshold(SurgSim::Framework::LOG_LEVEL_DEBUG);
	//SurgSim::Framework::Logger::getLogger("Physics/LinearSpring")->setThreshold(SurgSim::Framework::LOG_LEVEL_DEBUG);
	// Create the fem2D, and find four points on it for the constraints.  Then use those points to place the sutures.
	auto square = loadFem2D("square",
		makeRigidTranslation(Vector3d::Zero()),
		SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EULER_IMPLICIT);
	auto squarePhysics = checkAndConvert<Fem2DRepresentation>(square->getComponent("Physics"),
			"SurgSim::Physics::Fem2DRepresentation");
	scene->addSceneElement(square);
	
	auto element = createMassSpring1D("MassSpring 1D Euler Implicit",
		makeRigidTranslation(Vector3d::Zero()), Vector4d(1, 0, 0, 1),
		SurgSim::Math::INTEGRATIONSCHEME_EULER_IMPLICIT);
	auto massSpring = checkAndConvert<MassSpringRepresentation>(element->getComponent("Physics"),
		"SurgSim::Physics::MassSpringRepresentation");
	// The square's pose is at y=0, and its nodes are all at y=0. The third node of the mass spring is also at y=0.
	// So we use index 2 for the constraint.
	size_t massSpringIndex = 2;
	addConstraint(element, massSpring, squarePhysics, massSpringIndex);
	scene->addSceneElement(element);
	
	auto cameraPosition = Vector3d(0.0, 0.1, 0.5);
	auto cameraLookAt = Vector3d::Zero();
	runTest(cameraPosition, cameraLookAt, 5000.0);
}

}; // namespace Physics
}; // namespace SurgSim
