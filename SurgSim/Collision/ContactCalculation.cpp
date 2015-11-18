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

#include <thread>

#include "SurgSim/Framework/Log.h"

#include "SurgSim/Collision/ContactCalculation.h"

#include "SurgSim/Collision/DcdCollision.h"
#include "SurgSim/Collision/DefaultContactCalculation.h"
#include "SurgSim/Collision/Representation.h"

namespace SurgSim
{
namespace Collision
{

ContactCalculation::TableType ContactCalculation::m_contactDcdCalculations;
ContactCalculation::TableType ContactCalculation::m_contactCcdCalculations;
std::once_flag ContactCalculation::m_initializationFlag;

ContactCalculation::ContactCalculation()
{
}

ContactCalculation::~ContactCalculation()
{
}

void ContactCalculation::registerDcdContactCalculation(const std::shared_ptr<ContactCalculation>& calculation)
{
	std::call_once(m_initializationFlag, ContactCalculation::initializeTables);

	privateDcdRegister(calculation, calculation->getShapeTypes());
}

void ContactCalculation::registerCcdContactCalculation(const std::shared_ptr<ContactCalculation>& calculation)
{
	std::call_once(m_initializationFlag, ContactCalculation::initializeTables);

	privateCcdRegister(calculation, calculation->getShapeTypes());
}

const ContactCalculation::TableType& ContactCalculation::getDcdContactTable()
{
	std::call_once(m_initializationFlag, ContactCalculation::initializeTables);

	return m_contactDcdCalculations;
}

const ContactCalculation::TableType& ContactCalculation::getCcdContactTable()
{
	std::call_once(m_initializationFlag, ContactCalculation::initializeTables);

	return m_contactCcdCalculations;
}

void ContactCalculation::calculateContact(std::shared_ptr<CollisionPair> pair)
{
	doCalculateContact(pair);
}

std::list<std::shared_ptr<Contact>> ContactCalculation::calculateDcdContact(
	const std::shared_ptr<Math::Shape>& shape1, const Math::RigidTransform3d& pose1,
	const std::shared_ptr<Math::Shape>& shape2, const Math::RigidTransform3d& pose2)
{
	auto types = getShapeTypes();
	auto incoming = std::make_pair(shape1->getType(), shape2->getType());
	if (incoming == types)
	{
		return doCalculateDcdContact(shape1, pose1, shape2, pose2);
	}

	if (incoming.first == types.second && incoming.second == types.first)
	{
		auto contacts = doCalculateDcdContact(shape2, pose2, shape1, pose1);
		for (const auto& contact : contacts)
		{
			contact->normal = -contact->normal;
			contact->force = -contact->force;
		}
		return contacts;
	}

	SURGSIM_FAILURE() << "Incorrect shape type for this calculation expected "
					  << types.first << ", " << types.second
					  << " received " << incoming.first << ", " << incoming.second << ".";
	return std::list<std::shared_ptr<Contact>>();
}

void ContactCalculation::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
	std::pair<int, int> shapeTypes = getShapeTypes();
	int firstShapeType = pair->getFirst()->getShapeType();
	int secondShapeType = pair->getSecond()->getShapeType();

	if (firstShapeType != secondShapeType && firstShapeType == shapeTypes.second &&
		secondShapeType == shapeTypes.first)
	{
		pair->swapRepresentations();
		std::swap(firstShapeType, secondShapeType);
	}

	if (shapeTypes.first != SurgSim::Math::SHAPE_TYPE_NONE)
	{
		SURGSIM_ASSERT(firstShapeType == shapeTypes.first) <<
				"First Object, wrong type of object" << firstShapeType;
	}

	if (shapeTypes.second != SurgSim::Math::SHAPE_TYPE_NONE)
	{
		SURGSIM_ASSERT(secondShapeType == shapeTypes.second) <<
				"Second Object, wrong type of object" << secondShapeType;
	}

	std::shared_ptr<Math::Shape> shape1 = pair->getFirst()->getShape();
	if (shape1->isTransformable())
	{
		shape1 = pair->getFirst()->getPosedShape();
	}

	std::shared_ptr<Math::Shape> shape2 = pair->getSecond()->getShape();
	if (shape2->isTransformable())
	{
		shape2 = pair->getSecond()->getPosedShape();
	}

	// Are we using Dcd or Ccd for this pair ?
	std::list<std::shared_ptr<Contact>> contacts;
	if (pair->getType() == Collision::CollisionDetectionType::COLLISION_DETECTION_TYPE_DISCRETE)
	{
		contacts = doCalculateDcdContact(
			shape1, pair->getFirst()->getPose(),
			shape2, pair->getSecond()->getPose());
	}
	else if (pair->getType() == Collision::CollisionDetectionType::COLLISION_DETECTION_TYPE_CONTINUOUS)
	{
		Math::PosedShape posedShape1AtTime0, posedShape1AtTime1, posedShape2AtTime0, posedShape2AtTime1;

		posedShape1AtTime0 = pair->getFirst()->getPreviousPosedShape();
		posedShape1AtTime1 = pair->getFirst()->getCurrentPosedShape();

		posedShape2AtTime0 = pair->getSecond()->getPreviousPosedShape();
		posedShape2AtTime1 = pair->getSecond()->getCurrentPosedShape();

		contacts = doCalculateCcdContact(
			posedShape1AtTime0.m_shape, posedShape1AtTime0.m_pose,
			posedShape1AtTime1.m_shape, posedShape1AtTime1.m_pose,
			posedShape2AtTime0.m_shape, posedShape2AtTime0.m_pose,
			posedShape2AtTime1.m_shape, posedShape2AtTime1.m_pose);
	}
	else
	{
		using Framework::Logger;
		SURGSIM_LOG_ONCE(Logger::getLogger("ContactCalculation"), WARNING) <<
			"Invalid collision detection type, neither discrete nor continuous";
	}

	for (auto& contact : contacts)
	{
		pair->addContact(contact);
	}
}

void ContactCalculation::initializeTables()
{
	// Fill up both tables with default empty contact calculation
	for (int i = 0; i < SurgSim::Math::SHAPE_TYPE_COUNT; ++i)
	{
		for (int j = 0; j < SurgSim::Math::SHAPE_TYPE_COUNT; ++j)
		{
			m_contactDcdCalculations[i][j].reset(new Collision::DefaultContactCalculation(false));
			m_contactCcdCalculations[i][j].reset(new Collision::DefaultContactCalculation(false));
		}
	}

	// Fill up the Dcd contact calculation table
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::BoxCapsuleDcdContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::BoxDoubleSidedPlaneDcdContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::BoxPlaneDcdContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::BoxSphereDcdContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::CapsuleSphereDcdContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::OctreeCapsuleDcdContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::OctreeDoubleSidedPlaneDcdContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::OctreePlaneDcdContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::OctreeSphereDcdContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::SegmentMeshTriangleMeshDcdContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::SphereSphereDcdContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::SphereDoubleSidedPlaneDcdContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::SpherePlaneDcdContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::TriangleMeshParticlesDcdContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::TriangleMeshPlaneDcdContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::TriangleMeshTriangleMeshDcdContact>());

	const std::array<int, Math::SHAPE_TYPE_COUNT> allshapes =
	{
		Math::SHAPE_TYPE_BOX,
		Math::SHAPE_TYPE_CAPSULE,
		Math::SHAPE_TYPE_CYLINDER,
		Math::SHAPE_TYPE_DOUBLESIDEDPLANE,
		Math::SHAPE_TYPE_MESH,
		Math::SHAPE_TYPE_OCTREE,
		Math::SHAPE_TYPE_PARTICLES,
		Math::SHAPE_TYPE_PLANE,
		Math::SHAPE_TYPE_SPHERE,
		Math::SHAPE_TYPE_SURFACEMESH,
		Math::SHAPE_TYPE_SEGMENTMESH,
		Math::SHAPE_TYPE_COMPOUNDSHAPE
	};

	for (auto type : allshapes)
	{
		ContactCalculation::privateDcdRegister(std::make_shared<Collision::CompoundShapeDcdContact>(
											   std::make_pair(Math::SHAPE_TYPE_COMPOUNDSHAPE, type)));
	}

	// Fill up the Ccd contact calculation table
	//ContactCalculation::privateCcdRegister(std::make_shared<Collision::SegmentCcdSelfContact>());
}

void ContactCalculation::privateDcdRegister(
	const std::shared_ptr<ContactCalculation>& calculation)
{
	privateDcdRegister(calculation, calculation->getShapeTypes());
}

void ContactCalculation::privateDcdRegister(
	const std::shared_ptr<ContactCalculation>& calculation,
	const std::pair<int, int>& types)
{
	m_contactDcdCalculations[types.first][types.second] = calculation;
	m_contactDcdCalculations[types.second][types.first] = calculation;
}

void ContactCalculation::privateCcdRegister(
	const std::shared_ptr<ContactCalculation>& calculation)
{
	privateCcdRegister(calculation, calculation->getShapeTypes());
}

void ContactCalculation::privateCcdRegister(
	const std::shared_ptr<ContactCalculation>& calculation,
	const std::pair<int, int>& types)
{
	m_contactCcdCalculations[types.first][types.second] = calculation;
	m_contactCcdCalculations[types.second][types.first] = calculation;
}

}; // namespace Collision
}; // namespace SurgSim
