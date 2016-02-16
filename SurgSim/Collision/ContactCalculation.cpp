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

#include "SurgSim/Collision/CcdDcdCollision.h"
#include "SurgSim/Collision/ContactCalculation.h"
#include "SurgSim/Collision/DefaultContactCalculation.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Framework/Log.h"

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
	const Math::PosedShape<std::shared_ptr<Math::Shape>> posedShape1,
	const Math::PosedShape<std::shared_ptr<Math::Shape>> posedShape2)
{
	auto types = getShapeTypes();
	auto incoming = std::make_pair(posedShape1.getShape()->getType(), posedShape2.getShape()->getType());
	if (incoming == types)
	{
		return doCalculateDcdContact(posedShape1, posedShape2);
	}

	if (incoming.first == types.second && incoming.second == types.first)
	{
		auto contacts = doCalculateDcdContact(posedShape2, posedShape1);
		for (const auto& contact : contacts)
		{
			contact->normal = -contact->normal;
			contact->force = -contact->force;
			std::swap(contact->penetrationPoints.first, contact->penetrationPoints.second);
		}
		return contacts;
	}

	if (types.first != Math::SHAPE_TYPE_NONE && types.second != Math::SHAPE_TYPE_NONE)
	{
		SURGSIM_FAILURE() << "Incorrect shape type for this calculation expected "
						  << types.first << ", " << types.second
						  << " received " << incoming.first << ", " << incoming.second << ".";
	}

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

	std::list<std::shared_ptr<Contact>> contacts;
	if (pair->getType() == Collision::CollisionDetectionType::COLLISION_DETECTION_TYPE_DISCRETE)
	{
		Math::PosedShape<std::shared_ptr<Math::Shape>> posedShape1(pair->getFirst()->getShape(),
			pair->getFirst()->getPose());
		Math::PosedShape<std::shared_ptr<Math::Shape>> posedShape2(pair->getSecond()->getShape(),
			pair->getSecond()->getPose());
		contacts = doCalculateDcdContact(posedShape1, posedShape2);
	}
	else if (pair->getType() == Collision::CollisionDetectionType::COLLISION_DETECTION_TYPE_CONTINUOUS)
	{
		contacts = doCalculateCcdContact(
			pair->getFirst()->getPosedShapeMotion(),
			pair->getSecond()->getPosedShapeMotion());
	}
	else
	{
		SURGSIM_FAILURE() << "Invalid collision detection type, neither discrete nor continuous";
	}

	for (auto& contact : contacts)
	{
		pair->addContact(contact);
	}
}

std::list<std::shared_ptr<Contact>> ContactCalculation::doCalculateDcdContact(
	const Math::PosedShape<std::shared_ptr<Math::Shape>>& posedShape1,
	const Math::PosedShape<std::shared_ptr<Math::Shape>>& posedShape2)
{
	SURGSIM_FAILURE() << "Not implemented";
	return std::list<std::shared_ptr<Contact>>();
}

std::list<std::shared_ptr<Contact>> ContactCalculation::doCalculateCcdContact(
	const Math::PosedShapeMotion<std::shared_ptr<Math::Shape>>& posedShapeMotion1,
	const Math::PosedShapeMotion<std::shared_ptr<Math::Shape>>& posedShapeMotion2)
{
	SURGSIM_FAILURE() << "Not implemented";
	return std::list<std::shared_ptr<Contact>>();
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
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::BoxCapsuleContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::BoxDoubleSidedPlaneContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::BoxPlaneContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::BoxSphereContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::CapsuleSphereContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::OctreeCapsuleContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::OctreeDoubleSidedPlaneContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::OctreePlaneContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::OctreeSphereContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::SegmentMeshTriangleMeshContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::SphereSphereContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::SphereDoubleSidedPlaneContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::SpherePlaneContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::TriangleMeshParticlesContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::TriangleMeshPlaneContact>());
	ContactCalculation::privateDcdRegister(std::make_shared<Collision::TriangleMeshTriangleMeshContact>());

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
		ContactCalculation::privateDcdRegister(std::make_shared<Collision::CompoundShapeContact>(
											   std::make_pair(Math::SHAPE_TYPE_COMPOUNDSHAPE, type)));
	}

	ContactCalculation::privateCcdRegister(std::make_shared<Collision::SegmentSelfContact>());
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
