// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Shape.h"
#include "SurgSim/Physics/Representation.h"


namespace SurgSim
{
namespace Collision
{

Representation::Representation(const std::string& name) :
	SurgSim::Framework::Representation(name),
	m_collisionDetectionType(COLLISION_DETECTION_TYPE_DISCRETE),
	m_selfCollisionDetectionType(COLLISION_DETECTION_TYPE_NONE)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Representation, std::vector<std::string>, Ignore, getIgnoring, setIgnoring);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Representation, std::vector<std::string>, Allow, getAllowing, setAllowing);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Representation, CollisionDetectionType, CollisionDetectionType,
			getCollisionDetectionType, setCollisionDetectionType);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Representation, CollisionDetectionType, SelfCollisionDetectionType,
			getSelfCollisionDetectionType, setSelfCollisionDetectionType);
}

Representation::~Representation()
{

}

void Representation::setCollisionDetectionType(CollisionDetectionType type)
{
	m_collisionDetectionType = type;
}

CollisionDetectionType Representation::getCollisionDetectionType() const
{
	return m_collisionDetectionType;
}

void Representation::setSelfCollisionDetectionType(CollisionDetectionType type)
{
	m_selfCollisionDetectionType = type;
}

CollisionDetectionType Representation::getSelfCollisionDetectionType() const
{
	return m_selfCollisionDetectionType;
}

const std::shared_ptr<SurgSim::Math::Shape> Representation::getPosedShape()
{
	boost::lock_guard<boost::mutex> lock(m_posedShapeMutex);

	Math::RigidTransform3d pose = getPose();
	if (pose.isApprox(Math::RigidTransform3d::Identity()))
	{
		m_posedShape = getShape();
		m_posedShapePose = Math::RigidTransform3d::Identity();
	}
	else if (m_posedShape == nullptr || !pose.isApprox(m_posedShapePose))
	{
		m_posedShape = getShape()->getTransformed(pose);
		m_posedShapePose = pose;
	}

	return m_posedShape;
}

void Representation::invalidatePosedShape()
{
	boost::lock_guard<boost::mutex> lock(m_posedShapeMutex);

	m_posedShape = nullptr;
}

SurgSim::DataStructures::BufferedValue<ContactMapType>& Representation::getCollisions()
{
	return m_collisions;
}

void Representation::addContact(const std::shared_ptr<Representation>& other,
								const std::shared_ptr<SurgSim::Collision::Contact>& contact)
{
	boost::lock_guard<boost::mutex> lock(m_collisionsMutex);

	m_collisions.unsafeGet()[other].push_back(contact);
}

bool Representation::collidedWith(const std::shared_ptr<Representation>& other)
{
	auto collisions = m_collisions.safeGet();
	return (collisions->find(other) != collisions->end());
}

void Representation::update(const double& dt)
{
}

bool Representation::ignore(const std::string& fullName)
{
	if (!m_allowing.empty())
	{
		SURGSIM_LOG_SEVERE(Framework::Logger::getDefaultLogger())
			<< "Collision Representation named " << getName() << " can not ignore " << fullName
			<< ". You can only set what representations to ignore or allow, not both.";
		return false;
	}
	else
	{
		return m_ignoring.insert(fullName).second;
	}
}

bool Representation::ignore(const std::shared_ptr<Representation>& representation)
{
	std::string fullName = representation->getFullName();
	SURGSIM_LOG_IF(representation->getSceneElement() == nullptr, Framework::Logger::getDefaultLogger(), WARNING)
		<< "Ignoring " << fullName << " may not work. It is not in a scene element yet, so its full name is unknown.";
	return ignore(fullName);
}

void Representation::setIgnoring(const std::vector<std::string>& fullNames)
{
	if (!m_allowing.empty())
	{
		SURGSIM_LOG_SEVERE(Framework::Logger::getDefaultLogger())
			<< "Collision Representation named " << getName() << " can not ignore other representations. "
			<< "You can only set what representations to ignore or allow, not both.";
	}
	else
	{
		m_ignoring.clear();
		for (auto& fullName : fullNames)
		{
			ignore(fullName);
		}
	}
}

std::vector<std::string> Representation::getIgnoring() const
{
	return std::vector<std::string>(std::begin(m_ignoring), std::end(m_ignoring));
}

bool Representation::isIgnoring(const std::string& fullName) const
{
	if (!m_allowing.empty())
	{
		return m_allowing.find(fullName) == m_allowing.end();
	}
	else
	{
		return m_ignoring.find(fullName) != m_ignoring.end();
	}
}

bool Representation::isIgnoring(const std::shared_ptr<Representation>& representation) const
{
	return isIgnoring(representation->getFullName());
}

void Representation::setAllowing(const std::vector<std::string>& fullNames)
{
	if (!m_ignoring.empty())
	{
		SURGSIM_LOG_SEVERE(Framework::Logger::getDefaultLogger())
			<< "Collision Representation named " << getName() << " cannot use setAllowing. "
			<< "You can only set what representations to ignore or allow, not both.";
	}
	else
	{
		m_allowing.clear();
		for (auto& fullName : fullNames)
		{
			m_allowing.insert(fullName);
		}
	}
}

std::vector<std::string> Representation::getAllowing() const
{
	return std::vector<std::string>(std::begin(m_allowing), std::end(m_allowing));
}

void Representation::doRetire()
{
	m_collisions.unsafeGet().clear();
	m_collisions.publish();
	Framework::Representation::doRetire();
}

}; // namespace Collision
}; // namespace SurgSim
