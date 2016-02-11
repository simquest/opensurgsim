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
	m_logger(Framework::Logger::getLogger("Collision/Representation")),
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

const Math::PosedShapeMotion<std::shared_ptr<Math::Shape>>& Representation::getPosedShapeMotion() const
{
	boost::shared_lock<boost::shared_mutex> lock(m_posedShapeMotionMutex);

	return m_posedShapeMotion;
}

void Representation::setPosedShapeMotion(const Math::PosedShapeMotion<std::shared_ptr<Math::Shape>>& posedShapeMotion)
{
	boost::unique_lock<boost::shared_mutex> lock(m_posedShapeMotionMutex);

	m_posedShapeMotion = posedShapeMotion;
}

const std::shared_ptr<Math::Shape> Representation::getPosedShape()
{
	boost::unique_lock<boost::shared_mutex> lock(m_posedShapeMotionMutex);

	Math::RigidTransform3d identity = Math::RigidTransform3d::Identity();
	Math::RigidTransform3d pose = getPose();
	if (pose.isApprox(identity))
	{
		Math::PosedShape<std::shared_ptr<Math::Shape>> newPosedShape(getShape(), identity);
		m_posedShapeMotion.second = newPosedShape;
	}
	else if (m_posedShapeMotion.second.getShape() == nullptr || !pose.isApprox(m_posedShapeMotion.second.getPose()))
	{
		Math::PosedShape<std::shared_ptr<Math::Shape>> newPosedShape(getShape()->getTransformed(pose), pose);
		m_posedShapeMotion.second = newPosedShape;
	}

	return m_posedShapeMotion.second.getShape();
}

void Representation::invalidatePosedShapeMotion()
{
	boost::unique_lock<boost::shared_mutex> lock(m_posedShapeMotionMutex);

	m_posedShapeMotion.invalidate();
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

	bool result = false;
	if (m_allowing.empty())
	{
		result = m_ignoring.insert(fullName).second;
	}
	else
	{
		auto found = m_allowing.find(fullName);
		if (found != m_allowing.end())
		{
			m_allowing.erase(found);
			result = true;
		}
		else
		{
			SURGSIM_LOG_WARNING(m_logger)
					<< getFullName() << " Trying to un-allow" << fullName << " but it wasn't found.";
		}
	}

	return result;
}

bool Representation::ignore(const std::shared_ptr<Representation>& representation)
{
	if (representation->getSceneElement() == nullptr)
	{
		SURGSIM_LOG_WARNING(m_logger) << getFullName() << " cannot ignore " << representation->getName() <<
									  ", which is not in a scene element.";
		return false;
	}
	return ignore(representation->getFullName());
}

bool Representation::allow(const std::string& fullName)
{
	bool result = false;
	if (m_ignoring.empty())
	{
		result = m_allowing.insert(fullName).second;
	}
	else
	{
		auto found = m_ignoring.find(fullName);
		if (found != m_ignoring.end())
		{
			m_ignoring.erase(found);
			result = true;
		}
		else
		{
			SURGSIM_LOG_WARNING(m_logger)
					<< getFullName() << " Trying un-ignore" << fullName << " but it wasn't found.";
		}
	}
	return result;
}

bool Representation::allow(const std::shared_ptr<Representation>& representation)
{
	if (representation->getSceneElement() == nullptr)
	{
		SURGSIM_LOG_WARNING(m_logger) << getFullName() << " cannot allow " << representation->getName() <<
									  ", which is not in a scene element.";
		return false;
	}
	return allow(representation->getFullName());
}

void Representation::setIgnoring(const std::vector<std::string>& fullNames)
{
	if (m_allowing.empty())
	{
		m_ignoring.clear();
		std::copy(fullNames.cbegin(), fullNames.cend(), std::inserter(m_ignoring, m_ignoring.begin()));
	}
	else
	{
		SURGSIM_LOG_SEVERE(m_logger) << getFullName() << " cannot use setIgnoring. "
									 << "You can only set what representations to ignore or allow, not both.";
	}
}

std::vector<std::string> Representation::getIgnoring() const
{
	return std::vector<std::string>(std::begin(m_ignoring), std::end(m_ignoring));
}

bool Representation::isIgnoring(const std::string& fullName) const
{
	if (m_allowing.empty())
	{
		return m_ignoring.find(fullName) != m_ignoring.end();
	}
	return m_allowing.find(fullName) == m_allowing.end();
}

bool Representation::isIgnoring(const std::shared_ptr<Representation>& representation) const
{
	return isIgnoring(representation->getFullName());
}

bool Representation::isAllowing(const std::string& fullName) const
{
	return !isIgnoring(fullName);
}

bool Representation::isAllowing(const std::shared_ptr<Representation>& representation) const
{
	return isAllowing(representation->getFullName());
}


void Representation::setAllowing(const std::vector<std::string>& fullNames)
{
	if (m_ignoring.empty())
	{
		m_allowing.clear();
		std::copy(fullNames.cbegin(), fullNames.cend(), std::inserter(m_allowing, m_allowing.begin()));
	}
	else
	{
		SURGSIM_LOG_SEVERE(m_logger) << getFullName() << " cannot use setAllowing. "
									 << "You can only set what representations to ignore or allow, not both.";
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

Math::Aabbd Representation::getBoundingBox() const
{
	SURGSIM_ASSERT(getShape() != nullptr);
	return Math::transformAabb(getPose(), getShape()->getBoundingBox());
}

}; // namespace Collision
}; // namespace SurgSim
