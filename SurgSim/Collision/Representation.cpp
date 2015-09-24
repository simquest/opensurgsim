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
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Shape.h"
#include "SurgSim/Physics/Representation.h"

namespace SurgSim
{
namespace Collision
{

Representation::Representation(const std::string& name) :
	SurgSim::Framework::Representation(name)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Representation, std::vector<std::string>, Ignoring, getIgnoring, setIgnoring);
}

Representation::~Representation()
{

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
	return m_ignoring.insert(fullName).second;
}

bool Representation::ignore(const std::shared_ptr<Representation>& representation)
{
	return ignore(representation->getFullName());
}

void Representation::setIgnoring(const std::vector<std::string>& ignoring)
{
	m_ignoring.clear();
	for (auto& fullName : ignoring)
	{
		ignore(fullName);
	}
}

std::vector<std::string> Representation::getIgnoring() const
{
	return std::vector<std::string>(std::begin(m_ignoring), std::end(m_ignoring));
}

bool Representation::isIgnoring(const std::string& fullName) const
{
	return m_ignoring.find(fullName) != m_ignoring.end();
}

bool Representation::isIgnoring(const std::shared_ptr<Representation>& representation) const
{
	return isIgnoring(representation->getFullName());
}

}; // namespace Collision
}; // namespace SurgSim
