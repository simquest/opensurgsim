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
}

Representation::~Representation()
{

}

const std::shared_ptr<SurgSim::Math::Shape> Representation::getPosedShape()
{
	boost::lock_guard<boost::mutex> lock(m_mutexPosedShape);

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
	boost::lock_guard<boost::mutex> lock(m_mutexPosedShape);

	m_posedShape = nullptr;
}

SurgSim::DataStructures::BufferedValue<ContactMapType>& Representation::getCollisions()
{
	return m_collisions;
}

void Representation::addContactWith(const std::shared_ptr<Representation>& other, std::shared_ptr<SurgSim::Collision::Contact> contact)
{
	boost::lock_guard<boost::mutex> lock(m_mutexCollisions);

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

}; // namespace Collision
}; // namespace SurgSim
