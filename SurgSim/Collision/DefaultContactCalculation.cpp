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

#include "SurgSim/Collision/DefaultContactCalculation.h"

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/Shape.h"

namespace SurgSim
{
namespace Collision
{

DefaultContactCalculation::DefaultContactCalculation(bool doAssert) :
	m_doAssert(doAssert)
{
}

DefaultContactCalculation::~DefaultContactCalculation()
{
}

std::pair<int, int> DefaultContactCalculation::getShapeTypes()
{
	return std::pair<int, int>(SurgSim::Math::SHAPE_TYPE_NONE, SurgSim::Math::SHAPE_TYPE_NONE);
}

void DefaultContactCalculation::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
	SURGSIM_ASSERT(!m_doAssert)
			<< "Contact calculation not implemented for pairs with types ("
			<< pair->getFirst()->getShapeType() << ", " << pair->getSecond()->getShapeType() << ").";
	SURGSIM_LOG_ONCE(SurgSim::Framework::Logger::getDefaultLogger(), WARNING)
			<< "Contact calculation not implemented for pairs with types ("
			<< pair->getFirst()->getShapeType() << ", " << pair->getSecond()->getShapeType() << ").";
}

std::list<std::shared_ptr<Contact>> DefaultContactCalculation::doCalculateDcdContact(
	const std::shared_ptr<Math::Shape>& shape1, const Math::RigidTransform3d& pose1,
	const std::shared_ptr<Math::Shape>& shape2, const Math::RigidTransform3d& pose2)
{
	SURGSIM_ASSERT(!m_doAssert)
			<< "Contact calculation not implemented for pairs with types ("
			<< shape1->getType() << ", " << shape2->getType() << ").";
	SURGSIM_LOG_ONCE(SurgSim::Framework::Logger::getDefaultLogger(), WARNING)
			<< "Contact calculation not implemented for pairs with types ("
			<< shape1->getType() << ", " << shape2->getType() << ").";
	return std::list<std::shared_ptr<Contact>>();

}

std::list<std::shared_ptr<Contact>> DefaultContactCalculation::doCalculateCcdContact(
	const std::shared_ptr<Math::Shape>& shape1AtTime0, const Math::RigidTransform3d& pose1AtTime0,
	const std::shared_ptr<Math::Shape>& shape1AtTime1, const Math::RigidTransform3d& pose1AtTime1,
	const std::shared_ptr<Math::Shape>& shape2AtTime0, const Math::RigidTransform3d& pose2AtTime0,
	const std::shared_ptr<Math::Shape>& shape2AtTime1, const Math::RigidTransform3d& pose2AtTime1)
{
	SURGSIM_ASSERT(!m_doAssert)
		<< "Contact calculation not implemented for pairs with types ("
		<< shape1AtTime0->getType() << ", " << shape2AtTime0->getType() << ").";
	SURGSIM_LOG_ONCE(SurgSim::Framework::Logger::getDefaultLogger(), WARNING)
		<< "Contact calculation not implemented for pairs with types ("
		<< shape1AtTime0->getType() << ", " << shape2AtTime0->getType() << ").";
	return std::list<std::shared_ptr<Contact>>();
}

}; // namespace Collision
}; // namespace SurgSim
