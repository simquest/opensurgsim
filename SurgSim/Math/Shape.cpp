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

#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Math/Shape.h"

namespace SurgSim
{

namespace Math
{

Shape::Shape()
{
	m_aabb.setEmpty();
}

Shape::~Shape()
{
}

bool Shape::isTransformable() const
{
	return false;
}

std::shared_ptr<Shape> Shape::getTransformed(const RigidTransform3d& pose) const
{
	SURGSIM_FAILURE() << "getTransformed not implemented for " << getClassName();
	return nullptr;
}

std::string Shape::getClassName() const
{
	SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger()) <<
			"getClassName() called on Math::Shape base class, this is wrong" <<
			" in almost all cases, this means there is a class that does not have getClassName() defined.";
	return "SurgSim::Math::Shape";
}

const Math::Aabbd& Shape::getBoundingBox() const
{
	return m_aabb;
}

} // namespace Math
} // namespace SurgSim
