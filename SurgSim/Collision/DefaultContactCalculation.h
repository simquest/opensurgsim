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

#ifndef SURGSIM_COLLISION_DEFAULTCONTACTCALCULATION_H
#define SURGSIM_COLLISION_DEFAULTCONTACTCALCULATION_H

#include <memory>

#include "SurgSim/Collision/ContactCalculation.h"

namespace SurgSim
{
namespace Collision
{

class CollisionPair;

/// A default calculation, it does nothing and can be used as a placeholder
class DefaultContactCalculation : public ContactCalculation
{
public:
	/// Constructor
	/// \param doAssert If set the calculation will throw an exception if it is executed, this
	/// 				can be used to detect cases where a  contact calculation is being called
	/// 				on a pair that should be implemented
	explicit DefaultContactCalculation(bool doAssert = false);

	/// Destructor
	virtual ~DefaultContactCalculation();

	std::pair<int, int> getShapeTypes() override;

private:
	bool m_doAssert;

	void doCalculateContact(std::shared_ptr<CollisionPair> pair) override;

	std::list<std::shared_ptr<Contact>> doCalculateDcdContact(const std::shared_ptr<Math::Shape>& shape1,
		const std::shared_ptr<Math::Shape>& shape2) override;

	std::list<std::shared_ptr<Contact>> doCalculateCcdContact(const Math::ShapeMotion& shapeMotion1,
		const Math::ShapeMotion& shapeMotion2) override;
};

}; // namespace Collision
}; // namespace SurgSim

#endif
