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

#ifndef SURGSIM_COLLISION_COMPOUNDSHAPECONTACT_H
#define SURGSIM_COLLISION_COMPOUNDSHAPECONTACT_H

#include "SurgSim/Collision/ContactCalculation.h"

namespace SurgSim
{

namespace Collision
{

class CompoundShapeContact : public ContactCalculation
{
public:
	/// Constructor
	explicit CompoundShapeContact(const std::pair<int, int>& types);

protected:
	std::pair<int, int> getShapeTypes() override;

	std::list<std::shared_ptr<Contact>> doCalculateDcdContact(const std::shared_ptr<Math::Shape>& shape1,
		const std::shared_ptr<Math::Shape>& shape2) override;

	/// Local shape types for this instance, these can be set to match the expected shapes
	std::pair<int, int> m_types;

};

}
}

#endif
