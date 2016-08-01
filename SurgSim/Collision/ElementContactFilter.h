// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#ifndef SURGSIM_COLLISION_ELEMENTCONTACTFILTER_H
#define SURGSIM_COLLISION_ELEMENTCONTACTFILTER_H

#include "SurgSim/Collision/ContactFilter.h"
#include "SurgSim/DataStructures/Location.h"

namespace SurgSim
{

namespace Physics
{
class PhysicsManagerState;
}

namespace Collision
{
class CollisionPair;
class Representation;
struct Contact;

SURGSIM_STATIC_REGISTRATION(ElementContactFilter);

class ElementContactFilter : public ContactFilter
{
public:
	ElementContactFilter(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Collision::ElementContactFilter);

	virtual bool doInitialize() override;

	virtual bool doWakeUp() override;

	void addFilterElements(const std::shared_ptr<Collision::Representation>& rep, std::vector<size_t> indices);

	void setFilterElements(const std::shared_ptr<Collision::Representation>& rep, std::vector<size_t> indices);

	std::vector<size_t> getFilterElements(const std::shared_ptr<Collision::Representation>& rep) const;

	void clearElements(const std::shared_ptr<Collision::Representation>& rep);

protected:
	virtual void doFilterContacts(
		const std::shared_ptr<Physics::PhysicsManagerState>& state,
		const std::shared_ptr<CollisionPair>& pair) override;

private:

	std::unordered_map<Collision::Representation*, std::vector<size_t>> m_data;

	void excecuteFilter(const std::shared_ptr<CollisionPair>& pair, size_t pairIndex, const std::vector<size_t>& filter);
};

template <class T>
const T& pairAt(const std::pair<T, T>& p, size_t i)
{
	SURGSIM_ASSERT(i == 0 || i == 1) << "Index for pair must be 0 or 1.";
	return (i == 0) ? p.first : p.second;
};

template <class T>
T& pairAt(std::pair<T, T>& p, size_t i)
{
	SURGSIM_ASSERT(i == 0 || i == 1) << "Index for pair must be 0 or 1.";
	return (i == 0) ? p.first : p.second;
};

}
}

#endif
