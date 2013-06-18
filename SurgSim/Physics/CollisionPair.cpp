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

#include <numeric>
#include <SurgSim/Framework/Assert.h>
#include <SurgSim/Physics/CollisionPair.h>
#include <SurgSim/Physics/CollisionRepresentation.h>

namespace SurgSim
{
namespace Physics
{

CollisionPair::CollisionPair(const std::shared_ptr<CollisionRepresentation>& first, 
							 const std::shared_ptr<CollisionRepresentation>& second) :
		m_representations(first, second), m_isSwapped(false)
{
	SURGSIM_ASSERT(first != second) << "Should try to collide with self";
	SURGSIM_ASSERT(first != nullptr && second != nullptr) << "CollisionRepresentation cannot be null";
}

CollisionPair::~CollisionPair()
{

}

void CollisionPair::clearContacts()
{
	m_contacts.clear();
}

void CollisionPair::swapRepresentations()
{
	SURGSIM_ASSERT(! hasContacts()) << "Can't swap representations after contacts have already been calculated";
	m_isSwapped = !m_isSwapped;
	std::swap(m_representations.first, m_representations.second);
}

bool CollisionPair::isSwapped() const
{
	return m_isSwapped;
}

}; // namespace Physics
}; // namespace SurgSim

