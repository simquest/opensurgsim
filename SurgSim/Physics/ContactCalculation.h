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

#ifndef SURGSIM_PHYSICS_CONTACTCALCULATION_H
#define SURGSIM_PHYSICS_CONTACTCALCULATION_H

#include <memory>

#include <SurgSim/Framework/ReuseFactory.h>
#include <SurgSim/Physics/CollisionPair.h>

namespace SurgSim
{
namespace Physics
{

typedef SurgSim::Framework::ReuseFactory<Contact> ContactFactory;

class ContactCalculation
{
public:
	explicit ContactCalculation(std::shared_ptr<ContactFactory> factory) : m_contactFactory(factory) {}
	virtual ~ContactCalculation() {}
	virtual void calculateContact(std::shared_ptr<CollisionPair> pair) = 0;

protected:

	/// Helper function to add a contact to a given CollisionPair.
	/// \param	pair  	The pair.
	/// \param	depth 	The depth.
	/// \param	normal	The normal.
	/// \param	contact The contact point, not used  for DCD.
	inline void addContact(std::shared_ptr<CollisionPair> pair, 
							const double& depth, 
							const Vector3d& normal, 
							const Vector3d& contactPoint = Vector3d(0,0,0)) 
	{
		std::shared_ptr<Contact> contact = m_contactFactory->getInstance();
		contact->depth = depth;
		contact->normal = normal;
		contact->contact = contactPoint;
		pair->addContact(contact);
	}

	std::shared_ptr<SurgSim::Framework::ReuseFactory<Contact>> m_contactFactory;
	
};

class DefaultContactCalculation : public ContactCalculation
{
public:
	explicit DefaultContactCalculation(bool doAssert = false) : m_doAssert(doAssert), ContactCalculation(nullptr) {}
	virtual ~DefaultContactCalculation() {}

	virtual void calculateContact(std::shared_ptr<CollisionPair> pair) override;

private:
	bool m_doAssert;
};

class SphereSphereDcdContact : public ContactCalculation
{
public:
	explicit SphereSphereDcdContact(std::shared_ptr<ContactFactory> factory) : ContactCalculation(factory) {};
	virtual void calculateContact(std::shared_ptr<CollisionPair> pair);
};

class SpherePlaneDcdContact : public ContactCalculation
{
public:
	explicit SpherePlaneDcdContact(std::shared_ptr<ContactFactory> factory) : ContactCalculation(factory) {};
	virtual void calculateContact(std::shared_ptr<CollisionPair> pair);
};

}; // Physics
}; // SurgSim

#endif