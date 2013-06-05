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
	explicit ContactCalculation()
	{
	}
	virtual ~ContactCalculation()
	{
	}
	virtual void calculateContact(std::shared_ptr<CollisionPair> pair) = 0;
};

class DefaultContactCalculation : public ContactCalculation
{
public:
	explicit DefaultContactCalculation(bool doAssert = false) : m_doAssert(doAssert)
	{
	}
	virtual ~DefaultContactCalculation() {}

	virtual void calculateContact(std::shared_ptr<CollisionPair> pair) override;

private:
	bool m_doAssert;
};

class SphereSphereDcdContact : public ContactCalculation
{
public:
	explicit SphereSphereDcdContact()
	{
	}

	virtual void calculateContact(std::shared_ptr<CollisionPair> pair);
};

class SpherePlaneDcdContact : public ContactCalculation
{
public:
	explicit SpherePlaneDcdContact(bool switchPair) :
		m_switchPair(switchPair)
	{
	}

	virtual void calculateContact(std::shared_ptr<CollisionPair> pair);
private:
	bool m_switchPair;
};

}; // Physics
}; // SurgSim

#endif
