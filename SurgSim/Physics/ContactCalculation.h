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

/// Base class responsible for calculating contact data between to given shapes, calculateContact needs to
/// determine whether the two shapes intersect, and if yes calculate the correct data for this contact, which
/// consists of, the normal to displace the first shape so that the two shapes just barely touch. And the
/// penetration point for each shape, the point the is furthest inside the other object.
/// If a subclass is works on asymmetric pairs it should implement a flag that enables the calculation to
/// switch the direction of the pair.
class ContactCalculation
{
public:

	/// Constructor
	explicit ContactCalculation(bool doSwapPairs = false) : m_doSwapPairs(doSwapPairs)
	{
	}


	/// Destructor
	virtual ~ContactCalculation()
	{
	}

	/// Calculate the actual contact between two shapes of the give CollisionPair.
	/// \param	pair	The pair that is under consideration.
	virtual void calculateContact(std::shared_ptr<CollisionPair> pair) = 0;

	bool needsSwap()
	{
		return m_doSwapPairs;
	}

private:

	bool m_doSwapPairs;

};

/// A default calculation, it does nothing and can be used as a placeholder
class DefaultContactCalculation : public ContactCalculation
{
public:

	/// Constructor
	/// \param doAssert If set the calculation will throw an exception if it is executed, this
	/// 				can be used to detect cases where a  contact calculation is being called
	/// 				on a pair that should be implemented
	explicit DefaultContactCalculation(bool doAssert = false) : m_doAssert(doAssert)
	{
	}

	/// Destructor
	virtual ~DefaultContactCalculation() {}


	/// \param	pair	The pair that is under consideration.
	virtual void calculateContact(std::shared_ptr<CollisionPair> pair) override;

private:
	bool m_doAssert;
};

/// Class to calculate intersections between spheres
class SphereSphereDcdContact : public ContactCalculation
{
public:
	explicit SphereSphereDcdContact()
	{
	}

	/// Calculate the actual contact between two shapes of the give CollisionPair.
	/// \param	pair	The pair that is under consideration.
	virtual void calculateContact(std::shared_ptr<CollisionPair> pair);
};


/// Class to calculate intersections between Spheres and DoubleSidedPlanes
class SphereDoubleSidedPlaneDcdContact : public ContactCalculation
{
public:

	/// Constructor.
	/// \param	swapPairs	Set to true if the calculation needs to switch the members of the pair.
	explicit SphereDoubleSidedPlaneDcdContact(bool swapPairs) : ContactCalculation(swapPairs)
	{

	}

	/// Calculate the actual contact between two shapes of the give CollisionPair.
	/// \param	pair	The pair that is under consideration.
	virtual void calculateContact(std::shared_ptr<CollisionPair> pair);

};

}; // Physics
}; // SurgSim

#endif
