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

#ifndef SURGSIM_COLLISION_CONTACTCALCULATION_H
#define SURGSIM_COLLISION_CONTACTCALCULATION_H

#include <memory>
#include <mutex>

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Shape.h"

namespace SurgSim
{

namespace Math
{
class Shape;
}

namespace Collision
{

/// Base class responsible for calculating contact data between two objects.
/// It is used for determining whether two objects intersect. If there is
/// contact, new Contacts are calculated.
/// \sa Contact
class ContactCalculation
{
public:
	/// Constructor
	ContactCalculation();

	/// Destructor
	virtual ~ContactCalculation();

	/// Calculate the contacts for a given pair
	/// \param	pair	A CollisionPair that is under consideration, new contacts will be added to this pair
	void calculateContact(std::shared_ptr<CollisionPair> pair);

	/// Calculate the dcd contacts between two posed/transformed shapes
	/// \param posedShape1, posedShape2 The two posed shape to consider for this dcd contact calculation
	/// \return a list of contacts between the two given posed shapes
	std::list<std::shared_ptr<Contact>> calculateDcdContact(
		const Math::PosedShape<std::shared_ptr<Math::Shape>> posedShape1,
		const Math::PosedShape<std::shared_ptr<Math::Shape>> posedShape2);

	/// Calculate the ccd contacts between two posed/transformed shapes
	/// \param posedShapeMotion1, posedShapeMotion2 The two posed shape motion to calculate ccd contact for
	/// \return a list of ccd contacts between the two given shapes
	/// \note The contact information is related to the end of the time range, so solving these contact will
	/// \note solve the collision at the end of the time range.
	std::list<std::shared_ptr<Contact>> calculateCcdContact(
		const Math::PosedShapeMotion<std::shared_ptr<Math::Shape>> posedShapeMotion1,
		const Math::PosedShapeMotion<std::shared_ptr<Math::Shape>> posedShapeMotion2);

	/// Virtual function that returns the shapes that this ContactCalculation class handles.
	/// \return Return the shape types this class handles.
	virtual std::pair<int, int> getShapeTypes() = 0;

	/// Register an instance of a contact calculation in the table
	/// \param calculation The calculation to be registered
	static void registerDcdContactCalculation(const std::shared_ptr<ContactCalculation>& calculation);

	/// Register an instance of a contact calculation in the table
	/// \param calculation The calculation to be registered
	static void registerCcdContactCalculation(const std::shared_ptr<ContactCalculation>& calculation);

	typedef
	std::array<std::array<std::shared_ptr<ContactCalculation>, Math::SHAPE_TYPE_COUNT>, Math::SHAPE_TYPE_COUNT>
	TableType;

	static const TableType& getDcdContactTable();
	static const TableType& getCcdContactTable();
private:

	/// Calculate the actual contact between two shapes of the given CollisionPair.
	/// \param	pair	The symmetric pair that is under consideration.
	virtual void doCalculateContact(std::shared_ptr<CollisionPair> pair);

	/// Virtual function receives the call from the public interface, usually will type the
	/// shapes statically to their known types and then execute a specific contact calculation
	/// between the two shapes
	/// \param posedShape1, posedShape2 The two posed shapes to calculate dcd contact for
	/// \return a list of dcd contacts between the two given posed shapes
	virtual std::list<std::shared_ptr<Contact>> doCalculateDcdContact(
		const Math::PosedShape<std::shared_ptr<Math::Shape>>& posedShape1,
		const Math::PosedShape<std::shared_ptr<Math::Shape>>& posedShape2);

	/// Virtual function receives the call from the public interface, usually will type the
	/// shapes statically to their known types and then execute a specific contact calculation
	/// between the two shapes
	/// \param posedShapeMotion1, posedShapeMotion2 The two posed shapes motion to calculate ccd contact for
	/// \return a list of ccd contacts between the two given posed shapes motion
	virtual std::list<std::shared_ptr<Contact>> doCalculateCcdContact(
		const Math::PosedShapeMotion<std::shared_ptr<Math::Shape>>& posedShapeMotion1,
		const Math::PosedShapeMotion<std::shared_ptr<Math::Shape>>& posedShapeMotion2);

	/// Statically initialize the tables, used via call once
	static void initializeTables();

	///@{
	/// registration to call at static scope (does not protect the initialization via call_once)
	/// Mirroring the public functions
	static void privateDcdRegister(
		const std::shared_ptr<ContactCalculation>& calculation,
		const std::pair<int, int>& types);
	static void privateDcdRegister(const std::shared_ptr<ContactCalculation>& calculation);
	///@}

	///@{
	/// registration to call at static scope (does not protect the initialization via call_once)
	/// Mirroring the public functions
	static void privateCcdRegister(
		const std::shared_ptr<ContactCalculation>& calculation,
		const std::pair<int, int>& types);
	static void privateCcdRegister(const std::shared_ptr<ContactCalculation>& calculation);
	///@}

	static TableType m_contactDcdCalculations; ///< Static table of Dcd contact calculations
	static TableType m_contactCcdCalculations; ///< Static table of Ccd contact calculations
	static std::once_flag m_initializationFlag; ///< Flag used for initialization.
};

}; // namespace Collision
}; // namespace SurgSim

#endif
