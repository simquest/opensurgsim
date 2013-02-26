// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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

#ifndef SURGSIM_INPUT_DATA_GROUP_H
#define SURGSIM_INPUT_DATA_GROUP_H

#include <SurgSim/Framework/Assert.h>
#include <SurgSim/Input/NamedData.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Vector.h>

namespace SurgSim
{
namespace Input
{

/// A collection of value entries of different types that can be accessed by name or index.
///
/// A DataGroup object contains a collection of values of one of several predefined types:
/// \li \em Poses contain the position and orientation of an object in space, represented as a 3D rigid-body
/// 	 (isometric) transformation.
/// \li \em Vectors contain a vector quantity that does not change when the coordinate system is translated,
/// 	such as a force or an oriented distance.
/// \li \em Scalars contain a scalar value (i.e. anything that can be represented as a double).
/// \li \em Integers contain an integer value.
/// \li \em Booleans contain a Boolean logic value (true or false).
/// \li \em Strings contain a text value.
///
/// Each entry can be also be marked as not currently valid, i.e. missing.  Its entry still remains in the
/// collection, but for the moment has no value associated with it.
///
/// Each entry of any particular value type in the collection can be accessed by using either its unique name (a
/// std::string) or its unique index (a non-negative integer). Access by name is more convenient, but also less
/// efficient.  The names and indices are unique within each type, but not necessarily across different types
/// (i.e. there could be a scalar and a vector both named "friction", or a pose and a boolean both at index 1).
/// However, it is recommended that you keep names separate between different types as well, to avoid confusion.
///
/// A DataGroup object constructed by the default constructor starts out empty, meaning it has not yet been
/// associated with a set of names and indices.  A <i>non</i>-empty object contains a fixed set of value entries;
/// entries <b>cannot be added or removed</b>, and names and indices of existing entries
/// <b>cannot be changed</b>.  A non-empty object also cannot ever become empty again.  These properties ensure
/// that a stable data layout is available to the code using this class so that it can, for example, record
/// entry indices and use them to retrieve the same entries later on.  (Currently, no attempt is made to ensure
/// type consistency, so attempting to read a pose entry using an index of a string may not result in an error.)
///
/// The set of names and indices within a DataGroup object object cannot be modified, but it can be initialized
/// using the \ref DataGroupBuilder class.  After doing that, you can create other objects with the same layout
/// by copy construction, or by assigning the initialized value to an empty (default-constructed) DataGroup
/// object.
class DataGroup
{
public:
	/// The type used for poses.
	typedef SurgSim::Math::RigidTransform3d PoseType;
	/// The type used for vectors.
	typedef SurgSim::Math::Vector3d VectorType;
	/// The type used for scalars.
	typedef double ScalarType;
	/// The type used for integers.
	typedef int IntegerType;
	/// The type used for booleans.
	typedef bool BooleanType;
	/// The type used for strings.
	typedef std::string StringType;


	/// Construct an empty object, with no associated names and indices yet.
	DataGroup() {};

	/// Construct an object as a copy of the data from another object.
	/// \param dataGroup The object to copy from.
	DataGroup(const DataGroup& dataGroup)
		: m_poses(dataGroup.m_poses),
		  m_vectors(dataGroup.m_vectors),
		  m_scalars(dataGroup.m_scalars),
		  m_integers(dataGroup.m_integers),
		  m_booleans(dataGroup.m_booleans),
		  m_strings(dataGroup.m_strings)
	{
	}

	/// Copy the data from another object.
	///
	/// The object being assigned into must either be empty (not yet associated with a set of names and indices), or
	/// the two objects must share the same data layout, resulting from earlier copy construction or assignment.
	/// ~~~~~
	/// DataGroup initial;
	/// // ...initialize "initial" to some non-empty value...
	/// DataGroup copyConstructed(initial);  // Layout is shared with initial
	/// copyConstructed = initial            // OK, using the same layout
	/// DataGroup another;                   // Object is empty (no layout)
	/// another = initial;                   // OK, layout is now shared with initial
	/// another = initial                    // OK, using the same layout
	/// ~~~~~
	///
	/// Note that the data layout must be the same, i.e. related to one another by object assignment or copy
	/// construction.  Objects that merely contain entries with the same names and indices are not acceptable!
	/// (Otherwise, we'd need to inefficiently compare layout contents each time we assign.)
	/// ~~~~~
	/// DataGroupBuilder builder;
	/// // ...initialize the entries in the builder...
	/// NamedData first = builder.createData();   // Layout of entries created from builder
	/// NamedData second = builder.createData();  // Another layout of entries created; names and indices match
	/// second = first;                           // ERROR at run-time, layouts were created separately!
	/// ~~~~~
	///
	/// \param namedData The object to copy from.
	/// \return The object that was assigned into.
	DataGroup& operator=(const DataGroup& dataGroup)
	{
		SURGSIM_ASSERT(dataGroup.isValid()) <<
			"Can't use an invalid (empty) DataGroup on the right-hand side of an assignment!";

		m_poses = dataGroup.m_poses;
		m_vectors = dataGroup.m_vectors;
		m_scalars = dataGroup.m_scalars;
		m_integers = dataGroup.m_integers;
		m_booleans = dataGroup.m_booleans;
		m_strings = dataGroup.m_strings;

		SURGSIM_ASSERT(isValid()) << "DataGroup isn't valid after assignment!";
		return *this;
	}

	/// Move the data from another object.
	///
	/// The same restrictions on object compatibility apply as in the case of the copy assignment
	///	operator=(const DataGroup&).
	///
	/// \param [in,out] dataGroup The object to copy from, which will be left in an ununsable state.
	/// \return The object that was assigned into.
	DataGroup& operator=(DataGroup&& dataGroup)
	{
		SURGSIM_ASSERT(dataGroup.isValid()) <<
			"Can't use an invalid (empty) DataGroup on the right-hand side of an assignment!";

		m_poses = std::move(dataGroup.m_poses);
		m_vectors = std::move(dataGroup.m_vectors);
		m_scalars = std::move(dataGroup.m_scalars);
		m_integers = std::move(dataGroup.m_integers);
		m_booleans = std::move(dataGroup.m_booleans);
		m_strings = std::move(dataGroup.m_strings);

		SURGSIM_ASSERT(isValid()) << "DataGroup isn't valid after assignment!";
		return *this;
	}

	/// Check if the object is valid (non-empty), meaning it is associated with a set of names and indices.
	/// If the object is empty, it can become valid on assignment from a valid object.
	///
	/// \return true if valid, false if empty.
	bool isValid() const
	{
		bool valid = poses().isValid();
		SURGSIM_ASSERT(poses().isValid() == valid &&
		               vectors().isValid() == valid &&
		               scalars().isValid() == valid &&
		               integers().isValid() == valid &&
		               booleans().isValid() == valid &&
		               strings().isValid() == valid) << "The object is only partially initialized!";
		return valid;
	}

	/// Return the pose data structure.
	/// \return the mutable pose data.
	NamedData<PoseType>& poses()
	{
		return m_poses;
	}

	/// Return the pose data structure.
	/// \return the read-only pose data.
	const NamedData<PoseType>& poses() const
	{
		return m_poses;
	}

	/// Return the vector data structure.
	/// \return the mutable vector data.
	NamedData<VectorType>& vectors()
	{
		return m_vectors;
	}

	/// Return the vector data structure.
	/// \return the read-only vector data.
	const NamedData<VectorType>& vectors() const
	{
		return m_vectors;
	}

	/// Return the scalar data structure.
	/// \return the mutable scalar data.
	NamedData<ScalarType>& scalars()
	{
		return m_scalars;
	}

	/// Return the scalar data structure.
	/// \return the read-only scalar data.
	const NamedData<ScalarType>& scalars() const
	{
		return m_scalars;
	}

	/// Return the integer data structure.
	/// \return the mutable integer data.
	NamedData<IntegerType>& integers()
	{
		return m_integers;
	}

	/// Return the integer data structure.
	/// \return the read-only integer data.
	const NamedData<IntegerType>& integers() const
	{
		return m_integers;
	}

	/// Return the boolean data structure.
	/// \return the mutable Boolean data.
	NamedData<BooleanType>& booleans()
	{
		return m_booleans;
	}

	/// Return the boolean data structure.
	/// \return the read-only Boolean data.
	const NamedData<BooleanType>& booleans() const
	{
		return m_booleans;
	}

	/// Return the string data structure.
	/// \return the mutable string data.
	NamedData<StringType>& strings()
	{
		return m_strings;
	}

	/// Return the string data structure.
	/// \return the read-only string data.
	const NamedData<StringType>& strings() const
	{
		return m_strings;
	}

	/// Mark all data as not current.
	void resetAll()
	{
		m_poses.resetAll();
		m_vectors.resetAll();
		m_scalars.resetAll();
		m_integers.resetAll();
		m_booleans.resetAll();
		m_strings.resetAll();
	}

private:
	/// The pose values.
	NamedData<PoseType> m_poses;

	/// The vector values.
	NamedData<VectorType> m_vectors;

	/// The scalar values.
	NamedData<ScalarType> m_scalars;

	/// The integer values.
	NamedData<IntegerType> m_integers;

	/// The boolean values.
	NamedData<BooleanType> m_booleans;

	/// The string values.
	NamedData<StringType> m_strings;
};

};  // namespace Input
};  // namespace SurgSim

#endif  // SURGSIM_INPUT_DATA_GROUP_H
