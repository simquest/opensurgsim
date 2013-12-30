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

#ifndef SURGSIM_DATASTRUCTURES_DATAGROUP_H
#define SURGSIM_DATASTRUCTURES_DATAGROUP_H

#include <Eigen/Core>

#include "SurgSim/DataStructures/NamedData.h"
#include "SurgSim/DataStructures/NamedVariantData.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace DataStructures
{

/// A collection of \ref NamedData objects.
///
/// A DataGroup object contains a NamedData for each of several predefined types:
/// \li \em Poses contain the position and orientation of an object in space, represented as a 3D rigid-body
/// 	 (isometric) transformation.
/// \li \em Vectors contain a vector quantity that does not change when the coordinate system is translated,
/// 	such as a force or an oriented distance.
/// \li \em Matrices contain a matrix.
/// \li \em Scalars contain a scalar value (i.e. anything that can be represented as a double).
/// \li \em Integers contain an integer value.
/// \li \em Booleans contain a Boolean logic value (true or false).
/// \li \em Strings contain a text value.
/// \li \em CustomData contain a custom data structure, \ref NamedVariantData.
///
/// The entries (names and indices) are unique within each NamedData, but not necessarily across different types
/// (i.e. there could be a scalar and a vector both named "friction", or a pose and a boolean both at index 1).
/// It is recommended that you keep names separate between different types to avoid confusion.
///
/// A DataGroup object constructed by the default constructor starts out uninitialized, meaning its NamedData member
/// objects have not been initialized. A DataGroup object can be initialized by initializing each and every of its
/// NamedData, or by using the \ref DataGroupBuilder class.  Given an initialized DataGroup object, you can create other
/// initialized objects with the same entries (in the NamedData) by copy construction, or by assigning the initialized
/// object to an uninitialized (default-constructed) object.
///
/// Assignment to an initialized DataGroup object is only possible if the two objects use the same pointers to the
/// directories in their respective NamedData, which happens when either of the two objects was copy-constructed from
/// the other.
/// 
/// Once a DataGroup is initialized, the "entries" (i.e., the strings and indices that are used to access the data)
/// cannot be changed, added to, removed from, or made empty.  These properties ensure that a stable data layout is
/// available to the code using this class.  For example, the calling code can cache the entries' indices and from then
/// on use the faster index-based lookup instead of the slower string-based lookup.
///
/// \sa SurgSim::DataStructures::NamedData, SurgSim::DataStructers::DataGroupBuilder
class DataGroup
{
public:
	/// The type used for poses.
	typedef SurgSim::Math::RigidTransform3d PoseType;
	/// The type used for vectors.
	typedef SurgSim::Math::Vector3d VectorType;
	/// The type used for matrices.
	typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign | Eigen::RowMajor> DynamicMatrixType;
	/// The type used for scalars.
	typedef double ScalarType;
	/// The type used for integers.
	typedef int IntegerType;
	/// The type used for booleans.
	typedef bool BooleanType;
	/// The type used for strings.
	typedef std::string StringType;

	/// Construct an empty object, with no associated names and indices yet.
	inline DataGroup();

	/// Construct an object as a copy of the data from another object.
	/// \param dataGroup The object to copy from.
	inline DataGroup(const DataGroup& dataGroup);

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
	/// \param dataGroup The object to copy from.
	/// \return The object that was assigned into.
	inline DataGroup& operator=(const DataGroup& dataGroup);

	/// Move the data from another object.
	///
	/// The same restrictions on object compatibility apply as in the case of the copy assignment
	///	operator=(const DataGroup&).
	///
	/// \param [in,out] dataGroup The object to copy from, which will be left in an unusable state.
	/// \return The object that was assigned into.
	inline DataGroup& operator=(DataGroup&& dataGroup);

	/// Check if the object has been initialized, which means each NamedData has been initialized (i.e., has a set of
	/// entries). If the object has not been initialized, it can become initialized on assignment from an initialized
	/// object.  Asserts on all of the NamedData having the same initialization state.
	///
	/// \return true if initialized.
	inline bool isInitialized() const;

	/// Return the pose data structure.
	/// \return the mutable pose data.
	inline NamedData<PoseType>& poses();

	/// Return the pose data structure.
	/// \return the read-only pose data.
	inline const NamedData<PoseType>& poses() const;

	/// Return the vector data structure.
	/// \return the mutable vector data.
	inline NamedData<VectorType>& vectors();

	/// Return the vector data structure.
	/// \return the read-only vector data.
	inline const NamedData<VectorType>& vectors() const;

	/// Return the matrix data structure.
	/// \return the mutable matrix data.
	inline NamedData<DynamicMatrixType>& matrices();

	/// Return the matrix data structure.
	/// \return the read-only matrix data.
	inline const NamedData<DynamicMatrixType>& matrices() const;

	/// Return the scalar data structure.
	/// \return the mutable scalar data.
	inline NamedData<ScalarType>& scalars();

	/// Return the scalar data structure.
	/// \return the read-only scalar data.
	inline const NamedData<ScalarType>& scalars() const;

	/// Return the integer data structure.
	/// \return the mutable integer data.
	inline NamedData<IntegerType>& integers();

	/// Return the integer data structure.
	/// \return the read-only integer data.
	inline const NamedData<IntegerType>& integers() const;

	/// Return the boolean data structure.
	/// \return the mutable Boolean data.
	inline NamedData<BooleanType>& booleans();

	/// Return the boolean data structure.
	/// \return the read-only Boolean data.
	inline const NamedData<BooleanType>& booleans() const;

	/// Return the string data structure.
	/// \return the mutable string data.
	inline NamedData<StringType>& strings();

	/// Return the string data structure.
	/// \return the read-only string data.
	inline const NamedData<StringType>& strings() const;

	/// Return the custom data structure.
	/// \return the mutable data.
	inline NamedVariantData& customData();

	/// Return the custom data structure.
	/// \return the read-only data.
	inline const NamedVariantData& customData() const;

	/// Mark all data as not current.
	inline void resetAll();

private:
	/// The pose values.
	NamedData<PoseType> m_poses;

	/// The vector values.
	NamedData<VectorType> m_vectors;

	/// The matrix values.
	NamedData<DynamicMatrixType> m_matrices;

	/// The scalar values.
	NamedData<ScalarType> m_scalars;

	/// The integer values.
	NamedData<IntegerType> m_integers;

	/// The boolean values.
	NamedData<BooleanType> m_booleans;

	/// The string values.
	NamedData<StringType> m_strings;

	/// The custom data values.
	NamedVariantData m_customData;
};

};  // namespace Input
};  // namespace SurgSim


#include "SurgSim/DataStructures/DataGroup-inl.h"


#endif  // SURGSIM_DATASTRUCTURES_DATAGROUP_H
