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

#include "SurgSim/DataStructures/Image.h"
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
/// \li \em Booleans contain a Boolean logic value (true or false).
/// \li \em Images contain float images.
/// \li \em Integers contain an integer value.
/// \li \em Poses contain the position and orientation of an object in space, represented as a 3D rigid-body
/// 	 (isometric) transformation.
/// \li \em Matrices contain a matrix.
/// \li \em Scalars contain a scalar value (i.e. anything that can be represented as a double).
/// \li \em Strings contain a text value.
/// \li \em Vectors contain a vector quantity that does not change when the coordinate system is translated,
/// 	such as a force or an oriented distance.
/// \li \em CustomData contain a custom data structure, \ref NamedVariantData.
///
/// The entries (names and indices) are unique within each NamedData member, but not necessarily across different types
/// (i.e. there could be a scalar and a vector both named "friction", or a pose and a boolean both at index 1).
/// It is recommended that you keep names separate between different types to avoid confusion.
///
/// A DataGroup object constructed by the default constructor starts out empty, meaning all its NamedData member
/// objects are "invalid". An empty DataGroup object can be made non-empty by:
/// \li using the \ref DataGroupBuilder class,
/// \li copy construction,
/// \li assigning from a non-empty DataGroup object, or
/// \li assigning a "valid" NamedData object (of the correct template type) to one or more NamedData members.
///
/// Assignment to a non-empty DataGroup object is only possible if either of the two objects in the assignment was made
/// non-empty based on the other object (see the above list items about copy construction and assignment from a
/// non-empty DataGroup object).
///
/// Once a DataGroup is non-empty, the "entries" (i.e., the strings and indices that are used to access the data)
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
	typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> DynamicMatrixType;
	/// The type used for scalars.
	typedef double ScalarType;
	/// The type used for integers.
	typedef int IntegerType;
	/// The type used for booleans.
	typedef bool BooleanType;
	/// The type used for strings.
	typedef std::string StringType;
	/// The type used for images.
	typedef Image<float> ImageType;

	/// Construct an empty object, with no associated names and indices yet.
	DataGroup();

	/// Construct an object as a copy of the data from another object.
	/// \param dataGroup The object to copy from.
	DataGroup(const DataGroup& dataGroup);

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
	DataGroup& operator=(const DataGroup& dataGroup);

	/// Move the data from another object.
	///
	/// The same restrictions on object compatibility apply as in the case of the copy assignment
	///	operator=(const DataGroup&).
	///
	/// \param [in,out] dataGroup The object to copy from, which will be left in an unusable state.
	/// \return The object that was assigned into.
	DataGroup& operator=(DataGroup&& dataGroup);

	/// Return the pose data structure.
	/// \return the mutable pose data.
	NamedData<PoseType>& poses();

	/// Return the pose data structure.
	/// \return the read-only pose data.
	const NamedData<PoseType>& poses() const;

	/// Return the vector data structure.
	/// \return the mutable vector data.
	NamedData<VectorType>& vectors();

	/// Return the vector data structure.
	/// \return the read-only vector data.
	const NamedData<VectorType>& vectors() const;

	/// Return the matrix data structure.
	/// \return the mutable matrix data.
	NamedData<DynamicMatrixType>& matrices();

	/// Return the matrix data structure.
	/// \return the read-only matrix data.
	const NamedData<DynamicMatrixType>& matrices() const;

	/// Return the scalar data structure.
	/// \return the mutable scalar data.
	NamedData<ScalarType>& scalars();

	/// Return the scalar data structure.
	/// \return the read-only scalar data.
	const NamedData<ScalarType>& scalars() const;

	/// Return the integer data structure.
	/// \return the mutable integer data.
	NamedData<IntegerType>& integers();

	/// Return the integer data structure.
	/// \return the read-only integer data.
	const NamedData<IntegerType>& integers() const;

	/// Return the boolean data structure.
	/// \return the mutable Boolean data.
	NamedData<BooleanType>& booleans();

	/// Return the boolean data structure.
	/// \return the read-only Boolean data.
	const NamedData<BooleanType>& booleans() const;

	/// Return the string data structure.
	/// \return the mutable string data.
	NamedData<StringType>& strings();

	/// Return the string data structure.
	/// \return the read-only string data.
	const NamedData<StringType>& strings() const;

	/// Return the image data structure.
	/// \return the mutable iamge data.
	NamedData<ImageType>& images();

	/// Return the image data structure.
	/// \return the read-only images data.
	const NamedData<ImageType>& images() const;

	/// Return the custom data structure.
	/// \return the mutable data.
	NamedVariantData& customData();

	/// Return the custom data structure.
	/// \return the read-only data.
	const NamedVariantData& customData() const;

	/// Mark all data as not current.
	void resetAll();

	/// An empty DataGroup can be assigned to by any DataGroup with only valid NamedData.
	/// return true if all the NamedData are invalid.
	bool isEmpty() const;

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

	/// The string values.
	NamedData<ImageType> m_images;

	/// The custom data values.
	NamedVariantData m_customData;
};

};  // namespace DataStructures
};  // namespace SurgSim

#endif  // SURGSIM_DATASTRUCTURES_DATAGROUP_H
