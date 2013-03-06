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

#ifndef SURGSIM_INPUT_DATA_GROUP_BUILDER_H
#define SURGSIM_INPUT_DATA_GROUP_BUILDER_H

#include <SurgSim/DataStructures/NamedDataBuilder.h>
#include <SurgSim/DataStructures/DataGroup.h>

namespace SurgSim
{
namespace DataStructures
{

/// A class that allows you to build a \ref DataGroup structure.
///
/// Since the data layout of a \ref DataGroup object cannot be modified, this class is helpful in initially setting
/// up the names and their corresponding indices.  You can add entries to the builder using the \ref addPose,
/// \ref addVector, \ref addScalar, \ref addInteger, \ref addBoolean, \ref addString, and \ref addEntriesFrom
/// calls, or using similar calls on one of the type-specific element groups; and then create the DataGroup
/// instance with createData() or createSharedData().
///
/// \sa DataGroup
class DataGroupBuilder
{
public:
	/// The type used for poses.
	typedef DataGroup::PoseType PoseType;
	/// The type used for vectors.
	typedef DataGroup::VectorType VectorType;
	/// The type used for scalars.
	typedef DataGroup::ScalarType ScalarType;
	/// The type used for integers.
	typedef DataGroup::IntegerType IntegerType;
	/// The type used for booleans.
	typedef DataGroup::BooleanType BooleanType;
	/// The type used for strings.
	typedef DataGroup::StringType StringType;

	/// Constructs an empty builder object.
	DataGroupBuilder();

	/// Produces a \ref DataGroup object with an immutable set of names and indices.
	/// None of the values will contain any current data.
	/// \return the DataGroup object *by value*.
	DataGroup createData() const;

	/// Produce a shared pointer to an empty \ref DataGroup object with an immutable set of names and indices.
	/// None of the values will contain any current data.
	/// \return a shared pointer to the DataGroup object.
	std::shared_ptr<DataGroup> createSharedData() const;

	/// Provides access to the pose value entries.
	/// \return a writable reference to the sub-object that contains pose value entries.
	NamedDataBuilder<PoseType>& poses();

	/// Provides access to the pose value entries.
	/// \return a read-only reference to the sub-object that contains pose value entries.
	const NamedDataBuilder<PoseType>& poses() const;

	/// Provides access to the vector value entries.
	/// \return a writable reference to the sub-object that contains vector value entries.
	NamedDataBuilder<VectorType>& vectors();

	/// Provides access to the vector value entries.
	/// \return a read-only reference to the sub-object that contains vector value entries.
	const NamedDataBuilder<VectorType>& vectors() const;

	/// Provides access to the scalar value entries.
	/// \return a writable reference to the sub-object that contains scalar value entries.
	NamedDataBuilder<ScalarType>& scalars();

	/// Provides access to the scalar value entries.
	/// \return a read-only reference to the sub-object that contains scalar value entries.
	const NamedDataBuilder<ScalarType>& scalars() const;

	/// Provides access to the integer value entries.
	/// \return a writable reference to the sub-object that contains integer value entries.
	NamedDataBuilder<IntegerType>& integers();

	/// Provides access to the integer value entries.
	/// \return a read-only reference to the sub-object that contains integer value entries.
	const NamedDataBuilder<IntegerType>& integers() const;

	/// Provides access to the Boolean value entries.
	/// \return a writable reference to the sub-object that contains Boolean value entries.
	NamedDataBuilder<BooleanType>& booleans();

	/// Provides access to the Boolean value entries.
	/// \return a read-only reference to the sub-object that contains Boolean value entries.
	const NamedDataBuilder<BooleanType>& booleans() const;

	/// Provides access to the string value entries.
	/// \return a writable reference to the sub-object that contains string value entries.
	NamedDataBuilder<StringType>& strings();

	/// Provides access to the string value entries.
	/// \return a read-only reference to the sub-object that contains string value entries.
	const NamedDataBuilder<StringType>& strings() const;

	/// A shortcut for adding a named pose entry.
	/// Identical to <code>%poses().addEntry(name)</code>.
	void addPose(const std::string& name);

	/// A shortcut for adding a named vector entry.
	/// Identical to <code>%vectors().addEntry(name)</code>.
	void addVector(const std::string& name);

	/// A shortcut for adding a named scalar entry.
	/// Identical to <code>%scalars().addEntry(name)</code>.
	void addScalar(const std::string& name);

	/// A shortcut for adding a named integer entry.
	/// Identical to <code>%integers().addEntry(name)</code>.
	void addInteger(const std::string& name);

	/// A shortcut for adding a named boolean entry.
	/// Identical to <code>%booleans().addEntry(name)</code>.
	void addBoolean(const std::string& name);

	/// A shortcut for adding a named string entry.
	/// Identical to <code>%strings().addEntry(name)</code>.
	void addString(const std::string& name);

	/// Create new entries from another DataGroupBuilder.
	/// \param builder The other builder.
	void addEntriesFrom(const DataGroupBuilder& builder);

	/// Create new entries from an already initialized DataGroup.
	/// \param data The data object.
	void addEntriesFrom(const DataGroup& data);

private:
	// Prevent copy construction and copy assignment.
	DataGroupBuilder(const DataGroupBuilder&);
	DataGroupBuilder& operator=(const DataGroupBuilder&);

	/// The subsidiary builder used for pose values.
	NamedDataBuilder<PoseType> m_poses;

	/// The subsidiary builder used for vector values.
	NamedDataBuilder<VectorType> m_vectors;

	/// The subsidiary builder used for scalar values.
	NamedDataBuilder<ScalarType> m_scalars;

	/// The subsidiary builder used for integer values.
	NamedDataBuilder<IntegerType> m_integers;

	/// The subsidiary builder used for boolean values.
	NamedDataBuilder<BooleanType> m_booleans;

	/// The subsidiary builder used for string values.
	NamedDataBuilder<StringType> m_strings;
};

};  // namespace Input
};  // namespace SurgSim

#endif  // SURGSIM_INPUT_DATA_GROUP_BUILDER_H
