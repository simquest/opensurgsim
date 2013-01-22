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

#include <SurgSim/Input/NamedDataBuilder.h>
#include <SurgSim/Input/DataGroup.h>

namespace SurgSim
{
namespace Input
{

/// A builder for a \c DataGroup.
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

	DataGroupBuilder() {};

	/// Produce an empty \ref DataGroup object with an immutable directory.
	/// \return the DataGroup object *by value*.
	DataGroup createData() const
	{
		DataGroup data;
		data.poses() = poses().createData();
		data.vectors() = vectors().createData();
		data.scalars() = scalars().createData();
		data.integers() = integers().createData();
		data.booleans() = booleans().createData();
		data.strings() = strings().createData();
		return data;
	}

	/// Produce a shared pointer to an empty \ref DataGroup object with an immutable directory.
	/// \return a shared pointer to the DataGroup object.
	std::shared_ptr<DataGroup> createSharedData() const
	{
		return std::make_shared<DataGroup>(createData());
	}


	/// Return the directory used for pose values.
	NamedDataBuilder<PoseType>& poses()
	{
		return m_poses;
	}

	/// Return the directory used for pose values.
	const NamedDataBuilder<PoseType>& poses() const
	{
		return m_poses;
	}

	/// Return the directory used for vector values.
	NamedDataBuilder<VectorType>& vectors()
	{
		return m_vectors;
	}

	/// Return the directory used for vector values.
	const NamedDataBuilder<VectorType>& vectors() const
	{
		return m_vectors;
	}

	/// Return the directory used for scalar values.
	NamedDataBuilder<ScalarType>& scalars()
	{
		return m_scalars;
	}

	/// Return the directory used for scalar values.
	const NamedDataBuilder<ScalarType>& scalars() const
	{
		return m_scalars;
	}

	/// Return the directory used for integer values.
	NamedDataBuilder<IntegerType>& integers()
	{
		return m_integers;
	}

	/// Return the directory used for integer values.
	const NamedDataBuilder<IntegerType>& integers() const
	{
		return m_integers;
	}

	/// Return the directory used for boolean values.
	NamedDataBuilder<BooleanType>& booleans()
	{
		return m_booleans;
	}

	/// Return the directory used for boolean values.
	const NamedDataBuilder<BooleanType>& booleans() const
	{
		return m_booleans;
	}

	/// Return the directory used for string values.
	NamedDataBuilder<StringType>& strings()
	{
		return m_strings;
	}

	/// Return the directory used for string values.
	const NamedDataBuilder<StringType>& strings() const
	{
		return m_strings;
	}

	/// A shortcut for adding a named pose entry.
	void addPose(const std::string& name)
	{
		poses().addEntry(name);
	}

	/// A shortcut for adding a named vector entry.
	void addVector(const std::string& name)
	{
		vectors().addEntry(name);
	}

	/// A shortcut for adding a named scalar entry.
	void addScalar(const std::string& name)
	{
		scalars().addEntry(name);
	}

	/// A shortcut for adding a named integer entry.
	void addInteger(const std::string& name)
	{
		integers().addEntry(name);
	}

	/// A shortcut for adding a named boolean entry.
	void addBoolean(const std::string& name)
	{
		booleans().addEntry(name);
	}

	/// A shortcut for adding a named string entry.
	void addString(const std::string& name)
	{
		strings().addEntry(name);
	}

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
