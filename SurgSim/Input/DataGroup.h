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
/// Each entry can also be marked as currently valid or missing.
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


	/// Create an empty object, with no associated directories yet.
	DataGroup() {};

	/// Create an object and copy the data from another object.
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

	/// Check whether the object is valid, meaning it has a valid directory.
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
	NamedData<PoseType>& poses()
	{
		return m_poses;
	}

	/// Return the pose data structure.
	const NamedData<PoseType>& poses() const
	{
		return m_poses;
	}

	/// Return the vector data structure.
	NamedData<VectorType>& vectors()
	{
		return m_vectors;
	}

	/// Return the vector data structure.
	const NamedData<VectorType>& vectors() const
	{
		return m_vectors;
	}

	/// Return the scalar data structure.
	NamedData<ScalarType>& scalars()
	{
		return m_scalars;
	}

	/// Return the scalar data structure.
	const NamedData<ScalarType>& scalars() const
	{
		return m_scalars;
	}

	/// Return the integer data structure.
	NamedData<IntegerType>& integers()
	{
		return m_integers;
	}

	/// Return the integer data structure.
	const NamedData<IntegerType>& integers() const
	{
		return m_integers;
	}

	/// Return the boolean data structure.
	NamedData<BooleanType>& booleans()
	{
		return m_booleans;
	}

	/// Return the boolean data structure.
	const NamedData<BooleanType>& booleans() const
	{
		return m_booleans;
	}

	/// Return the string data structure.
	NamedData<StringType>& strings()
	{
		return m_strings;
	}

	/// Return the string data structure.
	const NamedData<StringType>& strings() const
	{
		return m_strings;
	}

	/// Mark all data as not current.
	void reset()
	{
		m_poses.reset();
		m_vectors.reset();
		m_scalars.reset();
		m_integers.reset();
		m_booleans.reset();
		m_strings.reset();
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
