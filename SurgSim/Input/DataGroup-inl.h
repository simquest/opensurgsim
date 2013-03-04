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

#ifndef SURGSIM_INPUT_DATA_GROUP_INL_H
#define SURGSIM_INPUT_DATA_GROUP_INL_H

#include <SurgSim/Input/DataGroup.h>
#include <SurgSim/Framework/Assert.h>

namespace SurgSim
{
namespace Input
{

inline DataGroup::DataGroup()
{
}

inline DataGroup::DataGroup(const DataGroup& dataGroup) :
	m_poses(dataGroup.m_poses),
	m_vectors(dataGroup.m_vectors),
	m_scalars(dataGroup.m_scalars),
	m_integers(dataGroup.m_integers),
	m_booleans(dataGroup.m_booleans),
	m_strings(dataGroup.m_strings)
{
}

inline DataGroup& DataGroup::operator=(const DataGroup& dataGroup)
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

inline DataGroup& DataGroup::operator=(DataGroup&& dataGroup)
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

inline bool DataGroup::isValid() const
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

inline NamedData<DataGroup::PoseType>& DataGroup::poses()
{
	return m_poses;
}

inline const NamedData<DataGroup::PoseType>& DataGroup::poses() const
{
	return m_poses;
}

inline NamedData<DataGroup::VectorType>& DataGroup::vectors()
{
	return m_vectors;
}

inline const NamedData<DataGroup::VectorType>& DataGroup::vectors() const
{
	return m_vectors;
}

inline NamedData<DataGroup::ScalarType>& DataGroup::scalars()
{
	return m_scalars;
}

inline const NamedData<DataGroup::ScalarType>& DataGroup::scalars() const
{
	return m_scalars;
}

inline NamedData<DataGroup::IntegerType>& DataGroup::integers()
{
	return m_integers;
}

inline const NamedData<DataGroup::IntegerType>& DataGroup::integers() const
{
	return m_integers;
}

inline NamedData<DataGroup::BooleanType>& DataGroup::booleans()
{
	return m_booleans;
}

inline const NamedData<DataGroup::BooleanType>& DataGroup::booleans() const
{
	return m_booleans;
}

inline NamedData<DataGroup::StringType>& DataGroup::strings()
{
	return m_strings;
}

inline const NamedData<DataGroup::StringType>& DataGroup::strings() const
{
	return m_strings;
}

inline void DataGroup::resetAll()
{
	m_poses.resetAll();
	m_vectors.resetAll();
	m_scalars.resetAll();
	m_integers.resetAll();
	m_booleans.resetAll();
	m_strings.resetAll();
}


};  // namespace Input
};  // namespace SurgSim

#endif  // SURGSIM_INPUT_DATA_GROUP_INL_H
