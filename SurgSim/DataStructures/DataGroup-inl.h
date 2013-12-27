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

#ifndef SURGSIM_DATASTRUCTURES_DATAGROUP_INL_H
#define SURGSIM_DATASTRUCTURES_DATAGROUP_INL_H

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Framework/Assert.h"

namespace SurgSim
{
namespace DataStructures
{

inline DataGroup::DataGroup()
{
}

inline DataGroup::DataGroup(const DataGroup& dataGroup) :
	m_poses(dataGroup.m_poses),
	m_vectors(dataGroup.m_vectors),
	m_matrices(dataGroup.m_matrices),
	m_scalars(dataGroup.m_scalars),
	m_integers(dataGroup.m_integers),
	m_booleans(dataGroup.m_booleans),
	m_strings(dataGroup.m_strings),
	m_customData(dataGroup.m_customData)
{
}

inline DataGroup& DataGroup::operator=(const DataGroup& dataGroup)
{
	SURGSIM_ASSERT(dataGroup.isInitialized()) <<
		"Cannot use an uninitialized DataGroup on the right-hand side of an assignment!";

	m_poses = dataGroup.m_poses;
	m_vectors = dataGroup.m_vectors;
	m_matrices = dataGroup.m_matrices;
	m_scalars = dataGroup.m_scalars;
	m_integers = dataGroup.m_integers;
	m_booleans = dataGroup.m_booleans;
	m_strings = dataGroup.m_strings;
	m_customData = dataGroup.m_customData;

	SURGSIM_ASSERT(isInitialized()) << "DataGroup is not initialized after assignment!";
	return *this;
}

inline DataGroup& DataGroup::operator=(DataGroup&& dataGroup)
{
	SURGSIM_ASSERT(dataGroup.isInitialized()) <<
		"Cannot use an uninitialized DataGroup on the right-hand side of an assignment!";

	m_poses = std::move(dataGroup.m_poses);
	m_vectors = std::move(dataGroup.m_vectors);
	m_matrices = std::move(dataGroup.m_matrices);
	m_scalars = std::move(dataGroup.m_scalars);
	m_integers = std::move(dataGroup.m_integers);
	m_booleans = std::move(dataGroup.m_booleans);
	m_strings = std::move(dataGroup.m_strings);
	m_customData = std::move(dataGroup.m_customData);

	SURGSIM_ASSERT(isInitialized()) << "DataGroup is not initialized after assignment!";
	return *this;
}

inline bool DataGroup::isInitialized() const
{
	bool initialized = poses().isInitialized();
	SURGSIM_ASSERT(vectors().isInitialized() == initialized &&
				   matrices().isInitialized() == initialized &&
				   scalars().isInitialized() == initialized &&
				   integers().isInitialized() == initialized &&
				   booleans().isInitialized() == initialized &&
				   strings().isInitialized() == initialized &&
				   customData().isInitialized() == initialized) << "The DataGroup is only partially initialized!";
	return initialized;
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

inline NamedData<DataGroup::DynamicMatrixType>& DataGroup::matrices()
{
	return m_matrices;
}

inline const NamedData<DataGroup::DynamicMatrixType>& DataGroup::matrices() const
{
	return m_matrices;
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

inline NamedVariantData& DataGroup::customData()
{
	return m_customData;
}

inline const NamedVariantData& DataGroup::customData() const
{
	return m_customData;
}

inline void DataGroup::resetAll()
{
	m_poses.resetAll();
	m_vectors.resetAll();
	m_matrices.resetAll();
	m_scalars.resetAll();
	m_integers.resetAll();
	m_booleans.resetAll();
	m_strings.resetAll();
	m_customData.resetAll();
}


};  // namespace Input
};  // namespace SurgSim

#endif  // SURGSIM_DATASTRUCTURES_DATAGROUP_INL_H
