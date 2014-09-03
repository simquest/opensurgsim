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

#include "SurgSim/DataStructures/DataGroup.h"

namespace SurgSim
{
namespace DataStructures
{

DataGroup::DataGroup()
{
}

DataGroup::DataGroup(const DataGroup& dataGroup) :
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

DataGroup& DataGroup::operator=(const DataGroup& dataGroup)
{
	m_poses = dataGroup.m_poses;
	m_vectors = dataGroup.m_vectors;
	m_matrices = dataGroup.m_matrices;
	m_scalars = dataGroup.m_scalars;
	m_integers = dataGroup.m_integers;
	m_booleans = dataGroup.m_booleans;
	m_strings = dataGroup.m_strings;
	m_customData = dataGroup.m_customData;

	return *this;
}

DataGroup& DataGroup::operator=(DataGroup&& dataGroup)
{
	m_poses = std::move(dataGroup.m_poses);
	m_vectors = std::move(dataGroup.m_vectors);
	m_matrices = std::move(dataGroup.m_matrices);
	m_scalars = std::move(dataGroup.m_scalars);
	m_integers = std::move(dataGroup.m_integers);
	m_booleans = std::move(dataGroup.m_booleans);
	m_strings = std::move(dataGroup.m_strings);
	m_customData = std::move(dataGroup.m_customData);

	return *this;
}

NamedData<DataGroup::PoseType>& DataGroup::poses()
{
	return m_poses;
}

const NamedData<DataGroup::PoseType>& DataGroup::poses() const
{
	return m_poses;
}

NamedData<DataGroup::VectorType>& DataGroup::vectors()
{
	return m_vectors;
}

const NamedData<DataGroup::VectorType>& DataGroup::vectors() const
{
	return m_vectors;
}

NamedData<DataGroup::DynamicMatrixType>& DataGroup::matrices()
{
	return m_matrices;
}

const NamedData<DataGroup::DynamicMatrixType>& DataGroup::matrices() const
{
	return m_matrices;
}


NamedData<DataGroup::ScalarType>& DataGroup::scalars()
{
	return m_scalars;
}

const NamedData<DataGroup::ScalarType>& DataGroup::scalars() const
{
	return m_scalars;
}

NamedData<DataGroup::IntegerType>& DataGroup::integers()
{
	return m_integers;
}

const NamedData<DataGroup::IntegerType>& DataGroup::integers() const
{
	return m_integers;
}

NamedData<DataGroup::BooleanType>& DataGroup::booleans()
{
	return m_booleans;
}

const NamedData<DataGroup::BooleanType>& DataGroup::booleans() const
{
	return m_booleans;
}

NamedData<DataGroup::StringType>& DataGroup::strings()
{
	return m_strings;
}

const NamedData<DataGroup::StringType>& DataGroup::strings() const
{
	return m_strings;
}

NamedVariantData& DataGroup::customData()
{
	return m_customData;
}

const NamedVariantData& DataGroup::customData() const
{
	return m_customData;
}

void DataGroup::resetAll()
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

bool DataGroup::isEmpty() const
{
	return !(m_poses.isValid() ||
		m_vectors.isValid() ||
		m_matrices.isValid() ||
		m_scalars.isValid() ||
		m_integers.isValid() ||
		m_booleans.isValid() ||
		m_strings.isValid() ||
		m_customData.isValid());
}

};  // namespace DataStructures
};  // namespace SurgSim
