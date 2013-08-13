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

#include <SurgSim/DataStructures/DataGroupBuilder.h>

namespace SurgSim
{
namespace DataStructures
{


DataGroupBuilder::DataGroupBuilder()
{
}

DataGroup DataGroupBuilder::createData() const
{
	DataGroup data;
	data.poses() = poses().createData();
	data.vectors() = vectors().createData();
	data.matrices() = matrices().createData();
	data.scalars() = scalars().createData();
	data.integers() = integers().createData();
	data.booleans() = booleans().createData();
	data.strings() = strings().createData();
	data.customData() = customData().createData();
	return data;
}

std::shared_ptr<DataGroup> DataGroupBuilder::createSharedData() const
{
	return std::make_shared<DataGroup>(createData());
}

NamedDataBuilder<DataGroupBuilder::PoseType>& DataGroupBuilder::poses()
{
	return m_poses;
}

const NamedDataBuilder<DataGroupBuilder::PoseType>& DataGroupBuilder::poses() const
{
	return m_poses;
}

NamedDataBuilder<DataGroupBuilder::VectorType>& DataGroupBuilder::vectors()
{
	return m_vectors;
}

const NamedDataBuilder<DataGroupBuilder::VectorType>& DataGroupBuilder::vectors() const
{
	return m_vectors;
}

NamedDataBuilder<DataGroupBuilder::DynamicMatrixType>& DataGroupBuilder::matrices()
{
	return m_matrices;
}

const NamedDataBuilder<DataGroupBuilder::DynamicMatrixType>& DataGroupBuilder::matrices() const
{
	return m_matrices;
}

NamedDataBuilder<DataGroupBuilder::ScalarType>& DataGroupBuilder::scalars()
{
	return m_scalars;
}

const NamedDataBuilder<DataGroupBuilder::ScalarType>& DataGroupBuilder::scalars() const
{
	return m_scalars;
}

NamedDataBuilder<DataGroupBuilder::IntegerType>& DataGroupBuilder::integers()
{
	return m_integers;
}

const NamedDataBuilder<DataGroupBuilder::IntegerType>& DataGroupBuilder::integers() const
{
	return m_integers;
}

NamedDataBuilder<DataGroupBuilder::BooleanType>& DataGroupBuilder::booleans()
{
	return m_booleans;
}

const NamedDataBuilder<DataGroupBuilder::BooleanType>& DataGroupBuilder::booleans() const
{
	return m_booleans;
}

NamedDataBuilder<DataGroupBuilder::StringType>& DataGroupBuilder::strings()
{
	return m_strings;
}

const NamedDataBuilder<DataGroupBuilder::StringType>& DataGroupBuilder::strings() const
{
	return m_strings;
}

NamedVariantDataBuilder& DataGroupBuilder::customData()
{
	return m_customData;
}

const NamedVariantDataBuilder& DataGroupBuilder::customData() const
{
	return m_customData;
}

void DataGroupBuilder::addPose(const std::string& name)
{
	poses().addEntry(name);
}

void DataGroupBuilder::addVector(const std::string& name)
{
	vectors().addEntry(name);
}

void DataGroupBuilder::addMatrix(const std::string& name)
{
	matrices().addEntry(name);
}

void DataGroupBuilder::addScalar(const std::string& name)
{
	scalars().addEntry(name);
}

void DataGroupBuilder::addInteger(const std::string& name)
{
	integers().addEntry(name);
}

void DataGroupBuilder::addBoolean(const std::string& name)
{
	booleans().addEntry(name);
}

void DataGroupBuilder::addString(const std::string& name)
{
	strings().addEntry(name);
}

void DataGroupBuilder::addCustom(const std::string& name)
{
	customData().addEntry(name);
}


void DataGroupBuilder::addEntriesFrom(const DataGroupBuilder& builder)
{
	poses().addEntriesFrom(builder.poses());
	vectors().addEntriesFrom(builder.vectors());
	matrices().addEntriesFrom(builder.matrices());
	scalars().addEntriesFrom(builder.scalars());
	integers().addEntriesFrom(builder.integers());
	booleans().addEntriesFrom(builder.booleans());
	strings().addEntriesFrom(builder.strings());
	customData().addEntriesFrom(builder.customData());
}

void DataGroupBuilder::addEntriesFrom(const DataGroup& data)
{
	poses().addEntriesFrom(data.poses());
	vectors().addEntriesFrom(data.vectors());
	matrices().addEntriesFrom(data.matrices());
	scalars().addEntriesFrom(data.scalars());
	integers().addEntriesFrom(data.integers());
	booleans().addEntriesFrom(data.booleans());
	strings().addEntriesFrom(data.strings());
	customData().addEntriesFrom(data.customData());
}


};  // namespace Input
};  // namespace SurgSim
