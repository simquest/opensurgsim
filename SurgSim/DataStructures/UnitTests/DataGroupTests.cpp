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

/// \file
/// Tests for the DataGroup class.

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"
#include "gtest/gtest.h"

#include "MockObjects.h"

using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;


/// Creating a named data object.
TEST(DataGroupTests, CanConstruct)
{
	DataGroupBuilder builder;
	builder.addPose("test");
	builder.addVector("test");
	builder.addMatrix("test");
	builder.addScalar("test");
	builder.addInteger("test");
	builder.addBoolean("test");
	builder.addString("test");
	builder.addCustom("test");
	DataGroup data = builder.createData();

	EXPECT_TRUE(data.isValid());
	EXPECT_TRUE(data.poses().hasEntry("test"));
	EXPECT_FALSE(data.poses().hasData("test"));
	EXPECT_FALSE(data.strings().hasEntry("missing"));
	EXPECT_FALSE(data.strings().hasData("missing"));
}

/// Creating a shared_ref to a named data object.
TEST(DataGroupTests, CanCreateShared)
{
	DataGroupBuilder builder;
	builder.addPose("test");
	builder.addVector("test");
	builder.addMatrix("test");
	builder.addScalar("test");
	builder.addInteger("test");
	builder.addBoolean("test");
	builder.addString("test");
	builder.addCustom("test");
	std::shared_ptr<DataGroup> data = builder.createSharedData();

	EXPECT_TRUE(data->isValid());
	EXPECT_TRUE(data->poses().hasEntry("test"));
	EXPECT_FALSE(data->poses().hasData("test"));
	EXPECT_FALSE(data->strings().hasEntry("missing"));
	EXPECT_FALSE(data->strings().hasData("missing"));
}

/// Creating an unitialized data object.
TEST(DataGroupTests, Uninitialized)
{
	DataGroup data;
	EXPECT_FALSE(data.isValid());
}

/// Putting data into the container.
TEST(DataGroupTests, PutName)
{
	DataGroupBuilder builder;
	builder.addPose("pose");
	builder.addVector("vector");
	builder.addMatrix("matrix");
	builder.addScalar("scalar");
	builder.addInteger("integer");
	builder.addBoolean("boolean");
	builder.addString("string");
	builder.addCustom("mock_data");
	DataGroup data = builder.createData();

	const SurgSim::Math::Vector3d vector(1.23, 4.56, 7.89);
	const SurgSim::Math::Quaterniond quat =
		SurgSim::Math::makeRotationQuaternion(M_PI_2, SurgSim::Math::Vector3d(1, 0, 0));
	const SurgSim::Math::RigidTransform3d pose = SurgSim::Math::makeRigidTransform(quat, vector);
	DataGroup::DynamicMatrixType matrix(2,3);
	matrix.fill(3.0);

	MockData mockData;
	mockData.intValue = 2;
	mockData.floatValue = 2.3;
	mockData.doubleVector.push_back(3.2);

	data.poses().set("pose", pose);
	data.vectors().set("vector", vector);
	data.matrices().set("matrix", matrix);
	data.scalars().set("scalar", 1.23f);
	data.integers().set("integer", 123);
	data.booleans().set("boolean", true);
	data.strings().set("string", "string");
	data.customs().set("mock_data", mockData);

	EXPECT_TRUE(data.poses().hasEntry("pose"));
	EXPECT_TRUE(data.poses().hasData("pose"));

	EXPECT_TRUE(data.vectors().hasEntry("vector"));
	EXPECT_TRUE(data.vectors().hasData("vector"));

	EXPECT_TRUE(data.matrices().hasEntry("matrix"));
	EXPECT_TRUE(data.matrices().hasData("matrix"));

	EXPECT_TRUE(data.scalars().hasEntry("scalar"));
	EXPECT_TRUE(data.scalars().hasData("scalar"));

	EXPECT_TRUE(data.integers().hasEntry("integer"));
	EXPECT_TRUE(data.integers().hasData("integer"));

	EXPECT_TRUE(data.booleans().hasEntry("boolean"));
	EXPECT_TRUE(data.booleans().hasData("boolean"));

	EXPECT_TRUE(data.strings().hasEntry("string"));
	EXPECT_TRUE(data.strings().hasData("string"));

	EXPECT_TRUE(data.customs().hasEntry("mock_data"));
	EXPECT_TRUE(data.customs().hasData("mock_data"));
}

/// Getting data from the container.
TEST(DataGroupTests, GetName)
{
	DataGroupBuilder builder;
	builder.addPose("pose");
	builder.addVector("vector");
	builder.addMatrix("matrix");
	builder.addScalar("scalar");
	builder.addInteger("integer");
	builder.addBoolean("boolean");
	builder.addString("string");
	builder.addCustom("mock_data");
	DataGroup data = builder.createData();

	const SurgSim::Math::Vector3d vector(1.23, 4.56, 7.89);
	const SurgSim::Math::Quaterniond quat =
		SurgSim::Math::makeRotationQuaternion(M_PI_2, SurgSim::Math::Vector3d(1, 0, 0));
	const SurgSim::Math::RigidTransform3d pose = SurgSim::Math::makeRigidTransform(quat, vector);
	DataGroup::DynamicMatrixType matrix(2,3);
	matrix.fill(3.0);

	MockData mockData;
	mockData.intValue = 2;
	mockData.floatValue = 2.3;
	mockData.doubleVector.push_back(3.2);

	data.poses().set("pose", pose);
	data.vectors().set("vector", vector);
	data.matrices().set("matrix", matrix);
	data.scalars().set("scalar", 1.23);
	data.integers().set("integer", 123);
	data.booleans().set("boolean", true);
	data.strings().set("string", "string");
	data.customs().set("mock_data", mockData);

	{
		SurgSim::Math::RigidTransform3d value = SurgSim::Math::RigidTransform3d::Identity();
		EXPECT_TRUE(data.poses().get("pose", &value));
		EXPECT_NEAR(0, (value.linear() - quat.matrix()).norm(), 1e-9);
		EXPECT_NEAR(0, (value.translation() - vector).norm(), 1e-9);
	}
	{
		SurgSim::Math::Vector3d value(0, 0, 0);
		EXPECT_TRUE(data.vectors().get("vector", &value));
		EXPECT_NEAR(0, (value - vector).norm(), 1e-9);
	}
	{
		DataGroup::DynamicMatrixType value;
		EXPECT_TRUE(data.matrices().get("matrix", &value));
		EXPECT_EQ(matrix.cols(), value.cols());
		EXPECT_EQ(matrix.rows(), value.rows());
		EXPECT_NEAR(0, (value - matrix).norm(), 1e-9);
	}
	{
		double value = 0;
		EXPECT_TRUE(data.scalars().get("scalar", &value));
		EXPECT_NEAR(1.23, value, 1e-9);
	}
	{
		int value = 0;
		EXPECT_TRUE(data.integers().get("integer", &value));
		EXPECT_EQ(123, value);
	}
	{
		bool value = 0;
		EXPECT_TRUE(data.booleans().get("boolean", &value));
		EXPECT_EQ(true, value);
	}
	{
		std::string value = "";
		EXPECT_TRUE(data.strings().get("string", &value));
		EXPECT_EQ("string", value);
	}
	{
		MockData value;
		EXPECT_TRUE(data.customs().get("mock_data", &value));
		EXPECT_EQ(mockData.intValue, value.intValue);
		EXPECT_NEAR(mockData.floatValue, value.floatValue, 1e-9);
		EXPECT_EQ(mockData.doubleVector.size(), value.doubleVector.size());
	}
}

/// Resetting the data in the container.
TEST(DataGroupTests, ResetAll)
{
	DataGroupBuilder builder;
	builder.addPose("first");
	builder.addScalar("second");
	builder.addString("third");
	builder.addCustom("fourth");
	DataGroup data = builder.createData();

	data.scalars().set("second", 1.23);
	data.strings().set("third", "hello");
	data.customs().set("fourth", MockData());

	data.resetAll();

	EXPECT_TRUE(data.poses().hasEntry("first"));
	EXPECT_FALSE(data.poses().hasData("first"));

	EXPECT_TRUE(data.scalars().hasEntry("second"));
	EXPECT_FALSE(data.scalars().hasData("second"));

	EXPECT_TRUE(data.strings().hasEntry("third"));
	EXPECT_FALSE(data.strings().hasData("third"));

	EXPECT_TRUE(data.customs().hasEntry("fourth"));
	EXPECT_FALSE(data.customs().hasData("fourth"));
}

/// Resetting one data entry at a time.
TEST(DataGroupTests, ResetOne)
{
	DataGroupBuilder builder;
	builder.addPose("first");
	builder.addScalar("second");
	builder.addString("third");
	builder.addCustom("fourth");
	DataGroup data = builder.createData();

	data.scalars().set("second", 1.23);
	data.strings().set("third", "hello");
	data.customs().set("fourth", MockData());

	data.strings().reset("third");

	EXPECT_TRUE(data.poses().hasEntry("first"));
	EXPECT_FALSE(data.poses().hasData("first"));

	EXPECT_TRUE(data.scalars().hasEntry("second"));
	EXPECT_TRUE(data.scalars().hasData("second"));

	EXPECT_TRUE(data.strings().hasEntry("third"));
	EXPECT_FALSE(data.strings().hasData("third"));

	EXPECT_TRUE(data.customs().hasEntry("fourth"));
	EXPECT_TRUE(data.customs().hasData("fourth"));

	data.scalars().reset("second");

	EXPECT_TRUE(data.scalars().hasEntry("second"));
	EXPECT_FALSE(data.scalars().hasData("second"));
}
