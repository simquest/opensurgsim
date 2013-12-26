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
#include "SurgSim/Framework/LockedContainer.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"
#include "gtest/gtest.h"

#include "SurgSim/DataStructures/UnitTests/MockObjects.h"

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

	DataGroupBuilder builder2;
	builder2.addInteger("test");
	DataGroup data2 = builder2.createData();
	EXPECT_TRUE(data2.isValid());
	EXPECT_TRUE(data2.integers().hasEntry("test"));
	EXPECT_FALSE(data2.integers().hasData("test"));
	EXPECT_FALSE(data2.strings().hasEntry("missing"));
	EXPECT_FALSE(data2.strings().hasData("missing"));

	DataGroupBuilder builder3;
	DataGroup data3 = builder3.createData();
	EXPECT_TRUE(data3.isValid());  // A DataGroup created by an empty DataGroupBuilder is valid.
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

/// Creating an uninitialized data object.
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

	Mock3DData<double> mockData(10,10,10);
	mockData.set(5, 5, 5, 1.2345);

	data.poses().set("pose", pose);
	data.vectors().set("vector", vector);
	data.matrices().set("matrix", matrix);
	data.scalars().set("scalar", 1.23f);
	data.integers().set("integer", 123);
	data.booleans().set("boolean", true);
	data.strings().set("string", "string");
	data.customData().set("mock_data", mockData);

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

	EXPECT_TRUE(data.customData().hasEntry("mock_data"));
	EXPECT_TRUE(data.customData().hasData("mock_data"));
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

	Mock3DData<double> mockData(10,10,10);
	mockData.set(5, 5, 5, 1.23);
	mockData.set(1, 2, 3, 4.56);

	data.poses().set("pose", pose);
	data.vectors().set("vector", vector);
	data.matrices().set("matrix", matrix);
	data.scalars().set("scalar", 1.23);
	data.integers().set("integer", 123);
	data.booleans().set("boolean", true);
	data.strings().set("string", "string");
	data.customData().set("mock_data", mockData);

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
		Mock3DData<double> value;
		EXPECT_TRUE(data.customData().get("mock_data", &value));
		EXPECT_NEAR(1.23, value.get(5, 5, 5), 1e-9);
		EXPECT_NEAR(4.56, value.get(1, 2, 3), 1e-9);
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
	data.customData().set("fourth", Mock3DData<double>(10, 10, 10));

	data.resetAll();

	EXPECT_TRUE(data.poses().hasEntry("first"));
	EXPECT_FALSE(data.poses().hasData("first"));

	EXPECT_TRUE(data.scalars().hasEntry("second"));
	EXPECT_FALSE(data.scalars().hasData("second"));

	EXPECT_TRUE(data.strings().hasEntry("third"));
	EXPECT_FALSE(data.strings().hasData("third"));

	EXPECT_TRUE(data.customData().hasEntry("fourth"));
	EXPECT_FALSE(data.customData().hasData("fourth"));
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
	data.customData().set("fourth", Mock3DData<double>(10, 10, 10));

	data.strings().reset("third");

	EXPECT_TRUE(data.poses().hasEntry("first"));
	EXPECT_FALSE(data.poses().hasData("first"));

	EXPECT_TRUE(data.scalars().hasEntry("second"));
	EXPECT_TRUE(data.scalars().hasData("second"));

	EXPECT_TRUE(data.strings().hasEntry("third"));
	EXPECT_FALSE(data.strings().hasData("third"));

	EXPECT_TRUE(data.customData().hasEntry("fourth"));
	EXPECT_TRUE(data.customData().hasData("fourth"));

	data.scalars().reset("second");

	EXPECT_TRUE(data.scalars().hasEntry("second"));
	EXPECT_FALSE(data.scalars().hasData("second"));
}

/// Copy Constructing DataGroups
TEST(DataGroupTests, CopyConstruction)
{
	DataGroupBuilder builder;
	builder.addPose("test");
	builder.addBoolean("test");
	builder.addBoolean("test2");
	DataGroup data = builder.createData();
	const bool trueBool = true;
	data.booleans().set("test2", trueBool);
	DataGroup copied_data = data;
	EXPECT_TRUE(copied_data.isValid());
	EXPECT_TRUE(copied_data.poses().hasEntry("test"));
	EXPECT_FALSE(copied_data.poses().hasData("test"));
	EXPECT_TRUE(copied_data.booleans().hasEntry("test"));
	EXPECT_FALSE(copied_data.booleans().hasData("test"));
	EXPECT_TRUE(copied_data.booleans().hasEntry("test2"));
	EXPECT_TRUE(copied_data.booleans().hasData("test2"));
	bool outBool, outCopiedBool;
	data.booleans().get("test2", &outBool);
	copied_data.booleans().get("test2", &outCopiedBool);
	EXPECT_EQ(outBool, outCopiedBool);
	EXPECT_EQ(trueBool, outCopiedBool);
	EXPECT_FALSE(copied_data.strings().hasEntry("missing"));
	EXPECT_FALSE(copied_data.strings().hasData("missing"));
}

/// Assigning DataGroups, testing DataGroup::operator=
TEST(DataGroupTests, Assignment)
{
	DataGroupBuilder builder;
	builder.addPose("test");
	builder.addBoolean("test");
	builder.addBoolean("test2");
	DataGroup data = builder.createData();
	const bool trueBool = true;
	data.booleans().set("test2", trueBool);
	DataGroup copied_data;
	copied_data = data;
	EXPECT_TRUE(copied_data.isValid());
	EXPECT_TRUE(copied_data.poses().hasEntry("test"));
	EXPECT_FALSE(copied_data.poses().hasData("test"));
	EXPECT_TRUE(copied_data.booleans().hasEntry("test"));
	EXPECT_FALSE(copied_data.booleans().hasData("test"));
	EXPECT_TRUE(copied_data.booleans().hasEntry("test2"));
	EXPECT_TRUE(copied_data.booleans().hasData("test2"));
	bool outBool, outCopiedBool;
	data.booleans().get("test2", &outBool);
	copied_data.booleans().get("test2", &outCopiedBool);
	EXPECT_EQ(outBool, outCopiedBool);
	EXPECT_EQ(trueBool, outCopiedBool);
	EXPECT_FALSE(copied_data.strings().hasEntry("missing"));
	EXPECT_FALSE(copied_data.strings().hasData("missing"));

	DataGroup data2;
	EXPECT_THROW(data = data2, SurgSim::Framework::AssertionFailure); // the right-hand DataGroup is not valid

	DataGroup data3 = builder.createData();
	// The left-hand DataGroup must be either empty or copy-constructed from the right-hand DataGroup.
	// Having the same entries is not sufficient.
	EXPECT_THROW(data = data3, SurgSim::Framework::AssertionFailure);

	DataGroup data5(data); // data5 can assign to data because data5 was copy-constructed from data
	data5.booleans().set("test2", !trueBool);
	EXPECT_NO_THROW(data = data5);
	EXPECT_NO_THROW(data5 = data);
}

TEST(DataGroupTests, DataGroupInLockedContainer)
{
	DataGroupBuilder builder;
	builder.addBoolean("test");
	DataGroup data = builder.createData();
	const bool trueBool = true;
	data.booleans().set("test", trueBool);
	SurgSim::Framework::LockedContainer<SurgSim::DataStructures::DataGroup> lockedDataGroup;
	DataGroup copied_data;
	// the DataGroup in the LockedContainer was default-constructed and so is not valid
	// you cannot "get" an invalid DataGroup out of the LockedContainer...so don't put one in there to begin with or 
	// the call to get will fail
	EXPECT_THROW(lockedDataGroup.get(&copied_data), SurgSim::Framework::AssertionFailure);
	
	lockedDataGroup.set(data);
	lockedDataGroup.get(&copied_data);
	EXPECT_TRUE(copied_data.isValid());
	EXPECT_TRUE(copied_data.booleans().hasEntry("test"));
	EXPECT_TRUE(copied_data.booleans().hasData("test"));
	bool outBool, outCopiedBool;
	data.booleans().get("test", &outBool);
	copied_data.booleans().get("test", &outCopiedBool);
	EXPECT_EQ(outBool, outCopiedBool);
	EXPECT_EQ(trueBool, outCopiedBool);
}