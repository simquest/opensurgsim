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
#include "SurgSim/DataStructures/DataGroupCopier.h"
#include "SurgSim/DataStructures/NamedData.h"
#include "SurgSim/Framework/LockedContainer.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"
#include "gtest/gtest.h"

#include "SurgSim/DataStructures/UnitTests/MockObjects.h"

using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;
using SurgSim::DataStructures::DataGroupCopier;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

namespace
{
const double EPSILON = 1e-9;
};

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
	builder.addImage("test");
	builder.addCustom("test");
	DataGroup data = builder.createData();
	EXPECT_TRUE(data.poses().hasEntry("test"));
	EXPECT_FALSE(data.poses().hasData("test"));
	EXPECT_FALSE(data.strings().hasEntry("missing"));
	EXPECT_FALSE(data.strings().hasData("missing"));

	DataGroupBuilder builder2;
	builder2.addInteger("test");
	DataGroup data2 = builder2.createData();
	EXPECT_TRUE(data2.integers().hasEntry("test"));
	EXPECT_FALSE(data2.integers().hasData("test"));
	EXPECT_FALSE(data2.strings().hasEntry("missing"));
	EXPECT_FALSE(data2.strings().hasData("missing"));

	DataGroupBuilder builder3;
	DataGroup data3, data4 = builder3.createData();
	EXPECT_NO_THROW(data3 = data4);  // A DataGroup created by an empty DataGroupBuilder is valid (aka non-empty).
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
	builder.addImage("test");
	builder.addCustom("test");
	std::shared_ptr<DataGroup> data = builder.createSharedData();

	EXPECT_TRUE(data->poses().hasEntry("test"));
	EXPECT_FALSE(data->poses().hasData("test"));
	EXPECT_FALSE(data->strings().hasEntry("missing"));
	EXPECT_FALSE(data->strings().hasData("missing"));
}

/// Creating an invalid (aka empty) data object.
TEST(DataGroupTests, Uninitialized)
{
	DataGroup data, data2;
	EXPECT_THROW(data = data2, SurgSim::Framework::AssertionFailure);
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
	builder.addImage("image");
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

	DataGroup::ImageType mockImage(3,3,1);

	data.poses().set("pose", pose);
	data.vectors().set("vector", vector);
	data.matrices().set("matrix", matrix);
	data.scalars().set("scalar", 1.23f);
	data.integers().set("integer", 123);
	data.booleans().set("boolean", true);
	data.strings().set("string", "string");
	data.images().set("image", mockImage);
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

	EXPECT_TRUE(data.images().hasEntry("image"));
	EXPECT_TRUE(data.images().hasData("image"));

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
	builder.addImage("image");
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

	DataGroup::ImageType mockImage(3,3,1);
	mockImage.getChannel(0) << 0, 3, 6,
							   1, 4, 7,
							   2, 5, 8;

	data.poses().set("pose", pose);
	data.vectors().set("vector", vector);
	data.matrices().set("matrix", matrix);
	data.scalars().set("scalar", 1.23);
	data.integers().set("integer", 123);
	data.booleans().set("boolean", true);
	data.strings().set("string", "string");
	data.images().set("image", mockImage);
	data.customData().set("mock_data", mockData);

	{
		SurgSim::Math::RigidTransform3d value = SurgSim::Math::RigidTransform3d::Identity();
		EXPECT_TRUE(data.poses().get("pose", &value));
		EXPECT_NEAR(0, (value.linear() - quat.matrix()).norm(), EPSILON);
		EXPECT_NEAR(0, (value.translation() - vector).norm(), EPSILON);
	}
	{
		SurgSim::Math::Vector3d value(0, 0, 0);
		EXPECT_TRUE(data.vectors().get("vector", &value));
		EXPECT_NEAR(0, (value - vector).norm(), EPSILON);
	}
	{
		DataGroup::DynamicMatrixType value;
		EXPECT_TRUE(data.matrices().get("matrix", &value));
		EXPECT_EQ(matrix.cols(), value.cols());
		EXPECT_EQ(matrix.rows(), value.rows());
		EXPECT_NEAR(0, (value - matrix).norm(), EPSILON);
	}
	{
		double value = 0;
		EXPECT_TRUE(data.scalars().get("scalar", &value));
		EXPECT_NEAR(1.23, value, EPSILON);
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
		DataGroup::ImageType value;
		EXPECT_TRUE(data.images().get("image", &value));
		EXPECT_NEAR(0.0f, value.getData()[0], EPSILON);
		EXPECT_NEAR(1.0f, value.getData()[1], EPSILON);
		EXPECT_NEAR(2.0f, value.getData()[2], EPSILON);
		EXPECT_NEAR(3.0f, value.getData()[3], EPSILON);
		EXPECT_NEAR(4.0f, value.getData()[4], EPSILON);
		EXPECT_NEAR(5.0f, value.getData()[5], EPSILON);
		EXPECT_NEAR(6.0f, value.getData()[6], EPSILON);
		EXPECT_NEAR(7.0f, value.getData()[7], EPSILON);
		EXPECT_NEAR(8.0f, value.getData()[8], EPSILON);
	}
	{
		Mock3DData<double> value;
		EXPECT_TRUE(data.customData().get("mock_data", &value));
		EXPECT_NEAR(1.23, value.get(5, 5, 5), EPSILON);
		EXPECT_NEAR(4.56, value.get(1, 2, 3), EPSILON);
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
	builder.addImage("fifth");
	DataGroup data = builder.createData();

	data.scalars().set("second", 1.23);
	data.strings().set("third", "hello");
	data.customData().set("fourth", Mock3DData<double>(10, 10, 10));
	data.images().set("fifth", DataGroup::ImageType(10,10,2));

	data.resetAll();

	EXPECT_TRUE(data.poses().hasEntry("first"));
	EXPECT_FALSE(data.poses().hasData("first"));

	EXPECT_TRUE(data.scalars().hasEntry("second"));
	EXPECT_FALSE(data.scalars().hasData("second"));

	EXPECT_TRUE(data.strings().hasEntry("third"));
	EXPECT_FALSE(data.strings().hasData("third"));

	EXPECT_TRUE(data.customData().hasEntry("fourth"));
	EXPECT_FALSE(data.customData().hasData("fourth"));

	EXPECT_TRUE(data.images().hasEntry("fifth"));
	EXPECT_FALSE(data.images().hasData("fifth"));
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
	// Having the same entries is not sufficient for DataGroup assignment.
	// There are three situations in which assignment will not assert:
	// 1) the DataGroup being assigned to is "empty" (i.e., was default-constructed and has not yet been assigned to or
	// otherwise altered),
	// 2) one of the DataGroups was default-constructed and then the other DataGroup was assigned to it, or
	// 3) one of the DataGroups was copy-constructed from the other.
	EXPECT_THROW(data = data3, SurgSim::Framework::AssertionFailure);

	DataGroup data4(data);
	data4.booleans().set("test2", !trueBool);
	EXPECT_NO_THROW(data = data4); // data4 can assign to data because data4 was copy-constructed from data
	EXPECT_NO_THROW(data4 = data);
}

/// Non-assignment copying DataGroups with DataGroupCopier.
TEST(DataGroupTests, DataGroupCopier)
{
	DataGroupBuilder sourceBuilder;
	sourceBuilder.addPose("test");
	sourceBuilder.addBoolean("test");
	sourceBuilder.addBoolean("test2");
	DataGroup sourceData = sourceBuilder.createData();

	DataGroupBuilder targetBuilder;
	targetBuilder.addPose("test2"); //different pose name
	targetBuilder.addBoolean("test2"); // same boolean name
	targetBuilder.addEntriesFrom(sourceData);
	DataGroup targetData = targetBuilder.createData();

	ASSERT_TRUE(targetData.poses().hasEntry("test"));
	ASSERT_TRUE(targetData.poses().hasEntry("test2"));
	ASSERT_TRUE(targetData.booleans().hasEntry("test"));
	ASSERT_TRUE(targetData.booleans().hasEntry("test2"));

	ASSERT_THROW(targetData = sourceData, SurgSim::Framework::AssertionFailure);

	ASSERT_THROW(DataGroupCopier badCopier(sourceData, nullptr), SurgSim::Framework::AssertionFailure);
	DataGroupCopier copier(sourceData, &targetData);

	const RigidTransform3d testPose = SurgSim::Math::makeRigidTransform(Vector3d(1.0, 2.0, 3.0),
		Vector3d(1.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0));
	ASSERT_TRUE(sourceData.poses().set("test", testPose));

	const bool testBoolean = true;
	ASSERT_TRUE(sourceData.booleans().set("test", testBoolean));

	const bool testBoolean2 = false;
	ASSERT_TRUE(sourceData.booleans().set("test2", testBoolean2));
	// Setting the initial value for the targetData's test2 boolean to the opposite value.
	ASSERT_TRUE(targetData.booleans().set("test2", !testBoolean2));

	// Copy the data.
	ASSERT_THROW(copier.copy(sourceData, nullptr), SurgSim::Framework::AssertionFailure);
	ASSERT_NO_THROW(copier.copy(sourceData, &targetData));

	RigidTransform3d outTestPose;
	ASSERT_TRUE(targetData.poses().get("test", &outTestPose));
	EXPECT_TRUE(outTestPose.isApprox(testPose, EPSILON));

	EXPECT_FALSE(targetData.poses().hasData("test2"));

	bool outTestBoolean;
	ASSERT_TRUE(targetData.booleans().get("test", &outTestBoolean));
	EXPECT_EQ(testBoolean, outTestBoolean);

	bool outTestBoolean2;
	ASSERT_TRUE(targetData.booleans().get("test2", &outTestBoolean2));
	EXPECT_EQ(testBoolean2, outTestBoolean2);
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
	// the DataGroup in the LockedContainer was default-constructed and so is invalid (aka empty)
	// you cannot "get" an invalid DataGroup out of the LockedContainer.  "set" must be called before "get".
	EXPECT_THROW(lockedDataGroup.get(&copied_data), SurgSim::Framework::AssertionFailure);

	lockedDataGroup.set(data);
	lockedDataGroup.get(&copied_data);
	EXPECT_TRUE(copied_data.booleans().hasEntry("test"));
	EXPECT_TRUE(copied_data.booleans().hasData("test"));
	bool outBool, outCopiedBool;
	data.booleans().get("test", &outBool);
	copied_data.booleans().get("test", &outCopiedBool);
	EXPECT_EQ(outBool, outCopiedBool);
	EXPECT_EQ(trueBool, outCopiedBool);
}

TEST(DataGroupTests, IsEmpty)
{
	DataGroupBuilder builder;
	builder.addBoolean("test");
	DataGroup data;
	EXPECT_TRUE(data.isEmpty());

	data = builder.createData();
	EXPECT_FALSE(data.isEmpty());

	DataGroup data2;
	std::vector<std::string> names;
	names.push_back("test string");
	data2.scalars() = SurgSim::DataStructures::NamedData<SurgSim::DataStructures::DataGroup::ScalarType>(names);
	EXPECT_FALSE(data2.isEmpty());
}
