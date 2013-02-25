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

/** @file
 * Tests for the IndexDirectory class.
 */

#include "SurgSim/Input/NamedDataBuilder.h"
#include "SurgSim/Input/IndexDirectory.h"
#include "gtest/gtest.h"

using SurgSim::Input::NamedDataBuilder;
using SurgSim::Input::IndexDirectory;


/// Run a few tests against an empty index directory.
TEST(IndexDirectoryTests, EmptyTests)
{
	{
		IndexDirectory dir;

		EXPECT_EQ(0, dir.getNumEntries());
		EXPECT_EQ("", dir.getName(0));

		EXPECT_EQ(-1, dir.getIndex("missing"));
		EXPECT_FALSE(dir.hasEntry("missing"));
	}
	{
		NamedDataBuilder<float> builder;

		EXPECT_EQ(0, builder.getNumEntries());
		EXPECT_EQ("", builder.getName(0));

		EXPECT_EQ(-1, builder.getIndex("missing"));
		EXPECT_FALSE(builder.hasEntry("missing"));

		std::shared_ptr<const IndexDirectory> dir = builder.createData().getDirectory();

		EXPECT_EQ(0, dir->getNumEntries());
		EXPECT_EQ("", dir->getName(0));

		EXPECT_EQ(-1, dir->getIndex("missing"));
		EXPECT_FALSE(dir->hasEntry("missing"));
	}
}

/// Populate an index directory and run a few tests against that.
TEST(IndexDirectoryTests, SmallDirectoryTests)
{
	NamedDataBuilder<float> builder;
	builder.addEntry("first");
	builder.addEntry("second");
	std::shared_ptr<const IndexDirectory> dir = builder.createData().getDirectory();

	EXPECT_EQ(2, dir->getNumEntries());
	EXPECT_EQ(2U, dir->size());

	EXPECT_EQ("first", dir->getName(0));
	EXPECT_EQ(0, dir->getIndex("first"));
	EXPECT_TRUE(dir->hasEntry("first"));

	EXPECT_EQ("second", dir->getName(1));
	EXPECT_EQ(1, dir->getIndex("second"));
	EXPECT_TRUE(dir->hasEntry("second"));

	EXPECT_EQ(-1, dir->getIndex("missing"));
	EXPECT_FALSE(dir->hasEntry("missing"));
}

/// Check return values from populating an index directory.
TEST(IndexDirectoryTests, ReturnValueFromAdd)
{
	NamedDataBuilder<float> builder;
	EXPECT_EQ(-1, builder.addEntry(""));
	EXPECT_EQ(0, builder.addEntry("entry"));
	EXPECT_EQ(-1, builder.addEntry("entry"));

	// Also check that the failed calls did not increase the number of entries:
	EXPECT_EQ(1, builder.getNumEntries());
	EXPECT_EQ(1, builder.createData().getDirectory()->getNumEntries());
}

/// Try passing bad key/index values.
TEST(IndexDirectoryTests, BadKeyOrIndex)
{
	NamedDataBuilder<float> builder;
	EXPECT_EQ(0, builder.addEntry("first"));
	EXPECT_EQ(1, builder.addEntry("second"));
	EXPECT_EQ(-1, builder.addEntry(""));

	std::shared_ptr<const IndexDirectory> dir = builder.createData().getDirectory();	
	EXPECT_EQ("", dir->getName(-1));
	EXPECT_EQ("", dir->getName(-12345));
	EXPECT_EQ("", dir->getName(+56789));
	EXPECT_EQ("", dir->getName(2));
}
