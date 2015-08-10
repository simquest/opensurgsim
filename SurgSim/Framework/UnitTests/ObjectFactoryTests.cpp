// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
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

#include <gtest/gtest.h>
#include "SurgSim/Framework/ObjectFactory.h"

namespace
{


class TestClass
{
public:
	TestClass() : stringValue("default"), className("TestClass")
	{

	}

	~TestClass()
	{

	}

	explicit TestClass(const std::string& val) : stringValue(val)
	{

	}

	std::string stringValue;
	std::string className;
};

class TestClassA : public TestClass
{
public:
	TestClassA() : TestClass("a")
	{
		className = "TestClassA";
	}

	explicit TestClassA(const std::string& val) : TestClass(val)
	{
		className = "TestClassA";
	}

};

class TestClassB : public TestClass
{
public:
	TestClassB() : TestClass("b")
	{
		className = "TestClassB";
	}

	explicit TestClassB(const std::string& val) : TestClass(val)
	{
		className = "TestClassB";
	}
};

}


namespace SurgSim
{
namespace Framework
{

TEST(ObjectFactoryTests, Single)
{
	ObjectFactory<TestClass> factory;

	factory.registerClass<TestClassA>("TestClassA");
	factory.registerClass<TestClassB>("TestClassB");

	auto a = factory.create("TestClassA");
	EXPECT_NE(nullptr, a);
	EXPECT_EQ("a", a->stringValue);

	auto b = factory.create("TestClassB");
	EXPECT_NE(nullptr, b);
	EXPECT_EQ("b", b->stringValue);

	EXPECT_ANY_THROW(factory.create("xxx"));
}

TEST(ObjectFactoryTests, OneParameter)
{
	ObjectFactory1<TestClass, std::string> factory;
	factory.registerClass<TestClassA>("TestClassA");
	factory.registerClass<TestClassB>("TestClassB");

	auto a = factory.create("TestClassA", "abc");
	EXPECT_NE(nullptr, a);
	EXPECT_EQ("abc", a->stringValue);
	EXPECT_EQ("TestClassA", a->className);

	auto b = factory.create("TestClassB", "cde");
	EXPECT_NE(nullptr, b);
	EXPECT_EQ("cde", b->stringValue);
	EXPECT_EQ("TestClassB", b->className);

	EXPECT_ANY_THROW(factory.create("xxx","xyz"));

}


}; // Framework
}; // SurgSim