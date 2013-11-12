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
#include <Surgsim/Framework/Accessible.h>
#include <functional>
#include <boost/any.hpp>

class TestClass : public SurgSim::Framework::Accessible
{
public:
	TestClass()
	{
		using namespace std::placeholders;
		addGetter("a", std::bind(&TestClass::getA, this));
		// addSetter("a", boost::bind(&TestClass::setA, this, boost::bind(SurgSim::Framework::Converter<int>(),_1)));
		addSetter("a", std::bind(&TestClass::setA, this, std::bind(SurgSim::Framework::convert<int>,_1)));
	}
	int a;

	int getA() { return a;}
	void setA(int val) { a = val;}
};

namespace SurgSim
{
namespace Framework
{

TEST(AccessibleTests, GetterTest)
{
	TestClass t;
	t.a = 5;

	EXPECT_EQ(5, boost::any_cast<int>(t.getValue("a")));
}

TEST(AccessibleTests, SetterTest)
{
	TestClass t;
	t.a = 0;

	t.setValue("a", 4);
	EXPECT_EQ(4, t.getA());
}

TEST(AccessibleTests, TransferTest)
{
	TestClass a,b;
	a.a = 100;
	b.a = 0;

	b.setValue("a",a.getValue("a"));

	EXPECT_EQ(a.a, b.a);
}

}; // namespace Framework
}; // namespace SurgSim
