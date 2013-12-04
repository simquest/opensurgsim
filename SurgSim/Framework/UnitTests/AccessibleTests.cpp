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
#include <SurgSim/Framework/Accessible.h>
#include <functional>
#include <boost/any.hpp>
#include <SurgSim/Math/Matrix.h>

#include <memory>

class TestClass : public SurgSim::Framework::Accessible
{
public:
	TestClass()
	{
		setGetter("a", std::bind(&TestClass::getA, this));
		setSetter("a", std::bind(&TestClass::setA, this, std::bind(SurgSim::Framework::convert<int>,
						std::placeholders::_1)));

		SURGSIM_ADD_RW_PROPERTY(TestClass, double, b, getB, setB);
		SURGSIM_ADD_RW_PROPERTY(TestClass, std::shared_ptr<int>, c, getC, setC);

		c = std::make_shared<int>(4);
	}
	int a;
	double b;

	std::shared_ptr<int> c;

	int getA() { return a; }
	void setA(int val) { a = val; }

	double getB() { return b; }
	void setB(double val) { b = val; }

	std::shared_ptr<int> getC() { return c; }
	void setC(std::shared_ptr<int> val) { c = val; }
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

	EXPECT_ANY_THROW(t.getValue("xxx"));
}

TEST(AccessibleTests, SetterTest)
{
	TestClass t;
	t.a = 0;

	t.setValue("a", 4);
	EXPECT_EQ(4, t.getA());
	EXPECT_ANY_THROW(t.setValue("xxxx",666.66));
}

TEST(AccessibleTests, TransferTest)
{
	TestClass a,b;
	a.a = 100;
	b.a = 0;

	b.setValue("a",a.getValue("a"));

	EXPECT_EQ(a.a, b.a);
}

TEST(AccessibleTests, MacroTest)
{
	TestClass a;
	a.b = 100.0;

	EXPECT_EQ(a.b, boost::any_cast<double>(a.getValue("b")));
	a.setValue("b",50.0);
	EXPECT_EQ(50.0, a.b);
}

TEST(AccessibleTest, TemplateFunction)
{
	TestClass a;
	a.a = 10;
	a.b = 100.0;

	// Parameter Deduction
	int aDotA = 123;
	double aDotB = 456;
	EXPECT_TRUE(a.getValue("a", &aDotA));
	EXPECT_EQ(10, aDotA);
	EXPECT_TRUE(a.getValue("b", &aDotB));
	EXPECT_EQ(100.0, aDotB);

	EXPECT_FALSE(a.getValue("xxxx", &aDotA));

	double* noValue = nullptr;

	EXPECT_FALSE(a.getValue("a", noValue));
}

TEST(AccessibleTest, SharedPointerTest)
{
	TestClass a;
	std::shared_ptr<int> x = std::make_shared<int>(5);
	std::shared_ptr<int> y;

	y = boost::any_cast<std::shared_ptr<int>>(a.getValue("c"));
	EXPECT_EQ(4,*y);

	a.setValue("c",x);
	y = boost::any_cast<std::shared_ptr<int>>(a.getValue("c"));
	EXPECT_EQ(5,*y);
}

TEST(AccessibleTest, ConvertDoubleToFloat)
{
	// Values don't matter only care for them to be filled
	SurgSim::Math::Matrix44d sourceDouble = SurgSim::Math::Matrix44d::Random();
	SurgSim::Math::Matrix44f sourceFloat = SurgSim::Math::Matrix44f::Random();

	SurgSim::Math::Matrix44f target;

	EXPECT_NO_THROW({target = convert<SurgSim::Math::Matrix44f>(sourceDouble);});
	EXPECT_NO_THROW({target = convert<SurgSim::Math::Matrix44f>(sourceFloat);});
}

}; // namespace Framework
}; // namespace SurgSim
