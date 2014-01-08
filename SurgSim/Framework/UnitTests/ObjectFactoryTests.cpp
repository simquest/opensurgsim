

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

	TestClass(std::string val) : stringValue(val)
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

	TestClassA(const std::string& val) : TestClass(val)
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

	TestClassB(const std::string& val) : TestClass(val)
	{
		className = "TestClassB";
	}
};

}


namespace SurgSim
{
namespace Framework
{

TEST(ObjectFactoryTest, Single)
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