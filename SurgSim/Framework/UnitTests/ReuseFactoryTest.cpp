#include <gtest/gtest.h>

#include <SurgSim/Framework/ReuseFactory.h>

using SurgSim::Framework::ReuseFactory;

class MockObject
{
public:
	MockObject() {};
	~MockObject() {};
};

class ReuseFactoryTest : public ::testing::Test
{
public:
	void SetUp()
	{
	}

	void TearDown()
	{
	}
};

TEST_F(ReuseFactoryTest, InitTest)
{
	ASSERT_NO_THROW({ReuseFactory<MockObject>();});
}

TEST_F(ReuseFactoryTest, GetNewTest)
{
	ReuseFactory<MockObject> objectFactory;

	std::shared_ptr<MockObject> object = objectFactory.getInstance();
	ASSERT_TRUE(object != nullptr) << "Object = " << object;
}

TEST_F(ReuseFactoryTest, DeleteTest)
{
	ReuseFactory<MockObject> objectFactory;

	std::weak_ptr<MockObject> weakPointer;
	{
		std::shared_ptr<MockObject> sharedPointer;
		{
			// Get a new object.
			std::shared_ptr<MockObject> object = objectFactory.getInstance();
			ASSERT_TRUE(object != nullptr) << "Object = " << object;

			weakPointer = object;
			ASSERT_TRUE(weakPointer.lock() != nullptr) << "Weak pointer to object = " << weakPointer.lock();

			// Make another shared pointer to the object.
			sharedPointer = object;
		}

		// The object should not have been deleted yet, as there is still a shared pointer to it.
		EXPECT_TRUE(weakPointer.lock() != nullptr) << "Weak pointer to object = " << weakPointer.lock();
	}

	// It should be deleted now.
	EXPECT_TRUE(weakPointer.lock() == nullptr) << "Weak pointer to object = " << weakPointer.lock();
}

TEST_F(ReuseFactoryTest, ReuseTest)
{
	ReuseFactory<MockObject> objectFactory;

	std::weak_ptr<MockObject> weakPointer;
	MockObject* pointer;

	// Get a new object
	{
		std::shared_ptr<MockObject> object = objectFactory.getInstance();
		ASSERT_TRUE(object != nullptr) << "Object = " << object;

		weakPointer = object;
		ASSERT_TRUE(weakPointer.lock() != nullptr) << "Weak pointer to object = " << weakPointer.lock();

		pointer = object.get();
	}

	// It should be deleted now.
	EXPECT_TRUE(weakPointer.lock() == nullptr) << "Weak pointer to object = " << weakPointer.lock();
	
	// Reuse the object that has been deleted.
	{
		std::shared_ptr<MockObject> object = objectFactory.getInstance();
		ASSERT_TRUE(object != nullptr) << "Object = " << object;

		EXPECT_TRUE(object.get() == pointer) << "Original object = " << pointer << ", New object = " << object.get();

		// Get another object, which should be new.
		{
			std::shared_ptr<MockObject> object = objectFactory.getInstance();
			ASSERT_TRUE(object != nullptr) << "Object = " << object;

			EXPECT_TRUE(object.get() != pointer ) << "Original object = " << pointer << ", New object = " << object.get();
		}
	}
}
