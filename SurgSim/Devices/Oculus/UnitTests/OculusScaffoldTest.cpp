#include <memory>
#include <gtest/gtest.h>

#include "SurgSim/Devices/Oculus/OculusDevice.h"
#include "SurgSim/Devices/Oculus/OculusScaffold.h"

using SurgSim::Device::OculusDevice;
using SurgSim::Device::OculusScaffold;

TEST(OculusScaffoldTest, CreateAndDestroyScaffold)
{
	auto scaffold = OculusScaffold::getOrCreateSharedInstance();
	ASSERT_NE(nullptr, scaffold) << "The scaffold was not created!";

	std::weak_ptr<OculusScaffold> weakScaffold = scaffold;
	{
		std::shared_ptr<OculusScaffold> stillHaveScaffold = weakScaffold.lock();
		EXPECT_NE(nullptr, stillHaveScaffold) << "Unable to get scaffold from weak ref (while strong ref exists)";
		EXPECT_EQ(scaffold, stillHaveScaffold) << "Scaffold mismatch!";
	}
	{
		std::shared_ptr<OculusScaffold> sameScaffold = OculusScaffold::getOrCreateSharedInstance();
		EXPECT_NE(nullptr, sameScaffold) << "Unable to get scaffold from class";
		EXPECT_EQ(scaffold, sameScaffold) << "Scaffold mismatch!";
	}

	scaffold.reset();
	{
		std::shared_ptr<OculusScaffold> dontHaveScaffold = weakScaffold.lock();
		EXPECT_EQ(nullptr, dontHaveScaffold) << "Able to get scaffold from weak ref (with no strong ref)";
	}

	scaffold = OculusScaffold::getOrCreateSharedInstance();
	ASSERT_NE(nullptr, scaffold) << "The scaffold was not created the 2nd time!";
	std::weak_ptr<OculusScaffold> yetAnotherWeakScaffold = scaffold;
	{
		std::shared_ptr<OculusScaffold> stillHaveScaffold = yetAnotherWeakScaffold.lock();
		ASSERT_NE(nullptr, stillHaveScaffold) << "Unable to get scaffold from weak ref (while strong ref exists)";
		ASSERT_EQ(scaffold, stillHaveScaffold) << "Scaffold mismatch!";
	}
}

TEST(OculusScaffoldTest, ScaffoldLifeCycle)
{
	std::weak_ptr<OculusScaffold> lastScaffold;
	{
		std::shared_ptr<OculusScaffold> scaffold = OculusScaffold::getOrCreateSharedInstance();
		ASSERT_NE(nullptr, scaffold) << "The scaffold was not created!";
		lastScaffold = scaffold;
	}
	{
		std::shared_ptr<OculusScaffold> dontHaveScaffold = lastScaffold.lock();
		EXPECT_EQ(nullptr, dontHaveScaffold) << "Able to get scaffold from weak ref (with no strong ref)";
		lastScaffold.reset();
	}

	{
		std::shared_ptr<OculusDevice> device = std::make_shared<OculusDevice>("Oculus");
		ASSERT_NE(nullptr, device) << "Creation failed.  Is an Oculus device plugged in?";
		// note: the device is NOT initialized!
		{
			std::shared_ptr<OculusScaffold> scaffold = OculusScaffold::getOrCreateSharedInstance();
			EXPECT_NE(nullptr, scaffold) << "The scaffold was not retrieved!";
			lastScaffold = scaffold;  // save the scaffold for later

			std::shared_ptr<OculusScaffold> sameScaffold = lastScaffold.lock();
			EXPECT_NE(nullptr, sameScaffold);
			EXPECT_EQ(scaffold, sameScaffold);
		}
		// The device has not been initialized, so it should NOT be hanging on to the device!
		{
			std::shared_ptr<OculusScaffold> deadScaffold = lastScaffold.lock();
			EXPECT_EQ(nullptr, deadScaffold);
		}
		// the ("empty") device is about to get destroyed
	}

	{
		std::shared_ptr<OculusDevice> device = std::make_shared<OculusDevice>("Oculus");
		ASSERT_NE(nullptr, device) << "Device creation failed.";
		ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is an Oculus device plugged in?";
		{
			std::shared_ptr<OculusScaffold> scaffold = OculusScaffold::getOrCreateSharedInstance();
			EXPECT_NE(nullptr, scaffold) << "The scaffold was not retrieved!";
			lastScaffold = scaffold;  // save the scaffold for later

			std::shared_ptr<OculusScaffold> sameScaffold = lastScaffold.lock();
			EXPECT_NE(nullptr, sameScaffold);
			EXPECT_EQ(scaffold, sameScaffold);
		}
		// The same scaffold is supposed to still be around because of the device
		{
			std::shared_ptr<OculusScaffold> sameScaffold = lastScaffold.lock();
			EXPECT_NE(nullptr, sameScaffold);

			std::shared_ptr<OculusScaffold> scaffold = OculusScaffold::getOrCreateSharedInstance();
			EXPECT_NE(nullptr, scaffold) << "The scaffold was not retrieved!";
			EXPECT_EQ(sameScaffold, scaffold);
		}
		// the device and the scaffold are about to get destroyed
	}

	{
		std::shared_ptr<OculusScaffold> deadScaffold = lastScaffold.lock();
		EXPECT_EQ(nullptr, deadScaffold);
	}

	{
		auto device = std::make_shared<OculusDevice>("Oculus");
		ASSERT_NE(nullptr, device) << "Device creation failed.";
		ASSERT_TRUE(device->initialize()) << "Initialization failed.  Didn't this work a moment ago?";
		std::shared_ptr<OculusScaffold> scaffold = OculusScaffold::getOrCreateSharedInstance();
		EXPECT_NE(nullptr, scaffold) << "The scaffold was not retrieved!";

		std::shared_ptr<OculusScaffold> deadScaffold = lastScaffold.lock();
		EXPECT_EQ(nullptr, deadScaffold);
	}
}


TEST(OculusScaffoldTest, CreateDeviceSeveralTimes)
{
	std::weak_ptr<OculusScaffold> lastScaffold;

	for (size_t i = 0;  i < 6;  ++i)
	{
		SCOPED_TRACE(i);
		EXPECT_EQ(nullptr, lastScaffold.lock());
		auto device = std::make_shared<OculusDevice>("Oculus");
		ASSERT_NE(nullptr, device) << "Device creation failed.";
		ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is an Oculus device plugged in?";
		std::shared_ptr<OculusScaffold> scaffold = OculusScaffold::getOrCreateSharedInstance();
		ASSERT_NE(nullptr, scaffold) << "The scaffold was not retrieved!";
		lastScaffold = scaffold;
		// the device and the scaffold will be destroyed here
	}
}

TEST(OculusScaffoldTest, RegisterAndUnregisterDevice)
{
	std::shared_ptr<OculusScaffold> scaffold = OculusScaffold::getOrCreateSharedInstance();
	ASSERT_NE(nullptr, scaffold) << "The scaffold was not retrieved!";

	auto device = std::make_shared<OculusDevice>("Oculus");
	ASSERT_NE(nullptr, device) << "Device creation failed.";
	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is an Oculus device plugged in?";

	ASSERT_TRUE(device->finalize()) << "Finalization failed.";
	ASSERT_NE(nullptr, scaffold) << "The scaffold should NOT be destroyed!";

	ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is an Oculus device plugged in?";
	ASSERT_NE(nullptr, scaffold) << "The scaffold was not retrieved!";
}

TEST(OculusScaffoldTest, CreateDeviceSeveralTimesWithScaffoldRef)
{
	std::shared_ptr<OculusScaffold> lastScaffold;

	for (size_t i = 0;  i < 6;  ++i)
	{
		SCOPED_TRACE(i);
		auto device = std::make_shared<OculusDevice>("Oculus");
		ASSERT_NE(nullptr, device) << "Device creation failed.";
		ASSERT_TRUE(device->initialize()) << "Initialization failed.  Is an Oculus device plugged in?";
		std::shared_ptr<OculusScaffold> scaffold = OculusScaffold::getOrCreateSharedInstance();
		ASSERT_NE(nullptr, scaffold) << "The scaffold was not retrieved!";
		if (!lastScaffold)
		{
			lastScaffold = scaffold;
		}
		EXPECT_EQ(lastScaffold, scaffold);
		// the device will be destroyed here, but the scaffold stays around because we have a shared_ptr to it.
	}
}
