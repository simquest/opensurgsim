// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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
/// Tests for the CombiningOutputComponent class.

#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Input/CombiningOutputComponent.h"
#include "SurgSim/Input/CommonDevice.h"
#include "SurgSim/Input/InputManager.h"
#include "SurgSim/Input/OutputComponent.h"
#include "SurgSim/Math/Vector.h"

namespace
{
const double ERROR_EPSILON = 1e-7;

auto DO_NOTHING_FUNCTOR = [](const std::vector<std::shared_ptr<SurgSim::Input::OutputComponent>>&,
	SurgSim::DataStructures::DataGroup*) {return false; };

/// Device that exposes pullOutput and getOutputData.
class MockDevice : public SurgSim::Input::CommonDevice
{
public:

	MockDevice(const std::string& name) : SurgSim::Input::CommonDevice(name)
	{
	}

	~MockDevice()
	{
	}

	using CommonDevice::pullOutput;

	using CommonDevice::getOutputData;

	bool initialize() override
	{
		return true;
	}

	bool isInitialized() const override
	{
		return true;
	}

	bool finalize() override
	{
		return true;
	}
};

};

TEST(CombiningOutputComponentTest, NoOutputs)
{
	auto combiningOutputComponent = std::make_shared<SurgSim::Input::CombiningOutputComponent>("combiner");
	auto mockDevice = std::make_shared<MockDevice>("device");
	mockDevice->setOutputProducer(combiningOutputComponent);
	EXPECT_FALSE(mockDevice->pullOutput());
}

TEST(CombiningOutputComponentTest, DuplicateOutputs)
{
	auto combiningOutputComponent = std::make_shared<SurgSim::Input::CombiningOutputComponent>("combiner");
	auto mockDevice = std::make_shared<MockDevice>("device");
	mockDevice->setOutputProducer(combiningOutputComponent);

	std::vector<std::shared_ptr<SurgSim::Framework::Component>> outputs;
	auto output1 = std::make_shared<SurgSim::Input::OutputComponent>("output1");
	auto output2 = std::make_shared<SurgSim::Input::OutputComponent>("output2");
	outputs.push_back(output1);
	outputs.push_back(output2);
	outputs.push_back(output1);
	outputs.push_back(output2);
	outputs.push_back(output2);
	combiningOutputComponent->setOutputs(outputs);

	auto storedOutputs = combiningOutputComponent->getOutputs();
	ASSERT_EQ(2, storedOutputs.size());
	EXPECT_EQ(output1, storedOutputs[0]);
	EXPECT_EQ(output2, storedOutputs[1]);
}

TEST(CombiningOutputComponentTest, EmptyOutputs)
{
	auto combiningOutputComponent = std::make_shared<SurgSim::Input::CombiningOutputComponent>("combiner");
	auto mockDevice = std::make_shared<MockDevice>("device");
	mockDevice->setOutputProducer(combiningOutputComponent);

	std::vector<std::shared_ptr<SurgSim::Framework::Component>> outputs;
	outputs.push_back(std::make_shared<SurgSim::Input::OutputComponent>("output1"));
	outputs.push_back(std::make_shared<SurgSim::Input::OutputComponent>("output2"));
	combiningOutputComponent->setOutputs(outputs);

	ASSERT_EQ(outputs, combiningOutputComponent->getOutputs());
	EXPECT_FALSE(mockDevice->pullOutput());
}

TEST(CombiningOutputComponentTest, OneNonEmptyOutput)
{
	auto combiningOutputComponent = std::make_shared<SurgSim::Input::CombiningOutputComponent>("combiner");
	auto mockDevice = std::make_shared<MockDevice>("device");
	mockDevice->setOutputProducer(combiningOutputComponent);

	std::vector<std::shared_ptr<SurgSim::Framework::Component>> outputs;
	auto output = std::make_shared<SurgSim::Input::OutputComponent>("output1");
	outputs.push_back(output);
	combiningOutputComponent->setOutputs(outputs);

	SurgSim::DataStructures::DataGroupBuilder builder;
	builder.addVector(SurgSim::DataStructures::Names::FORCE);
	builder.addBoolean("extraData");

	SurgSim::DataStructures::DataGroup data = builder.createData();
	auto initialForce = SurgSim::Math::Vector3d(0.89, 0.0, -324.67);
	data.vectors().set(SurgSim::DataStructures::Names::FORCE, initialForce);
	data.booleans().set("extraData", true);

	output->setData(data);

	ASSERT_TRUE(mockDevice->pullOutput());
	SurgSim::DataStructures::DataGroup actualData = mockDevice->getOutputData();
	ASSERT_FALSE(actualData.isEmpty());

	SurgSim::Math::Vector3d actualForce;
	ASSERT_TRUE(actualData.vectors().get(SurgSim::DataStructures::Names::FORCE, &actualForce));
	EXPECT_TRUE(actualForce.isApprox(initialForce));
	EXPECT_FALSE(actualData.booleans().hasEntry("extraData"));

	EXPECT_ANY_THROW(combiningOutputComponent->setData(data));
}

TEST(CombiningOutputComponentTest, SetCombiner)
{
	auto combiningOutputComponent = std::make_shared<SurgSim::Input::CombiningOutputComponent>("combiner");
	combiningOutputComponent->setCombiner(DO_NOTHING_FUNCTOR);
	auto mockDevice = std::make_shared<MockDevice>("device");
	mockDevice->setOutputProducer(combiningOutputComponent);

	std::vector<std::shared_ptr<SurgSim::Framework::Component>> outputs;
	auto output1 = std::make_shared<SurgSim::Input::OutputComponent>("output1");
	outputs.push_back(output1);
	auto output2 = std::make_shared<SurgSim::Input::OutputComponent>("output2");
	outputs.push_back(output2);
	auto output3 = std::make_shared<SurgSim::Input::OutputComponent>("output3");
	outputs.push_back(output3);
	combiningOutputComponent->setOutputs(outputs);

	SurgSim::DataStructures::DataGroupBuilder builder;
	builder.addVector(SurgSim::DataStructures::Names::FORCE);

	SurgSim::DataStructures::DataGroup data = builder.createData();
	auto initialForce = SurgSim::Math::Vector3d(0.89, 0.0, -324.67);
	data.vectors().set(SurgSim::DataStructures::Names::FORCE, initialForce);

	output1->setData(data);
	output3->setData(data);

	EXPECT_FALSE(mockDevice->pullOutput());
}


TEST(CombiningOutputComponentTest, MultipleOutputs)
{
	auto combiningOutputComponent = std::make_shared<SurgSim::Input::CombiningOutputComponent>("combiner");
	auto mockDevice = std::make_shared<MockDevice>("device");
	mockDevice->setOutputProducer(combiningOutputComponent);

	std::vector<std::shared_ptr<SurgSim::Framework::Component>> outputs;
	auto output1 = std::make_shared<SurgSim::Input::OutputComponent>("output1");
	outputs.push_back(output1);
	auto output2 = std::make_shared<SurgSim::Input::OutputComponent>("output2");
	outputs.push_back(output2);
	auto output3 = std::make_shared<SurgSim::Input::OutputComponent>("output3");
	outputs.push_back(output3);
	combiningOutputComponent->setOutputs(outputs);

	SurgSim::DataStructures::DataGroupBuilder builder;
	builder.addVector(SurgSim::DataStructures::Names::FORCE);

	SurgSim::DataStructures::DataGroup data = builder.createData();
	auto initialForce = SurgSim::Math::Vector3d(0.89, 0.0, -324.67);
	data.vectors().set(SurgSim::DataStructures::Names::FORCE, initialForce);

	output1->setData(data);
	output3->setData(data);

	ASSERT_TRUE(mockDevice->pullOutput());
	SurgSim::DataStructures::DataGroup actualData = mockDevice->getOutputData();
	ASSERT_FALSE(actualData.isEmpty());

	SurgSim::Math::Vector3d actualForce;
	ASSERT_TRUE(actualData.vectors().get(SurgSim::DataStructures::Names::FORCE, &actualForce));
	EXPECT_TRUE(actualForce.isApprox(2.0 * initialForce));

	// Check subsequent calls to pullOutput will correctly handle non-asserting DataGroup assignment.
	EXPECT_NO_THROW(mockDevice->pullOutput());

	// Check first OutputComponent going away.
	outputs.clear();
	output1.reset();
	EXPECT_TRUE(mockDevice->pullOutput());
	EXPECT_EQ(2, combiningOutputComponent->getOutputs().size());
}


TEST(CombiningOutputComponentTest, Serialization)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	auto inputManager = std::make_shared<SurgSim::Input::InputManager>();
	runtime->addManager(inputManager);
	runtime->addSceneElements("CombiningOutputComponent.yaml");

	auto element = runtime->getScene()->getSceneElement("element");
	ASSERT_NE(element, nullptr);
	auto outputs = element->getComponents<SurgSim::Input::OutputComponent>();
	ASSERT_EQ(outputs.size(), 4);
	std::shared_ptr<SurgSim::Input::CombiningOutputComponent> combiningOutputComponent;
	for (size_t i = 0; i < outputs.size(); ++i)
	{
		combiningOutputComponent = std::dynamic_pointer_cast<SurgSim::Input::CombiningOutputComponent>(outputs[i]);
		if (combiningOutputComponent != nullptr)
		{
			outputs.erase(outputs.begin() + i);
			break;
		}
	}
	ASSERT_NE(combiningOutputComponent, nullptr);

	std::set<std::shared_ptr<SurgSim::Framework::Component>> componentsFromElement;
	componentsFromElement.insert(outputs.begin(), outputs.end());

	std::set<std::shared_ptr<SurgSim::Framework::Component>> componentsFromCombiner;
	auto actualOutputs = combiningOutputComponent->getOutputs();
	componentsFromCombiner.insert(actualOutputs.begin(), actualOutputs.end());
	ASSERT_EQ(componentsFromElement, componentsFromCombiner);
	EXPECT_EQ("output1", actualOutputs[0]->getName());
	EXPECT_EQ("output2", actualOutputs[1]->getName());
	EXPECT_EQ("output3", actualOutputs[2]->getName());

	auto mockDevice = std::make_shared<MockDevice>("device");
	inputManager->addDevice(mockDevice);
	
	SurgSim::DataStructures::DataGroupBuilder builder;
	builder.addVector(SurgSim::DataStructures::Names::FORCE);

	SurgSim::DataStructures::DataGroup data = builder.createData();
	auto initialForce = SurgSim::Math::Vector3d(0.89, 0.0, -324.67);
	data.vectors().set(SurgSim::DataStructures::Names::FORCE, initialForce);

	std::static_pointer_cast<SurgSim::Input::OutputComponent>(actualOutputs[0])->setData(data);
	std::static_pointer_cast<SurgSim::Input::OutputComponent>(actualOutputs[2])->setData(data);

	runtime->start(true);
	runtime->step();
	runtime->step();
	runtime->step();
	boost::this_thread::sleep(boost::posix_time::milliseconds(50));
	runtime->stop();

	ASSERT_TRUE(mockDevice->pullOutput());
	SurgSim::DataStructures::DataGroup actualData = mockDevice->getOutputData();
	ASSERT_FALSE(actualData.isEmpty());
	SurgSim::Math::Vector3d actualForce;
	ASSERT_TRUE(actualData.vectors().get(SurgSim::DataStructures::Names::FORCE, &actualForce));
	EXPECT_TRUE(actualForce.isApprox(2.0 * initialForce));

	YAML::Node node;
	EXPECT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*combiningOutputComponent));
	EXPECT_TRUE(node.IsMap());
	std::shared_ptr<SurgSim::Input::CombiningOutputComponent> newComponent;
	EXPECT_NO_THROW(newComponent = std::dynamic_pointer_cast<SurgSim::Input::CombiningOutputComponent>(
		node.as<std::shared_ptr<SurgSim::Framework::Component>>()));
	ASSERT_NE(newComponent, nullptr);
	auto newOutputs = newComponent->getValue<std::vector<std::shared_ptr<SurgSim::Framework::Component>>>("Outputs");
	for (auto newOutput : newOutputs)
	{
		EXPECT_NE(std::dynamic_pointer_cast<SurgSim::Input::OutputComponent>(newOutput), nullptr);
	}
}
