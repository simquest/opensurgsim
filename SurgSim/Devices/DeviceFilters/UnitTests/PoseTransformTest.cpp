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

/// \file
/// Tests for the PoseTransform class.

#include <memory>
#include <string>
#include <gtest/gtest.h>
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Devices/DeviceFilters/PoseTransform.h"
#include "SurgSim/Input/CommonDevice.h"
#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Input/OutputProducerInterface.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/RigidTransform.h"

using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;
using SurgSim::Device::PoseTransform;
using SurgSim::Input::CommonDevice;
using SurgSim::Input::InputConsumerInterface;
using SurgSim::Input::OutputProducerInterface;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

const double errorEpsilon = 1e-9;

class TestInputOutputDevice : public CommonDevice
{
public:
	TestInputOutputDevice(const std::string& name) :
		CommonDevice(name)
	{
	}

	void doPushInput()
	{
		pushInput();
	}

	void doPullOutput()
	{
		pullOutput();
	}

	const SurgSim::DataStructures::DataGroup& doGetOutputData() const
	{
		return getOutputData();
	}

	SurgSim::DataStructures::DataGroup& doGetInitialInputData()
	{
		return getInitialInputData();
	}

	SurgSim::DataStructures::DataGroup& doGetInputData()
	{
		return getInputData();
	}

	virtual bool initialize()
	{
		return true;
	}

	virtual bool finalize()
	{
		return true;
	}
};

class TestInputConsumerInterface : public InputConsumerInterface
{
public:
	TestInputConsumerInterface()
	{
	}

	virtual void handleInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData) override
	{
		m_data = inputData;
	}

	virtual void initializeInput(const std::string& device,
		const SurgSim::DataStructures::DataGroup& inputData) override
	{
		handleInput(device, inputData);
	}

	DataGroup m_data;
};

class TestOutputProducerInterface : public OutputProducerInterface
{
public:
	TestOutputProducerInterface()
	{
	}

	virtual bool requestOutput(const std::string& device, SurgSim::DataStructures::DataGroup* outputData) override
	{
		*outputData = m_data;
		return true;
	}

	DataGroup m_data;
};

TEST(PoseTransformDeviceFilterTest, AsInputConsumer)
{
	auto poseTransformer = std::make_shared<PoseTransform>("PoseTransformFilter");
	ASSERT_TRUE(poseTransformer->initialize());
	auto device = std::make_shared<TestInputOutputDevice>("InputDevice");
	auto inputConsumer = std::make_shared<TestInputConsumerInterface>();

	DataGroupBuilder builder;
	builder.addPose("pose");
	builder.addPose("anotherPose");

	DataGroup data = builder.createData();
	RigidTransform3d initialPose;
	initialPose.matrix() << 0.529453820664377, 0.830923707192042, 0.171010071662834, 2.0,
		-0.785101696592397, 0.403558881227842, 0.469846310392954, 3.0, 
		0.32139380484327, -0.383022221559489, 0.866025403784439, 4.0,
		0.0, 0.0, 0.0, 1.0; // Euler z-x-z, 20/30/40 degrees

	RigidTransform3d anotherInitialPose;
	anotherInitialPose.matrix() << 0.657741706348699, 0.748222844697849, -0.086824088833465, -3.5,
		-0.681235546590068, 0.54171630256426, -0.492403876506104, -8.0,
		-0.321393804843269, 0.383022221559489, 0.866025403784439, 7.0,
		0.0, 0.0, 0.0, 1.0; // Euler z-x-z, 10/-30/40 degrees

	data.poses().set("pose", initialPose);
	data.poses().set("anotherPose", anotherInitialPose);

	// Normally the initial input data would be set by the constructor or scaffold.
	device->doGetInitialInputData() = data;

	// The device's input data is consumed by the filter, and the filter's input data is consumed by the InputConsumer.
	// InitializeInput is called in addInputConsumer, passing the initial input down the chain.
	device->addInputConsumer(poseTransformer);
	poseTransformer->addInputConsumer(inputConsumer);

	RigidTransform3d actualPose;
	ASSERT_TRUE(inputConsumer->m_data.poses().get("pose", &actualPose));
	// The PoseTransform should be using identity transform and scaling if they have not been set.
	EXPECT_TRUE(actualPose.isApprox(initialPose, errorEpsilon));

	RigidTransform3d transform;
	transform.matrix() << 0.10130572780775, -0.941511110779744, -0.321393804843269, 18.3,
		0.824533332339233, -0.10130572780775, 0.556670399226419, -12.6,
		-0.556670399226419, -0.32139380484327, 0.766044443118978, 1.0,
		0.0, 0.0, 0.0, 1.0; // Euler z-x-z, -30/40/-60 degrees

	poseTransformer->setTransform(transform);
	poseTransformer->setTranslationScale(3.7);

	device->doGetInputData() = data; // Normally the scaffold would set and push the input data.
	device->doPushInput();

	// The "pose" data should have its translation scaled and be pre-transformed.
	RigidTransform3d actualPoseAfterTransform;
	ASSERT_TRUE(inputConsumer->m_data.poses().get("pose", &actualPoseAfterTransform));
	RigidTransform3d expectedPose;
	expectedPose.matrix() << 0.68952469728513432, -0.17267687049352703, -0.70337642143478485, 3.8422607444418091,
		0.69499803949953043, 0.43102433404654489, 0.57549608908448779, 0.61577498919529994,
		0.20379748998859773, -0.88566400054213212, 0.41721200991588686, 4.6506255701250767,
		0.0, 0.0, 0.0, 1.0; // Calculated via numpy.
	EXPECT_TRUE(actualPoseAfterTransform.isApprox(expectedPose, errorEpsilon));

	// PoseTransform should pass through all other data unchanged.
	RigidTransform3d anotherPoseAfterTransform;
	ASSERT_TRUE(inputConsumer->m_data.poses().get("anotherPose", &anotherPoseAfterTransform));
	EXPECT_TRUE(anotherPoseAfterTransform.isApprox(anotherInitialPose, errorEpsilon));
}

TEST(PoseTransformDeviceFilterTest, AsOutputProducer)
{
	auto poseTransformer = std::make_shared<PoseTransform>("PoseTransformFilter");
	ASSERT_TRUE(poseTransformer->initialize());
	auto device = std::make_shared<TestInputOutputDevice>("OutputDevice");
	auto outputProducer = std::make_shared<TestOutputProducerInterface>();

	DataGroupBuilder builder;
	builder.addPose("pose");
	builder.addPose("anotherPose");

	DataGroup data = builder.createData();
	RigidTransform3d initialPose;
	initialPose.matrix() << -0.014297271682553, 0.696079050903939, 0.717822779601698, -179.3,
		-0.956807691132494, -0.218000810711764, 0.192340034102939, 223.92354,
		0.290369816289747, -0.684068418670008, 0.669130606358858, 8.7,
		0.0, 0.0, 0.0, 1.0; // Euler z-x-z, 75/48/23 degrees

	RigidTransform3d anotherInitialPose;
	anotherInitialPose.matrix() << -0.534717382486846, 0.445878658326802, -0.717822779601697, 0.7,
		-0.463195024434534, -0.865133331123227, -0.192340034102939, -33.3,
		-0.7067727288213, 0.229644380354319, 0.669130606358858, 2.1,
		0.0, 0.0, 0.0, 1.0; // Euler z-x-z, 75/-48/72 degrees

	data.poses().set("pose", initialPose);
	data.poses().set("anotherPose", anotherInitialPose);

	// Normally the data would be set by a behavior.
	outputProducer->m_data = data;

	// The OutputProducer sends data out to the filter, which sends data out to the device.
	poseTransformer->setOutputProducer(outputProducer);
	device->setOutputProducer(poseTransformer);

	device->doPullOutput(); // Normally the scaffold would pull the data;

	RigidTransform3d actualPose;
	ASSERT_TRUE(device->doGetOutputData().poses().get("pose", &actualPose));
	// The PoseTransform should be using identity transform and scaling if they have not been set.
	EXPECT_TRUE(actualPose.isApprox(initialPose, errorEpsilon));

	RigidTransform3d transform;
	transform.matrix() << 0.76461304237996, 0.265602364171189, 0.587215701058083, -0.95,
		0.3146967650075, 0.64126665474163, -0.699816421363698, -1.1,
		-0.562434744229297, 0.719883644530944, 0.4067366430758, 4.2,
		0.0, 0.0, 0.0, 1.0; // Euler z-x-z, -40/-66/38 degrees

	poseTransformer->setTransform(transform);
	poseTransformer->setTranslationScale(-31.18);

	device->doPullOutput(); // Normally the scaffold would pull the data;

	// The "pose" data should have its translation scaled and be pre-transformed.
	RigidTransform3d actualPoseAfterTransform;
	ASSERT_TRUE(device->doGetOutputData().poses().get("pose", &actualPoseAfterTransform));
	RigidTransform3d expectedPose;
	expectedPose.matrix() << -0.094552549982206385, 0.072633874091332429, 0.99286662529583847, 2259.965438390881,
		-0.82127373817845517, 0.55797948761443317, -0.11903082953554855, -2529.2107740265933,
		-0.56264488113751565, -0.82666995332842363, 0.0068939099016622207, -8276.648397154011,
		0.0, 0.0, 0.0, 1.0; // Calculated via numpy.
	EXPECT_TRUE(actualPoseAfterTransform.isApprox(expectedPose, errorEpsilon));

	// PoseTransform should pass through all other data unchanged.
	RigidTransform3d anotherPoseAfterTransform;
	ASSERT_TRUE(outputProducer->m_data.poses().get("anotherPose", &anotherPoseAfterTransform));
	EXPECT_TRUE(anotherPoseAfterTransform.isApprox(anotherInitialPose, errorEpsilon));
}

TEST(PoseTransformDeviceFilterTest, BothInputAndOutput)
{
	auto poseTransformer = std::make_shared<PoseTransform>("PoseTransformFilter");
	ASSERT_TRUE(poseTransformer->initialize());
	auto device = std::make_shared<TestInputOutputDevice>("InputDevice"); // two separate devices could be used
	auto inputConsumer = std::make_shared<TestInputConsumerInterface>();
	auto outputProducer = std::make_shared<TestOutputProducerInterface>();

	DataGroupBuilder inputBuilder;
	inputBuilder.addPose("pose");
	inputBuilder.addVector("inVector");

	DataGroup inputData = inputBuilder.createData();
	RigidTransform3d initialInputPose;
	initialInputPose.matrix() << 1.0, 0.0, 0.0, 2.0,
							0.0, 0.0, 1.0, 3.0,
							0.0, -1.0, 0.0, 4.0,
							0.0, 0.0, 0.0, 1.0; // 90 degree rotation about x-axis

	Vector3d initialInputVector = Vector3d::UnitY();

	inputData.poses().set("pose", initialInputPose);
	inputData.vectors().set("inVector", initialInputVector);
	// Normally the initial input data would be set by the constructor or scaffold.
	device->doGetInitialInputData() = inputData;

	DataGroupBuilder outputBuilder;
	outputBuilder.addPose("pose");
	outputBuilder.addVector("outVector");

	DataGroup outputData = outputBuilder.createData();
	RigidTransform3d initialOutputPose;
	initialOutputPose.matrix() << 0.0, 0.0, -1.0, 5.0,
								0.0, 1.0, 0.0, 6.0,
								1.0, 0.0, 0.0, 7.0,
								0.0, 0.0, 0.0, 1.0; // 90 degree rotation about y-axis

	Vector3d initialOutputVector = Vector3d::UnitZ();

	outputData.poses().set("pose", initialOutputPose);
	outputData.vectors().set("outVector", initialOutputVector);
	// Normally the data would be set by a behavior.
	outputProducer->m_data = outputData;

	// The device's input data is consumed by the filter, and the filter's input data is consumed by the InputConsumer.
	// InitializeInput is called in addInputConsumer, passing the initial input down the chain.
	device->addInputConsumer(poseTransformer);
	poseTransformer->addInputConsumer(inputConsumer);

	// The OutputProducer sends data out to the filter, which sends data out to the device.
	poseTransformer->setOutputProducer(outputProducer);
	device->setOutputProducer(poseTransformer);

	// The PoseTransform should keep the input data and output data separate.
	device->doPullOutput();
	device->doGetInputData() = inputData; // Normally the scaffold would set and push the input data.
	device->doPushInput();

	RigidTransform3d actualInputPose;
	ASSERT_TRUE(inputConsumer->m_data.poses().get("pose", &actualInputPose));
	EXPECT_TRUE(actualInputPose.isApprox(initialInputPose, errorEpsilon));
	Vector3d actualInputVector;
	ASSERT_TRUE(inputConsumer->m_data.vectors().get("inVector", &actualInputVector));
	EXPECT_TRUE(actualInputVector.isApprox(initialInputVector, errorEpsilon));

	RigidTransform3d actualOutputPose;
	ASSERT_TRUE(device->doGetOutputData().poses().get("pose", &actualOutputPose));
	EXPECT_TRUE(actualOutputPose.isApprox(initialOutputPose, errorEpsilon));
	Vector3d actualOutputVector;
	ASSERT_TRUE(device->doGetOutputData().vectors().get("outVector", &actualOutputVector));
	EXPECT_TRUE(actualOutputVector.isApprox(initialOutputVector, errorEpsilon));
}