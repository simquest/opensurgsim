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

#include <gtest/gtest.h>
#include <deque>

#include "SurgSim/Framework/SamplingMetricBase.h"
#include "SurgSim/Framework/UnitTests/MockObjects.h"


TEST(SamplingMetricBaseTest, SamplingMetricBaseInitTest)
{
	std::shared_ptr<MockSamplingMetric> mockMetric(std::make_shared<MockSamplingMetric>("Test Metric"));

	auto samples = mockMetric->getMeasurementValues();

	// Test initialization settings.
	EXPECT_EQ("Test Metric", mockMetric->getName());
	EXPECT_EQ(0u, samples.size());
	EXPECT_NEAR(0.0, mockMetric->getElapsedTime(), 1e-9);
	EXPECT_EQ(SurgSim::Framework::MANAGER_TYPE_BEHAVIOR, mockMetric->getTargetManagerType());
	EXPECT_LT(0u, mockMetric->getMaxNumberOfMeasurements());
	EXPECT_EQ(0u, mockMetric->getCurrentNumberOfMeasurements());
}

TEST(SamplingMetricBaseTest, SetGetTests)
{
	std::shared_ptr<MockSamplingMetric> mockMetric(std::make_shared<MockSamplingMetric>("Test Metric"));

	auto samples = mockMetric->getMeasurementValues();

	// Test sets and gets.
	mockMetric->setName("Test2");
	mockMetric->setMaxNumberOfMeasurements(5);
	mockMetric->setTargetManagerType(SurgSim::Framework::MANAGER_TYPE_PHYSICS);

	EXPECT_EQ("Test2", mockMetric->getName());
	EXPECT_EQ(5u, mockMetric->getMaxNumberOfMeasurements());
	EXPECT_EQ(SurgSim::Framework::MANAGER_TYPE_PHYSICS, mockMetric->getTargetManagerType());

}

TEST(SamplingMetricBaseTest, AbleToPerformMeasurementsTests)
{
	std::shared_ptr<MockSamplingMetric> mockMetric(std::make_shared<MockSamplingMetric>("Test Metric", true, 1.0));

	// Make 10 updates
	mockMetric->setMaxNumberOfMeasurements(5);
	for (int counter = 0; counter < 10; ++counter)
	{
		mockMetric->update(static_cast<double>(counter));
	}
	auto samples = mockMetric->getMeasurementValues();

	EXPECT_EQ(5u, mockMetric->getCurrentNumberOfMeasurements());
	EXPECT_EQ(5u, samples.size());
	EXPECT_NEAR(45.0, mockMetric->getElapsedTime(), 1e-9);

	// When we check the results, our deque holds 5 places. We skip the
	// first 5 because they already rolled off the end. The metric value
	// starts at one and accumulates. At position 5, we should already have
	// a value of 15 accumulated.
	int counter = 5;
	int accumulator = 15;

	for (auto sampleIterator = samples.begin();
		 sampleIterator != samples.end();
		 ++sampleIterator)
	{
		EXPECT_NEAR(static_cast<double>(accumulator), sampleIterator->first, 1e-9);
		EXPECT_NEAR(static_cast<double>(counter + 2), sampleIterator->second, 1e-9);
		++counter;
		accumulator += counter;
	}
}

TEST(SamplingMetricBaseTest, UnableToPerformMeasurementsTests)
{
	std::shared_ptr<MockSamplingMetric> mockMetric(std::make_shared<MockSamplingMetric>("Test Metric", false, 1.0));
	double accumulatedTime = 0.0;
	// Make 10 updates
	mockMetric->setMaxNumberOfMeasurements(5);

	for (int counter = 0; counter < 10; ++counter)
	{
		mockMetric->update(static_cast<double>(counter));
		accumulatedTime += counter;
	}
	auto samples = mockMetric->getMeasurementValues();

	EXPECT_EQ(0u, mockMetric->getCurrentNumberOfMeasurements());
	EXPECT_EQ(0u, samples.size());
	EXPECT_NEAR(accumulatedTime, mockMetric->getElapsedTime(), 1e-9);
}
