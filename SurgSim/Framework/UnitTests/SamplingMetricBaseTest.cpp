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
#include <deque>

#include "SurgSim/Framework/SamplingMetricBase.h"
#include "SurgSim/Framework/UnitTests/MockObjects.h"

TEST(SamplingMetricBaseTest, SamplingMetricBaseInitTest)
{
	std::shared_ptr<MockSamplingMetric> mockMetric(new MockSamplingMetric("Test Metric"));

	auto samples = mockMetric->getMeasurementValues();

	// Test initialization settings.
	EXPECT_EQ("Test Metric", mockMetric->getName());
	EXPECT_EQ(0, samples.size());
	EXPECT_EQ(0.0, mockMetric->getElapsedTime());
	EXPECT_EQ(SurgSim::Framework::MANAGER_TYPE_BEHAVIOR, mockMetric->getTargetManagerType());
	EXPECT_LT(0, mockMetric->getMaxNumberOfMeasurements());
	EXPECT_EQ(0, mockMetric->getCurrentNumberOfMeasurements());
}

TEST(SamplingMetricBaseTest, SetGetTests)
{
	std::shared_ptr<MockSamplingMetric> mockMetric(new MockSamplingMetric("Test Metric"));

	auto samples = mockMetric->getMeasurementValues();

	// Test sets and gets.
	mockMetric->setName("Test2");
	mockMetric->setMaxNumberOfMeasurements(5);
	mockMetric->setTargetManagerType(SurgSim::Framework::MANAGER_TYPE_PHYSICS);

	EXPECT_EQ("Test2", mockMetric->getName());
	EXPECT_EQ(5, mockMetric->getMaxNumberOfMeasurements());
	EXPECT_EQ(SurgSim::Framework::MANAGER_TYPE_PHYSICS, mockMetric->getTargetManagerType());

}

TEST(SamplingMetricBaseTest, AbleToPerformMeasurementsTests)
{
	std::shared_ptr<MockSamplingMetric> mockMetric(new MockSamplingMetric("Test Metric", true, 1.0));

	// Make 10 out
	mockMetric->setMaxNumberOfMeasurements(5);
	for (int ctr = 0; ctr < 10; ++ctr)
	{
		mockMetric->update((double) ctr);
	}
	auto samples = mockMetric->getMeasurementValues();

	EXPECT_EQ(5, mockMetric->getCurrentNumberOfMeasurements());
	EXPECT_EQ(5, samples.size());
	EXPECT_EQ(0.0, mockMetric->getElapsedTime());

	// When we check the results, our deque holds 5 places. We skip the
	// first 5 because they already rolled off the end. The metric value
	// starts at one
	auto sampleIterator = samples.begin();
	for (int ctr = 5;
		 sampleIterator != samples.end();
		 ++ctr, ++sampleIterator)
	{
		EXPECT_EQ((double) ctr, sampleIterator->first);
		EXPECT_EQ((double)(ctr + 2), sampleIterator->second);
	}
}

TEST(SamplingMetricBaseTest, UnableToPerformMeasurementsTests)
{
	std::shared_ptr<MockSamplingMetric> mockMetric(new MockSamplingMetric("Test Metric", false, 1.0));
	double accumulatedTime = 0.0;
	// Make 10 out
	mockMetric->setMaxNumberOfMeasurements(5);

	for (int ctr = 0; ctr < 10; ++ctr)
	{
		mockMetric->update((double) ctr);
		accumulatedTime += ctr;
	}
	auto samples = mockMetric->getMeasurementValues();

	EXPECT_EQ(0, mockMetric->getCurrentNumberOfMeasurements());
	EXPECT_EQ(0, samples.size());
	EXPECT_EQ(accumulatedTime, mockMetric->getElapsedTime());
}
