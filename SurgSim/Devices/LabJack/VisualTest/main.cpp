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

#include <memory>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Devices/DeviceFilters/DeviceFilter.h"
#include "SurgSim/Devices/IdentityPoseDevice/IdentityPoseDevice.h"
#include "SurgSim/Devices/LabJack/LabJackDevice.h"
#include "SurgSim/Input/CommonDevice.h"
#include "SurgSim/Input/DeviceInterface.h"
#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Input/OutputProducerInterface.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Testing/VisualTestCommon/ToolSquareTest.h"

using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;
using SurgSim::Devices::IdentityPoseDevice;
using SurgSim::Devices::LabJackDevice;
using SurgSim::Input::DeviceInterface;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

class LabJackToPoseFilter : public SurgSim::Devices::DeviceFilter
{

public:
	LabJackToPoseFilter(const std::string& name, int firstTimerForQuadrature, int resetQuadrature, int plusX,
		int minusX, double translationPerUpdate, int positiveAnalogDifferential, int analogSingleEnded, int xOut,
		int loopbackOut) :
		SurgSim::Devices::DeviceFilter(name),
		m_pose(RigidTransform3d::Identity()),
		m_translationPerUpdate(translationPerUpdate),
		m_lineForPlusX(plusX),
		m_lineForMinusX(minusX),
		m_firstTimerForQuadrature(firstTimerForQuadrature),
		m_lineForResetQuadrature(resetQuadrature),
		m_analogInputDifferentialPositive(positiveAnalogDifferential),
		m_analogInputSingleEnded(analogSingleEnded),
		m_cachedOutputIndices(false),
		m_digitalInputPlusXIndex(-1),
		m_digitalInputMinusXIndex(-1),
		m_timerInputIndex(-1),
		m_analogInputDifferentialIndex(-1),
		m_analogInputSingleEndedIndex(-1),
		m_analogOutputIndex(-1),
		m_digitalOutputIndex(-1)
	{
		DataGroupBuilder inputBuilder;
		inputBuilder.addPose(SurgSim::DataStructures::Names::POSE);
		getInputData() = inputBuilder.createData();
		m_poseIndex = getInputData().poses().getIndex(SurgSim::DataStructures::Names::POSE);

		DataGroupBuilder outputBuilder;
		const std::string outputName =
			SurgSim::DataStructures::Names::ANALOG_OUTPUT_PREFIX + std::to_string(xOut);
		outputBuilder.addScalar(outputName);
		const std::string digitalOutputName =
			SurgSim::DataStructures::Names::DIGITAL_OUTPUT_PREFIX + std::to_string(loopbackOut);
		outputBuilder.addBoolean(digitalOutputName);
		outputBuilder.addScalar(SurgSim::DataStructures::Names::TIMER_OUTPUT_PREFIX +
			std::to_string(m_firstTimerForQuadrature));
		m_outputData = outputBuilder.createData();
		m_analogOutputIndex = m_outputData.scalars().getIndex(outputName);
		m_digitalOutputIndex = m_outputData.booleans().getIndex(digitalOutputName);
	}
	
	void initializeInput(const std::string& device, const DataGroup& inputData) override
	{
		m_digitalInputPlusXIndex = inputData.booleans().getIndex(SurgSim::DataStructures::Names::DIGITAL_INPUT_PREFIX +
			std::to_string(m_lineForPlusX));
		m_digitalInputMinusXIndex = inputData.booleans().getIndex(SurgSim::DataStructures::Names::DIGITAL_INPUT_PREFIX +
			std::to_string(m_lineForMinusX));
		m_timerInputIndex = inputData.scalars().getIndex(SurgSim::DataStructures::Names::TIMER_INPUT_PREFIX +
			std::to_string(m_firstTimerForQuadrature));
		m_analogInputDifferentialIndex =
			inputData.scalars().getIndex(SurgSim::DataStructures::Names::ANALOG_INPUT_PREFIX +
			std::to_string(m_analogInputDifferentialPositive));
		m_analogInputSingleEndedIndex =
			inputData.scalars().getIndex(SurgSim::DataStructures::Names::ANALOG_INPUT_PREFIX +
			std::to_string(m_analogInputSingleEnded));

		filterInput(device, inputData, &getInputData());
	}

	void filterInput(const std::string& device, const DataGroup& dataToFilter, DataGroup* result) override
	{
		m_lastInputData = dataToFilter;
		// Turn LabJack inputs into a pose so it can control the sphere.
		if (m_digitalInputPlusXIndex >= 0)
		{
			bool value;
			if (dataToFilter.booleans().get(m_digitalInputPlusXIndex, &value))
			{
				// If the device passed us this line's input, and the input is high...
				if (value)
				{
					m_pose.translation() += Vector3d::UnitX() * m_translationPerUpdate;
				}
			}
		}

		if (m_digitalInputMinusXIndex >= 0)
		{
			bool value;
			if (dataToFilter.booleans().get(m_digitalInputMinusXIndex, &value))
			{
				// If the device passed us this line's input, and the input is high...
				if (value)
				{
					m_pose.translation() -= Vector3d::UnitX() * m_translationPerUpdate;
				}
			}
		}

		if (m_timerInputIndex >= 0)
		{
			double value;
			// For quadrature inputs, we only need the input value of the first of the timers.
			if (dataToFilter.scalars().get(m_timerInputIndex, &value))
			{
				m_pose.translation()[1] = value * m_translationPerUpdate;
			}
		}

		const double rotationScaling = 0.0001 * 180.0 / M_PI;
		if (m_analogInputDifferentialIndex >= 0)
		{
			double value;
			if (dataToFilter.scalars().get(m_analogInputDifferentialIndex, &value))
			{
				m_pose.rotate(SurgSim::Math::makeRotationQuaternion(value * rotationScaling, Vector3d::UnitX().eval()));
			}
		}

		if (m_analogInputSingleEndedIndex >= 0)
		{
			double value;
			if (dataToFilter.scalars().get(m_analogInputSingleEndedIndex, &value))
			{
				m_pose.rotate(SurgSim::Math::makeRotationQuaternion(value * rotationScaling, Vector3d::UnitY().eval()));
			}
		}

		result->poses().set(m_poseIndex, m_pose);
	}

	void filterOutput(const std::string& device, const DataGroup& dataToFilter, DataGroup* result) override
	{
		*result = m_outputData;
		const double xScaling = 100.0;
		const double x = std::min(5.0, std::abs(m_pose.translation().x() * xScaling));
		result->scalars().set(m_analogOutputIndex, x);

		bool value;
		if (m_lastInputData.booleans().get(m_digitalInputMinusXIndex, &value))
		{
			result->booleans().set(m_digitalOutputIndex, value);
		}
		else
		{
			result->booleans().reset(m_digitalOutputIndex);
		}

		bool resetQuadrature = false;
		if ((m_lastInputData.booleans().get(SurgSim::DataStructures::Names::DIGITAL_INPUT_PREFIX +
			std::to_string(m_lineForResetQuadrature), &resetQuadrature)) &&
			resetQuadrature)
		{
			result->scalars().set(SurgSim::DataStructures::Names::TIMER_OUTPUT_PREFIX +
				std::to_string(m_firstTimerForQuadrature), 0.0);
		}
		else
		{
			result->scalars().reset(SurgSim::DataStructures::Names::TIMER_OUTPUT_PREFIX +
				std::to_string(m_firstTimerForQuadrature));
		}
	}

private:
	DataGroup m_outputData;
	DataGroup m_lastInputData;

	RigidTransform3d m_pose;
	double m_translationPerUpdate;

	int m_lineForPlusX;
	int m_lineForMinusX;
	int m_firstTimerForQuadrature;
	int m_lineForResetQuadrature;
	int m_analogInputDifferentialPositive;
	int m_analogInputDifferentialNegative;
	int m_analogInputSingleEnded;

	bool m_cachedOutputIndices;

	int m_poseIndex;
	int m_digitalInputPlusXIndex;
	int m_digitalInputMinusXIndex;
	int m_timerInputIndex;
	int m_analogInputDifferentialIndex;
	int m_analogInputSingleEndedIndex;
	int m_analogOutputIndex;
	int m_digitalOutputIndex;
};

int main(int argc, char** argv)
{
	std::shared_ptr<LabJackDevice> toolDevice = std::make_shared<LabJackDevice>("LabJackDevice");
	toolDevice->setAddress(""); // Get the first-found of the specified type and connection.

	const int plusX = SurgSim::Devices::LabJack::FIO0;
	toolDevice->enableDigitalInput(plusX);
	const int minusX = SurgSim::Devices::LabJack::FIO1;
	toolDevice->enableDigitalInput(minusX);

	const int loopbackOut = SurgSim::Devices::LabJack::FIO2;
	toolDevice->enableDigitalOutput(loopbackOut);

	const int offset = 4;
	toolDevice->setTimerCounterPinOffset(offset); // the U3 requires the offset to be 4+.

	const int firstTimerForQuadrature = SurgSim::Devices::LabJack::TIMER0;
	toolDevice->enableTimer(firstTimerForQuadrature, SurgSim::Devices::LabJack::TIMERMODE_QUADRATURE);
	toolDevice->enableTimer(firstTimerForQuadrature + 1, SurgSim::Devices::LabJack::TIMERMODE_QUADRATURE);

	const int resetQuadrature = SurgSim::Devices::LabJack::FIO3;
	toolDevice->enableDigitalInput(resetQuadrature);

	const int singleEndedAnalog = SurgSim::Devices::LabJack::AIN1;
	toolDevice->enableAnalogInput(singleEndedAnalog, SurgSim::Devices::LabJack::Range::RANGE_10);

	const int positiveAnalogDifferential = SurgSim::Devices::LabJack::AIN2;
	const int negativeAnalogDifferential = SurgSim::Devices::LabJack::AIN3;
	toolDevice->enableAnalogInput(positiveAnalogDifferential, SurgSim::Devices::LabJack::Range::RANGE_10,
		negativeAnalogDifferential);

	const int xOut = SurgSim::Devices::LabJack::DAC1;
	toolDevice->enableAnalogOutput(xOut);

	const double translationPerUpdate = 0.001; // Millimeter per update.
	auto filter = std::make_shared<LabJackToPoseFilter>("LabJack to Pose filter", firstTimerForQuadrature,
		resetQuadrature, plusX, minusX, translationPerUpdate, positiveAnalogDifferential, singleEndedAnalog,
		xOut, loopbackOut);
	toolDevice->setOutputProducer(filter);
	toolDevice->addInputConsumer(filter);

	if (toolDevice->initialize())
	{
		// The square is controlled by a second device.  For a simple test, we're using an IdentityPoseDevice--
		// a pretend device that doesn't actually move.
		std::shared_ptr<DeviceInterface> squareDevice = std::make_shared<IdentityPoseDevice>("IdentityPoseDevice");

		std::string text = "Set FIO" + std::to_string(plusX);
		text += " low to move the sphere tool in positive x-direction.  ";
		text += "Set FIO" + std::to_string(minusX);
		text += " low to move in negative x, and that input will be output to FIO";
		text += std::to_string(loopbackOut) + " (a loopback).  ";

		text += "DAC" + std::to_string(xOut);
		text += " will provide a voltage proportional to the absolute value of the x-position, up to 5v.  ";

		text += "Spin a quadrature encoder attached to FIO" + std::to_string(firstTimerForQuadrature + offset);
		text += " and FIO" + std::to_string(firstTimerForQuadrature + offset + 1);
		text += " to move the sphere +/- y-direction.  Set FIO" + std::to_string(resetQuadrature);
		text += " high to reset the quadrature reading.  ";

		text += "Provide a differential analog input to AIN" + std::to_string(positiveAnalogDifferential);
		text += " and AIN" + std::to_string(negativeAnalogDifferential) + " to spin about the red axis.  ";

		text += "Provide a single-ended analog input to AIN" + std::to_string(singleEndedAnalog);
		text += " to spin about the green axis.";

		runToolSquareTest(filter, squareDevice,
			//2345678901234567890123456789012345678901234567890123456789012345678901234567890
			text.c_str());
	}
	else
	{
		std::cout << std::endl << "Error initializing tool." << std::endl;
	}

	std::cout << std::endl << "Exiting." << std::endl;
	// Cleanup and shutdown will happen automatically as objects go out of scope.

	return 0;
}
