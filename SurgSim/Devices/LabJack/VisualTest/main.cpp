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
using SurgSim::Device::IdentityPoseDevice;
using SurgSim::Device::LabJackDevice;
using SurgSim::Input::DeviceInterface;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

class LabJackToPoseFilter : public SurgSim::Input::CommonDevice,
	public SurgSim::Input::InputConsumerInterface, public SurgSim::Input::OutputProducerInterface
{

public:
	LabJackToPoseFilter(const std::string& name, int firstTimerForQuadrature,
		int lineForPlusX, int lineForMinusX, double translationPerUpdate, int positiveAnalogDifferential,
		int analogSingleEnded, int rotationOut) :
		SurgSim::Input::CommonDevice(name),
		m_digitalInputPlusXIndex(-1),
		m_digitalInputMinusXIndex(-1),
		m_timerInputIndex(-1),
		m_pose(RigidTransform3d::Identity()),
		m_lineForPlusX(lineForPlusX),
		m_lineForMinusX(lineForMinusX),
		m_firstTimerForQuadrature(firstTimerForQuadrature),
		m_translationPerUpdate(translationPerUpdate),
		m_analogInputDifferentialPositive(positiveAnalogDifferential),
		m_analogInputSingleEnded(analogSingleEnded),
		m_analogInputDifferentialPositiveIndex(-1),
		m_analogInputSingleEndedIndex(-1),
		m_analogOutput(rotationOut),
		m_analogOutputIndex(-1),
		m_cachedOutputIndices(false)
	{
		DataGroupBuilder inputBuilder;
		inputBuilder.addPose(SurgSim::DataStructures::Names::POSE);
		getInputData() = inputBuilder.createData();
		m_poseIndex = getInputData().poses().getIndex(SurgSim::DataStructures::Names::POSE);

		DataGroupBuilder outputBuilder;
		const std::string outputName =
			SurgSim::DataStructures::Names::ANALOG_OUTPUT_PREFIX + std::to_string(m_analogOutput);
		outputBuilder.addScalar(outputName);
		m_outputData = outputBuilder.createData();
		m_analogOutputIndex = m_outputData.scalars().getIndex(outputName);
	}

	virtual ~LabJackToPoseFilter()
	{
		finalize();
	}

	bool initialize()
	{
		return true;
	}

	bool finalize()
	{
		return true;
	}

	void initializeInput(const std::string& device, const DataGroup& inputData)
	{
		m_digitalInputPlusXIndex = inputData.scalars().getIndex(SurgSim::DataStructures::Names::DIGITAL_INPUT_PREFIX +
			std::to_string(m_lineForPlusX));
		m_digitalInputMinusXIndex = inputData.scalars().getIndex(SurgSim::DataStructures::Names::DIGITAL_INPUT_PREFIX +
			std::to_string(m_lineForMinusX));
		m_timerInputIndex = inputData.scalars().getIndex(SurgSim::DataStructures::Names::TIMER_INPUT_PREFIX +
			std::to_string(m_firstTimerForQuadrature));
		m_analogInputDifferentialPositiveIndex =
			inputData.scalars().getIndex(SurgSim::DataStructures::Names::ANALOG_INPUT_DIFFERENTIAL_PREFIX +
			std::to_string(m_analogInputDifferentialPositive));
		m_analogInputDifferentialNegativeIndex =
			inputData.scalars().getIndex(SurgSim::DataStructures::Names::ANALOG_INPUT_DIFFERENTIAL_PREFIX +
			std::to_string(m_analogInputDifferentialNegative));
		m_analogInputSingleEndedIndex =
			inputData.scalars().getIndex(SurgSim::DataStructures::Names::ANALOG_INPUT_SINGLE_ENDED_PREFIX +
			std::to_string(m_analogInputSingleEnded));

		inputFilter(inputData, &getInputData());
	}

	void handleInput(const std::string& device, const DataGroup& inputData)
	{
		inputFilter(inputData, &getInputData());
		pushInput();
	}

	bool requestOutput(const std::string& device, DataGroup* outputData)
	{
		bool state = pullOutput();
		if (state)
		{
			outputFilter(m_outputData, outputData);
		}
		return state;
	}

	void inputFilter(const DataGroup& dataToFilter, DataGroup* result)
	{
		// Turn LabJack inputs into a pose so it can control the sphere.
		if (m_digitalInputPlusXIndex >= 0)
		{
			double value;
			if (dataToFilter.scalars().get(m_digitalInputPlusXIndex, &value))
			{
				// If the device passed us this line's input, and the input is high...
				if (value > 0.5)
				{
					m_pose.translation() += Vector3d::UnitX() * m_translationPerUpdate;
				}
			}
		}

		if (m_digitalInputMinusXIndex >= 0)
		{
			double value;
			if (dataToFilter.scalars().get(m_digitalInputMinusXIndex, &value))
			{
				// If the device passed us this line's input, and the input is high...
				if (value > 0.5)
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
		if (m_analogInputDifferentialPositiveIndex >= 0)
		{
			double value;
			if (dataToFilter.scalars().get(m_analogInputDifferentialPositiveIndex, &value))
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

	void outputFilter(const DataGroup& dataToFilter, DataGroup* result)
	{
		*result = dataToFilter;
		const double xScaling = 100.0;
		const double x = std::min(5.0, std::abs(m_pose.translation().x() * xScaling));
		result->scalars().set(m_analogOutputIndex, x);
	}

private:
	int m_digitalInputPlusXIndex;
	int m_digitalInputMinusXIndex;
	int m_digitalOutputsIndex;
	int m_timerInputIndex;
	int m_poseIndex;
	RigidTransform3d m_pose;
	int m_lineForPlusX;
	int m_lineForMinusX;
	int m_firstTimerForQuadrature;
	double m_translationPerUpdate;
	int m_analogInputDifferentialPositive;
	int m_analogInputDifferentialNegative;
	int m_analogInputSingleEnded;
	int m_analogInputDifferentialPositiveIndex;
	int m_analogInputDifferentialNegativeIndex;
	int m_analogInputSingleEndedIndex;
	int m_analogOutput;
	int m_analogOutputIndex;
	bool m_cachedOutputIndices;
	DataGroup m_outputData;
};

int main(int argc, char** argv)
{
	std::shared_ptr<LabJackDevice> toolDevice = std::make_shared<LabJackDevice>("LabJackDevice");
	toolDevice->setAddress(""); // Get the first-found of the specified type and connection.

	const int lineForPlusX = 0;
	const int lineForMinusX = 1;
	std::unordered_set<int> digitalInputChannels;
	digitalInputChannels.insert(lineForPlusX);
	digitalInputChannels.insert(lineForMinusX);
	toolDevice->setDigitalInputChannels(digitalInputChannels);

	const int offset = 4;
	toolDevice->setTimerCounterPinOffset(offset); // the U3 requires the offset to be 4+.

	const int firstTimerForQuadrature = 0;
	std::unordered_map<int,SurgSim::Device::LabJackTimerMode> timers;
	timers[firstTimerForQuadrature] = SurgSim::Device::LABJACKTIMERMODE_QUAD;
	timers[firstTimerForQuadrature + 1] = SurgSim::Device::LABJACKTIMERMODE_QUAD;
	toolDevice->setTimers(timers);

	std::unordered_map<int, SurgSim::Device::LabJackAnalogInputRange> analogInputsSingleEnded;
	const int singleEndedAnalog = 1;
	analogInputsSingleEnded[singleEndedAnalog] = SurgSim::Device::LabJackAnalogInputRange::LABJACKANALOGINPUTRANGE_10;
	toolDevice->setAnalogInputsSingleEnded(analogInputsSingleEnded);

	std::unordered_map<int, std::pair<int, SurgSim::Device::LabJackAnalogInputRange>> analogInputsDifferential;
	const int positiveAnalogDifferential = 2;
	const int negativeAnalogDifferential = 3;
	analogInputsDifferential[positiveAnalogDifferential] =
		std::pair<int, SurgSim::Device::LabJackAnalogInputRange>(negativeAnalogDifferential,
		SurgSim::Device::LabJackAnalogInputRange::LABJACKANALOGINPUTRANGE_10);
	toolDevice->setAnalogInputsDifferential(analogInputsDifferential);

	std::unordered_set<int> analogOutputs;
	const int rotationOut = 1;
	analogOutputs.insert(rotationOut);
	toolDevice->setAnalogOutputChannels(analogOutputs);

	if (toolDevice->initialize())
	{
		const double translationPerUpdate = 0.001; // Millimeter per update.
		auto filter = std::make_shared<LabJackToPoseFilter>("LabJack to Pose filter", firstTimerForQuadrature,
			lineForPlusX, lineForMinusX, translationPerUpdate, positiveAnalogDifferential, singleEndedAnalog,
			rotationOut);
		toolDevice->setOutputProducer(filter);
		toolDevice->addInputConsumer(filter);
		filter->initialize();

		// The square is controlled by a second device.  For a simple test, we're using an IdentityPoseDevice--
		// a pretend device that doesn't actually move.
		std::shared_ptr<DeviceInterface> squareDevice = std::make_shared<IdentityPoseDevice>("IdentityPoseDevice");

		std::string text = "Set FIO" + std::to_string(lineForPlusX);
		text += " low to move the sphere tool in positive x-direction.  ";
		text += "Set FIO" + std::to_string(lineForMinusX);
		text += " low to move in negative x.  ";

		text += "DAC" + std::to_string(rotationOut);
		text += " will provide a voltage proportional to the absolute value of the x-position, up to 5v.  ";

		text += "Spin a quadrature encoder attached to FIO" + std::to_string(firstTimerForQuadrature + offset);
		text += " and FIO" + std::to_string(firstTimerForQuadrature + offset + 1);
		text += " to move the sphere +/- y-direction.  ";

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
