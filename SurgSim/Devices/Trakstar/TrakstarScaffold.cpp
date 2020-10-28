// This file is a part of the OpenSurgSim project.
// Copyright 2020, SimQuest Solutions Inc.
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

#include "SurgSim/Devices/Trakstar/TrakstarScaffold.h"

#include <algorithm>
#include <memory>

#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>

#include <ATC3DG.h>

#include "SurgSim/Devices/Trakstar/TrakstarDevice.h"
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Barrier.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SharedInstance.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::DataGroupBuilder;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

namespace 
{

using RECORD_TYPE = DOUBLE_POSITION_MATRIX_TIME_Q_RECORD;
static DATA_FORMAT_TYPE FORMAT_TYPE = DOUBLE_POSITION_MATRIX_TIME_Q;
static const int MAX_NUMBER_SENSORS = 8 * 4;

std::string getError(int error)
{
	char buffer[1024];
	char *pBuffer = &buffer[0];
	GetErrorText(error, pBuffer, sizeof(buffer), SIMPLE_MESSAGE);
	return std::string(buffer, strlen(buffer));
}
}

namespace SurgSim
{
namespace Devices
{

struct TrakstarScaffold::DeviceData
{
	/// Constructor
	/// \param device Device to be wrapped
	/// \param sensor The sensor identifier
	explicit DeviceData(TrakstarDevice* device, USHORT id) :
		deviceObject(device),
		sensorID(id)
	{
	}

	~DeviceData()
	{
	}
	
	/// The corresponding device object.
	SurgSim::Devices::TrakstarDevice* const deviceObject;
	/// The sensor ID.
	USHORT sensorID;
	/// Sensor configuration.
	SENSOR_CONFIGURATION sensorConfiguration;

private:
	DeviceData(const DeviceData&) = delete;
	DeviceData& operator=(const DeviceData&) = delete;
};

struct TrakstarScaffold::StateData
{
public:
	/// Initialize the state.
	StateData()
	{
	}
	
	/// The known devices.
	std::vector<std::unique_ptr<TrakstarScaffold::DeviceData>> registeredDevices;

	/// The mutex that protects the collection of known devices.
	boost::mutex mutex;

	/// The trakSTAR system configuration.
	SYSTEM_CONFIGURATION system;
	/// The transmitter configuration.
	TRANSMITTER_CONFIGURATION transmitter;
	/// The update rate requested for the system.
	DataStructures::OptionalValue<double> updateRate;
	/// The latest data records for all sensors.
	RECORD_TYPE records[MAX_NUMBER_SENSORS];

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	StateData(const StateData&) /*= delete*/;
	StateData& operator=(const StateData&) /*= delete*/;
};


TrakstarScaffold::TrakstarScaffold() :
	BasicThread("Devices/Trakstar"),
	m_state(new StateData)
{
	SURGSIM_LOG_DEBUG(m_logger) << "Shared scaffold created.";
	setRate(240.0); // The default GetSynchcronousRecord update rate is 80 measurements * 3 report rate = 240Hz
}

TrakstarScaffold::~TrakstarScaffold()
{
	if (isRunning())
	{
		stop();
	}

	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);

		if (!m_state->registeredDevices.empty())
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Trakstar: Destroying scaffold while devices are active!?!";
			m_state->registeredDevices.clear();
		}
	}

	if (isInitialized())
	{
		SHORT id = -1;
		SURGSIM_LOG_IF(!isSuccess(SetSystemParameter(SELECT_TRANSMITTER, &id, sizeof(id))), m_logger, SEVERE) <<
			"Finalizing Trakstar SDK failed.";
	}
	CloseBIRDSystem();
	SURGSIM_LOG_DEBUG(m_logger) << "Trakstar: Shared scaffold destroyed.";
}

bool TrakstarScaffold::isSuccess(int error)
{
	if (error != BIRD_ERROR_SUCCESS)
	{
		SURGSIM_LOG_SEVERE(m_logger) << getError(error);
		return false;
	}
	return true;
}

bool TrakstarScaffold::registerDevice(TrakstarDevice* device)
{
	const std::string name = device->getName();
	const unsigned short id = device->getSensorId();
	SURGSIM_LOG_DEBUG(m_logger) <<
		"Attempting to register TrakStar device, " << name << ", with sensorId, " << device->getSensorId() << ".";
	
	bool returnVal = true;
	std::unique_ptr<DeviceData> info(new DeviceData(device, id));
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	// Make sure the object is unique.
	auto sameObject = std::find_if(m_state->registeredDevices.cbegin(), m_state->registeredDevices.cend(),
		[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
	SURGSIM_ASSERT(sameObject == m_state->registeredDevices.end()) << "Trakstar: Tried to register a device" <<
		" which is already registered!";

	// Make sure the name is unique.
	auto sameName = std::find_if(m_state->registeredDevices.cbegin(), m_state->registeredDevices.cend(),
		[&name](const std::unique_ptr<DeviceData>& info) { return info->deviceObject->getName() == name; });
	SURGSIM_ASSERT(sameName == m_state->registeredDevices.end()) << "Trakstar: Tried to register a device" <<
		" when the same name, " << name << ", is already present!";

	// Make sure the sensor ID is unique.
	auto sameId = std::find_if(m_state->registeredDevices.cbegin(), m_state->registeredDevices.cend(),
		[&id](const std::unique_ptr<DeviceData>& info) { return info->deviceObject->getSensorId() == id; });
	SURGSIM_ASSERT(sameId == m_state->registeredDevices.end()) << "Trakstar: Tried to register a device" <<
		" when the same sensorId, " << id << ", is already present!";

	if (!isInitialized())
	{
		m_barrier.reset(new Framework::Barrier(2));
		this->start(m_barrier, true);
		m_barrier->wait(true);
		SURGSIM_LOG_INFO(m_logger) << "thread initialized.";
		if (isInitialized())
		{
			// If the thread is not at the barrier when setSynchronous is called, waiting may hang.
			boost::this_thread::sleep(boost::posix_time::milliseconds(200));
			this->setSynchronous(false);
			m_barrier->wait(true);
			SURGSIM_LOG_INFO(m_logger) << "thread doStartup() succeeded";
		}
	}

	if (isSuccess(GetSensorConfiguration(id, &info->sensorConfiguration)))
	{
		if (!info->sensorConfiguration.attached)
		{
			SURGSIM_LOG_SEVERE(m_logger) <<
				"sensorId " << id << " is not attached. Cannot use for device '" << name << "'.";
			returnVal = false;
		}
	}
	else
	{
		SURGSIM_LOG_SEVERE(m_logger) <<
			"Failed to get sensor configuration for device '" << name << "' with sensorId, " << id << ".";
		returnVal = false;
	}

	if (returnVal)
	{
		auto sensorRate = device->getOptionalUpdateRate();
		if (m_state->updateRate.hasValue())
		{
			SURGSIM_ASSERT(!sensorRate.hasValue() || (sensorRate.getValue() == m_state->updateRate.getValue())) <<
				"Two TrakstarDevices attempted to set different update rates.";
		}
		else
		{
			if (sensorRate.hasValue())
			{
				double rate = sensorRate.getValue();
				m_state->updateRate.setValue(rate);
				setRate(rate);
				double measurementRate = rate / 3.0; // See GetSynchronousRecord() (aka STREAM mode) and ReportRate.
				if (!isSuccess(SetSystemParameter(MEASUREMENT_RATE, &measurementRate, sizeof(measurementRate))))
				{
					SURGSIM_LOG_SEVERE(m_logger) << "Failed to set SDK measurement rate.";
					returnVal = false;
				}
			}
		}
	}

	if (returnVal)
	{
		m_state->registeredDevices.emplace_back(std::move(info));
		SURGSIM_LOG_INFO(m_logger) << "Device " << name << " initialized.";
	}
	else
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Device " << name << " failed initialization.";
	}
	return returnVal;
}

bool TrakstarScaffold::unregisterDevice(const TrakstarDevice* const device)
{
	bool found = false;
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);
		auto matching = std::find_if(m_state->registeredDevices.begin(), m_state->registeredDevices.end(),
			[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });

		if (matching != m_state->registeredDevices.end())
		{
			m_state->registeredDevices.erase(matching);
			// the iterator is now invalid but that's OK
			found = true;
			SURGSIM_LOG_INFO(m_logger) << "Device " << device->getName() << " unregistered.";
		}
	}
	SURGSIM_LOG_IF(!found, m_logger, SEVERE) << "Attempted to release a non-registered device " << device->getName();
	return found;
}

bool TrakstarScaffold::doInitialize()
{
	SURGSIM_ASSERT(!isInitialized()) << "Trakstar API already initialized.";

	bool returnVal = isSuccess(InitializeBIRDSystem());
	if (returnVal)
	{
		returnVal = isSuccess(GetBIRDSystemConfiguration(&m_state->system));
		SURGSIM_ASSERT(m_state->system.numberSensors <= MAX_NUMBER_SENSORS) << "Connected system can handle " <<
			m_state->system.numberSensors << " sensors, but the scaffold can only handle " << MAX_NUMBER_SENSORS << ".";
	}

	if (returnVal)
	{
		BOOL isMetric = true;
		if (!isSuccess(SetSystemParameter(METRIC, &isMetric, sizeof(isMetric))))
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Failed to set the measurements to metric.";
			returnVal = false;
		}
	}

	if (returnVal)
	{
		returnVal = false;
		bool hasFoundTransmitter = false;
		for (SHORT id = 0; id < m_state->system.numberTransmitters; id++)
		{
			GetTransmitterConfiguration((USHORT)(id), &m_state->transmitter);
			if (m_state->transmitter.attached)
			{
				hasFoundTransmitter = true;
				returnVal = isSuccess(SetSystemParameter(SELECT_TRANSMITTER, &id, sizeof(id)));
				break;
			}
		}
		if (hasFoundTransmitter)
		{
			SURGSIM_LOG_DEBUG(m_logger) << "Found an attached transmitter.";
		}
		else
		{
			SURGSIM_LOG_SEVERE(m_logger) << "None of the " << m_state->system.numberTransmitters <<
				" transmitter ports have an attached transmitter.";
		}
	}

	for (int i = 0; i < m_state->system.numberSensors; i++)
	{
		returnVal = returnVal && isSuccess(SetSensorParameter(i, DATA_FORMAT, &FORMAT_TYPE, sizeof(FORMAT_TYPE)));
	}

	SURGSIM_LOG_INFO(m_logger) << __func__ << " returning with value: " << (returnVal ? "true" : "false");
	return returnVal;
}

bool TrakstarScaffold::doStartUp()
{
	return true;
}

bool TrakstarScaffold::doUpdate(double dt)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);
	int lengthOfRecordData = (int)(m_state->system.numberSensors * sizeof(m_state->records[0]));
	if (isSuccess(GetSynchronousRecord(-1, &m_state->records[0], lengthOfRecordData)))
	{
		for (auto& device : m_state->registeredDevices)
		{
			updateDevice(device.get());
		}
	}
	else
	{
		for (auto& device : m_state->registeredDevices)
		{
			DataStructures::DataGroup& inputData = device->deviceObject->getInputData();
			inputData.poses().reset(DataStructures::Names::POSE);
			inputData.scalars().reset("time");
			inputData.scalars().reset("quality");
			device->deviceObject->pushInput();
		}
	}
	return true;
}

void TrakstarScaffold::updateDevice(TrakstarScaffold::DeviceData* info)
{
	auto status = GetSensorStatus(info->sensorID);
	SURGSIM_LOG_IF(status != 0, m_logger, WARNING) << "Sensor status: " << status;

	const auto& record = m_state->records[info->sensorID];
	RigidTransform3d pose;
	pose.linear() << record.s[0][0], record.s[1][0], record.s[2][0],
		record.s[0][1], record.s[1][1], record.s[2][1],
		record.s[0][2], record.s[1][2], record.s[2][2];
	// Convert millimeter to meter
	pose.translation() = Vector3d(record.x / 1000.0, record.y / 1000.0, record.z / 1000.0);

	DataStructures::DataGroup& inputData = info->deviceObject->getInputData();
	inputData.poses().set(DataStructures::Names::POSE, pose);
	inputData.scalars().set("time", record.time);
	inputData.scalars().set("quality", record.quality);
	info->deviceObject->pushInput();
}

SurgSim::DataStructures::DataGroup TrakstarScaffold::buildDeviceInputData()
{
	DataGroupBuilder builder;
	builder.addPose(DataStructures::Names::POSE);
	builder.addScalar("time");
	builder.addScalar("quality");
	return builder.createData();
}

std::shared_ptr<TrakstarScaffold> TrakstarScaffold::getOrCreateSharedInstance()
{
	static SurgSim::Framework::SharedInstance<TrakstarScaffold> sharedInstance;
	return sharedInstance.get();
}

};  // namespace Devices
};  // namespace SurgSim
