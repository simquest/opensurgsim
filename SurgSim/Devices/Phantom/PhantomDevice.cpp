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

#include "SurgSim/Devices/Phantom/PhantomDevice.h"

#include <iostream>
#include <iomanip>

#include <HD/hd.h>

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Matrix.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Framework/Log.h>
#include <SurgSim/Devices/Phantom/PhantomManager.h>
#include <SurgSim/Input/DataGroup.h>
#include <SurgSim/Input/DataGroupBuilder.h>

using SurgSim::Math::Vector3d;
using SurgSim::Math::Matrix44d;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::RigidTransform3d;

using SurgSim::Input::DataGroup;
using SurgSim::Input::DataGroupBuilder;


namespace SurgSim
{
namespace Device
{


struct PhantomDevice::State
{
	/// Initialize the state.
	State() : hHD(HD_INVALID_HANDLE)
	{
		for (int i = 0;  i < 3;  ++i)
		{
			positionBuffer[i] = 0;
		}
		for (int i = 0;  i < 16;  ++i)
		{
			transformBuffer[i] = (i % 5) ? 0 : 1;  // initialize to identity matrix
		}
		buttonsBuffer = 0;


		for (int i = 0;  i < 3;  ++i)
		{
			forceBuffer[i] = 0;
		}
	}

	/// The device handle.
	HHD hHD;

	/// The raw position read from the device.
	double positionBuffer[3];
	/// The raw pose transform read from the device.
	double transformBuffer[16];
	/// The raw button state read from the device.
	int buttonsBuffer;
	/// The raw force to be written to the device.
	double forceBuffer[3];
};

PhantomDevice::PhantomDevice(const PhantomManager& manager, const std::string& uniqueName,
                             const std::string& initializationName) :
	SurgSim::Input::CommonInputDevice(uniqueName, buildInputData()),
	m_logger(manager.getLogger()),
	m_initializationName(initializationName),
	m_messageLabel("Device " + uniqueName + ": "),
	m_state(new State)
{
}

PhantomDevice::~PhantomDevice()
{
	finalize();  // it's OK if we finalized already
	delete m_state;
	m_state = 0;
}

bool PhantomDevice::initialize()
{
	HHD hHD = HD_INVALID_HANDLE;
	if (m_initializationName.length() > 0)
	{
		hHD = hdInitDevice(m_initializationName.c_str());
	}
	else
	{
		hHD = hdInitDevice(HD_DEFAULT_DEVICE);
	}

	if (checkForFatalError("Failed to initialize"))
	{
		// HDAPI error message already logged
		SURGSIM_LOG_INFO(m_logger) << std::endl << "  OpenHaptics device name: '" << m_initializationName << "'" << std::endl;
		return false;
	}
	else if (hHD == HD_INVALID_HANDLE)
	{
		SURGSIM_LOG_SEVERE(m_logger) << m_messageLabel << "Failed to initialize" << std::endl <<
		                             "  Error details: unknown (HDAPI returned an invalid handle)" << std::endl <<
		                             "  OpenHaptics device name: '" << m_initializationName << "'" << std::endl;
		return false;
	}

	// Enable forces.
	hdMakeCurrentDevice(hHD);
	hdEnable(HD_FORCE_OUTPUT);
	checkForFatalError("Couldn't enable forces");

	m_state->hHD = hHD;

	SURGSIM_LOG_INFO(m_logger) << m_messageLabel << "Device initialized." << std::endl <<
	                           "  OpenHaptics device name: '" << m_initializationName << "'" << std::endl;

	return true;
}

bool PhantomDevice::finalize()
{
	HHD hHD = m_state->hHD;
	if (hHD == HD_INVALID_HANDLE)
	{
		return false;
	}

	hdDisableDevice(hHD);
	m_state->hHD = HD_INVALID_HANDLE;
	return true;
}

bool PhantomDevice::update()
{
	// TODO(bert): this code should cache the access indices.

	{
		// Use Eigen::Map to make the raw HDAPI output values look like Eigen data types
		Eigen::Map<Vector3d> force(m_state->forceBuffer);

		Vector3d forceValue;
		if (getOutputData().isValid() && getOutputData().vectors().get("force", forceValue))
		{
			force = forceValue;
		}
		else
		{
			force.setZero();
		}
	}

	hdBeginFrame(m_state->hHD);

	hdGetDoublev(HD_CURRENT_POSITION, m_state->positionBuffer);
	hdGetDoublev(HD_CURRENT_TRANSFORM, m_state->transformBuffer);
	hdGetIntegerv(HD_CURRENT_BUTTONS, &(m_state->buttonsBuffer));

	hdSetDoublev(HD_CURRENT_FORCE, m_state->forceBuffer);
	//hdSetDoublev(HD_CURRENT_TORQUE, m_state->torqueBuffer);

	hdEndFrame(m_state->hHD);

	bool fatalError = checkForFatalError("Error in device update");

	{
		// Use Eigen::Map to make the raw HDAPI output values look like Eigen data types
		Eigen::Map<Vector3d> position(m_state->positionBuffer);
		Eigen::Map<Matrix44d> transform(m_state->transformBuffer);

		RigidTransform3d pose;
		pose.linear() = transform.block<3,3>(0,0);
		pose.translation() = position;

		getInputData().poses().put("pose", pose);
		getInputData().booleans().put("button0", (m_state->buttonsBuffer & HD_DEVICE_BUTTON_1) != 0);
		getInputData().booleans().put("button1", (m_state->buttonsBuffer & HD_DEVICE_BUTTON_2) != 0);
		getInputData().booleans().put("button2", (m_state->buttonsBuffer & HD_DEVICE_BUTTON_3) != 0);
		getInputData().booleans().put("button3", (m_state->buttonsBuffer & HD_DEVICE_BUTTON_4) != 0);
	}

	return !fatalError;
}

DataGroup PhantomDevice::buildInputData()
{
	DataGroupBuilder builder;
	builder.addPose("pose");
	builder.addBoolean("button0");
	builder.addBoolean("button1");
	builder.addBoolean("button2");
	builder.addBoolean("button3");
	return builder.createData();
}


bool PhantomDevice::checkForFatalError(const char* message)
{
	return checkForFatalError(m_logger, m_messageLabel.c_str(), message);
}

bool PhantomDevice::checkForFatalError(const std::shared_ptr<SurgSim::Framework::Logger>& logger,
                                       const char* prefix, const char* message)
{
	HDErrorInfo error = hdGetError();
	if (error.errorCode == HD_SUCCESS)
	{
		return false;
	}

	// The HD API maintains an error stack, so in theory there could be more than one error pending.
	// We do head recursion to get them all in the correct order.
	bool anotherFatalError = checkForFatalError(logger, prefix, message);

	bool isFatal = ((error.errorCode != HD_WARM_MOTORS) &&
	                (error.errorCode != HD_EXCEEDED_MAX_FORCE) &&
	                (error.errorCode != HD_EXCEEDED_MAX_FORCE_IMPULSE) &&
	                (error.errorCode != HD_EXCEEDED_MAX_VELOCITY) &&
	                (error.errorCode != HD_FORCE_ERROR));

	SURGSIM_LOG_SEVERE(logger) << prefix << message << std::endl <<
	                           "  Error text: '" << hdGetErrorString(error.errorCode) << "'" << std::endl <<
	                           "  Error code: 0x" << std::hex << std::setw(4) << std::setfill('0') << error.errorCode <<
	                           " (internal: " << std::dec << error.internalErrorCode << ")" << std::endl;

	return (isFatal || anotherFatalError);
}

};  // namespace Device
};  // namespace SurgSim
