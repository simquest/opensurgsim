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

#ifndef SURGSIM_DEVICES_MULTIAXIS_WIN32_WDKHIDDEVICEHANDLE_H
#define SURGSIM_DEVICES_MULTIAXIS_WIN32_WDKHIDDEVICEHANDLE_H

#include <string>
#include <memory>
#include <array>
#include <vector>

#include "SurgSim/Devices/MultiAxis/SystemInputDeviceHandle.h"


// The following structure is defined by the Windows HID API, but we don't want to include the whole thing here.
struct _HIDP_CAPS;


namespace SurgSim
{
namespace Framework
{
class Logger;
};  // namespace Framework

namespace Devices
{

/// Access to an input/HID device using the HID API from the Windows Driver Kit.
/// \sa	SystemInputDeviceHandle
class WdkHidDeviceHandle : public SystemInputDeviceHandle
{
public:
	/// Destructor.
	~WdkHidDeviceHandle();

	/// Enumerates input devices.
	/// \param logger	The logger to be used during enumeration.
	/// \return	A list of device paths.
	static std::vector<std::string> enumeratePaths(SurgSim::Framework::Logger* logger);

	/// Opens the given path and creates an access wrapper for the device.
	/// \param	path	Full pathname for the device.
	/// \param logger	The logger to be used by the device.
	/// \return	The created device object, or an empty unique_ptr on failure.
	static std::unique_ptr<WdkHidDeviceHandle> open(const std::string& path,
		std::shared_ptr<SurgSim::Framework::Logger> logger);

	std::string getDeviceName() const override;

	bool getDeviceIds(int* vendorId, int* productId) const override;

	bool hasTranslationAndRotationAxes() const override;

	bool updateStates(AxisStates* axisStates, ButtonStates* buttonStates, bool* updated) override;

	void prepareForShutdown() override;

private:
	/// Constructor.
	/// Cannot be called directly.
	/// \sa open
	explicit WdkHidDeviceHandle(std::shared_ptr<SurgSim::Framework::Logger>&& logger);

	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	WdkHidDeviceHandle(const WdkHidDeviceHandle& other) /*= delete*/;
	WdkHidDeviceHandle& operator=(const WdkHidDeviceHandle& other) /*= delete*/;

	/// Gets the device capabilities.
	/// \param [out] capabilities	The capabilities data for the device.
	/// \return	true if it succeeds.
	bool getCapabilities(struct _HIDP_CAPS* capabilities) const;

	/// Starts an asynchronous read from the device.
	/// Updates the internal flags to indicate the state of the device I/O.
	/// \return	true if an asynchronous read has been started, or the read has already completed synchronously.
	bool startAsynchronousRead();

	/// Checks if an asynchronous read from the device has completed.
	/// Updates the internal flags to indicate the state of the device I/O.
	/// \param [out] numBytesRead	If the function returns true, the number of bytes read from the device.
	/// \return true if data has been received; false if the asynchronous read is still pending or an error has
	/// 	occurred.
	bool finishAsynchronousRead(size_t* numBytesRead);

	/// Cancels an asynchronous read from the device.
	/// Should be executed in the context of the thread that called startAsynchronousRead.
	void cancelAsynchronousRead();

	/// Decode the raw state update data received from the device.
	/// \param rawData	Raw state update data.
	/// \param rawDataSize	Size of the raw state update data.
	/// \param [in,out] axisStates	The states for each axis of the device.
	/// \param [in,out] buttonStates	The states for each device button.
	/// \param [out] updated True if any states were actually updated.  (Note that even if this value is true, the
	/// 	states may not have changed value; one or more states could have been updated to the same value.)
	void decodeStateUpdates(const unsigned char* rawData, size_t rawDataSize,
		AxisStates* axisStates, ButtonStates* buttonStates, bool* updated);


	struct State;
	std::unique_ptr<State> m_state;
};

};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_MULTIAXIS_WIN32_WDKHIDDEVICEHANDLE_H
