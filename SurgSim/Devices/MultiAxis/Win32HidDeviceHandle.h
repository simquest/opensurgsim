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

#ifndef SURGSIM_DEVICES_MULTIAXIS_WIN32HIDDEVICEHANDLE_H
#define SURGSIM_DEVICES_MULTIAXIS_WIN32HIDDEVICEHANDLE_H

#include <string>
#include <memory>
#include <array>
#include <vector>

#include <SurgSim/Devices/MultiAxis/SystemInputDeviceHandle.h>


// The following structure is defined by the Windows HID API, but we don't want to include the whole thing here.
struct _HIDP_CAPS;


namespace SurgSim
{
namespace Device
{

/// A wrapper for system-dependent access to a input/HID device.
class Win32HidDeviceHandle : public SystemInputDeviceHandle
{
public:
	/// Destructor.
	~Win32HidDeviceHandle();

	/// Enumerates input devices.
	/// \param logger	The logger to be used during enumeration.
	/// \return	A list of device paths.
	static std::vector<std::string> enumerate(SurgSim::Framework::Logger* logger);

	/// Opens the given path and creates an access wrapper for the device.
	/// \param	path	Full pathname for the device.
	/// \param logger	The logger to be used by the device.
	/// \return	The created device object, or an empty unique_ptr on failure.
	static std::unique_ptr<Win32HidDeviceHandle> open(const std::string& path,
		std::shared_ptr<SurgSim::Framework::Logger> logger);

	virtual std::string getDeviceName() const override;

	virtual bool getDeviceIds(int* vendorId, int* productId) const override;

	virtual bool hasTranslationAndRotationAxes() const override;

	virtual bool updateStates(AxisStates* axisStates, ButtonStates* buttonStates, bool* updated) override;

private:
	/// Constructor.
	/// Cannot be called directly; see open and enumerate.
	explicit Win32HidDeviceHandle(std::shared_ptr<SurgSim::Framework::Logger>&& logger);

	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	Win32HidDeviceHandle(const Win32HidDeviceHandle& other) /*= delete*/;
	Win32HidDeviceHandle& operator=(const Win32HidDeviceHandle& other) /*= delete*/;

	/// Gets the device capabilities.
	/// \param [out] capabilities	The capabilities data for the device.
	/// \return	true if it succeeds.
	bool getCapabilities(struct _HIDP_CAPS* capabilities) const;


	struct State;
	std::unique_ptr<State> m_state;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_MULTIAXIS_WIN32HIDDEVICEHANDLE_H
