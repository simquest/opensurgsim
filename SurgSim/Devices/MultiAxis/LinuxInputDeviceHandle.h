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

#ifndef SURGSIM_DEVICES_MULTIAXIS_LINUXINPUTDEVICEHANDLE_H
#define SURGSIM_DEVICES_MULTIAXIS_LINUXINPUTDEVICEHANDLE_H

#include <string>
#include <memory>
#include <array>
#include <vector>

#include <SurgSim/Devices/MultiAxis/SystemInputDeviceHandle.h>


namespace SurgSim
{
namespace Device
{

/// A wrapper for system-dependent access to a input/HID device.
class LinuxInputDeviceHandle : public SystemInputDeviceHandle
{
public:
	/// Destructor.
	~LinuxInputDeviceHandle();

	/// Enumerates input devices.
	/// \param logger	The logger to be used during enumeration.
	/// \return	A list of device paths.
	static std::vector<std::string> enumerate(SurgSim::Framework::Logger* logger);

	/// Opens the given path and creates an access wrapper for the device.
	/// \param	path	Full pathname for the device.
	/// \param logger	The logger to be used by the device.
	/// \return	The created device object, or an empty unique_ptr on failure.
	static std::unique_ptr<LinuxInputDeviceHandle> open(const std::string& path,
		std::shared_ptr<SurgSim::Framework::Logger> logger);

	virtual std::string getDeviceName() const override;

	virtual bool getDeviceIds(int* vendorId, int* productId) const override;

	virtual bool hasTranslationAndRotationAxes() const override;

	virtual bool updateStates(AxisStates* axisStates, ButtonStates* buttonStates, bool* updated) override;

	virtual bool canRead() const override;

	virtual bool canWrite() const override;

	virtual bool hasDataToRead() const override;

	virtual bool readBytes(void* dataBuffer, size_t bytesToRead, size_t* bytesActuallyRead) override;

	// XXX HORRIBLE HACK!!!
#ifndef HID_WINDDK_XXX
	virtual int get() const override;
#else /* HID_WINDDK_XXX */
	virtual void* get() const override;
#endif /* HID_WINDDK_XXX */

private:
	/// Constructor.
	/// Cannot be called directly; see open and enumerate.
	explicit LinuxInputDeviceHandle(std::shared_ptr<SurgSim::Framework::Logger>&& logger);

	// Prevent copy construction and copy assignment.
	LinuxInputDeviceHandle(const LinuxInputDeviceHandle& other) = delete;
	LinuxInputDeviceHandle& operator=(const LinuxInputDeviceHandle& other) = delete;

	/// Gets the indices of the available device buttons.
	/// \return a vector of indices.
	std::vector<int> getDeviceButtonsAndKeys();

	/// Query if this device has 3 translation and 3 rotation {\em absolute} axes.
	/// \return	true if the desired axes are present.
	bool hasAbsoluteTranslationAndRotationAxes() const;
	/// Query if this device has 3 translation and 3 rotation {\em relative} axes.
	/// \return	true if the desired axes are present.
	bool hasRelativeTranslationAndRotationAxes() const;

	struct State;
	std::unique_ptr<State> m_state;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_MULTIAXIS_LINUXINPUTDEVICEHANDLE_H
