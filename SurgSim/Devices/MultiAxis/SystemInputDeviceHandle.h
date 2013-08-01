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

#ifndef SURGSIM_DEVICES_MULTIAXIS_SYSTEMINPUTDEVICEHANDLE_H
#define SURGSIM_DEVICES_MULTIAXIS_SYSTEMINPUTDEVICEHANDLE_H

#include <string>
#include <memory>
#include <array>
#include <vector>

namespace SurgSim
{
namespace Framework
{
class Logger;
};  // namespace Framework

namespace Device
{

/// A wrapper for system-dependent access to a input/HID device.
class SystemInputDeviceHandle
{
public:
	/// The maximum number of axes supported by any device object.
	static const size_t MAX_NUM_AXES = 6;
	/// The maximum number of buttons supported by any device object.
	static const size_t MAX_NUM_BUTTONS = 4;

	/// Type used to store axis states.
	typedef std::array<int, MAX_NUM_AXES> AxisStates;
	/// Type used to store button states.
	typedef std::array<bool, MAX_NUM_BUTTONS> ButtonStates;


	/// Destructor.
	virtual ~SystemInputDeviceHandle();

	/// Enumerates input devices.
	/// \param logger	The logger to be used during enumeration.
	/// \return	A list of device paths.
	static std::vector<std::string> enumerate(SurgSim::Framework::Logger* logger);

	/// Opens the given path and creates an access wrapper for the device.
	/// \param path	Full pathname for the device.
	/// \param logger	The logger to be used by the device.
	/// \return	The created device object, or an empty unique_ptr on failure.
	static std::unique_ptr<SystemInputDeviceHandle> open(const std::string& path,
		std::shared_ptr<SurgSim::Framework::Logger> logger);

	/// Query if this device has 3 translation and 3 rotation axes.
	/// \return	true if the desired axes are present.
	virtual bool hasTranslationAndRotationAxes() const = 0;

	/// Updates the axis and states from the device input, if any.
	/// \param [in,out] axisStates	The states for each axis of the device.
	/// \param [in,out] buttonStates	The states for each device button.
	/// \param [out] updated True if any states were actually updated.  (Note that even if this value is true, the
	/// 	states may not have changed value; one or more states could have been updated to the same value.)
	/// \return	true if the operation was successful; false if the device is no longer in a usable state.
	virtual bool updateStates(AxisStates* axisStates, ButtonStates* buttonStates, bool* updated) = 0;

	/// Determines if the file handle can be read from.
	/// \return	true if the handle has been open for reading.
	virtual bool canRead() const = 0;

	/// Determines if the file handle can be written to.
	/// \return	true if the handle has been open for writing.
	virtual bool canWrite() const = 0;

	/// Checks whether this object has data available to be read.
	/// \return	true if there is data currently available.
	virtual bool hasDataToRead() const = 0;

	/// Reads bytes from the file handle.
	/// \param [out]	dataBuffer	Buffer to read into.  Must have room for at least bytesToRead bytes of data.
	/// \param	bytesToRead	The number of bytes to try reading.  Actual number of bytes received may be smaller.
	/// \param [out]	bytesActuallyRead	The number of bytes that were actually read into the buffer.
	/// \return	true if it succeeds, false if it fails.
	virtual bool readBytes(void* dataBuffer, size_t bytesToRead, size_t* bytesActuallyRead) = 0;

	// XXX HORRIBLE HACK!!!
#ifndef HID_WINDDK_XXX
	virtual int get() const = 0;
#else /* HID_WINDDK_XXX */
	virtual void* get() const = 0;
#endif /* HID_WINDDK_XXX */

protected:
	/// Default constructor.
	/// Cannot be called directly; see open and enumerate.
	SystemInputDeviceHandle();

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	SystemInputDeviceHandle(const SystemInputDeviceHandle& other) /*= delete*/;
	SystemInputDeviceHandle& operator=(const SystemInputDeviceHandle& other) /*= delete*/;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_MULTIAXIS_SYSTEMINPUTDEVICEHANDLE_H
