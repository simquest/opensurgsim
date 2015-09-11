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

#include "SurgSim/Devices/MultiAxis/win32/WdkHidDeviceHandle.h"

#undef  _WIN32_WINNT
#define _WIN32_WINNT 0x0501   // request Windows XP-compatible SDK APIs
#undef  WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN   // do not automatically include WinSock 1 and some other header files
#include <windows.h>

#include <setupapi.h>
extern "C" {  // sigh...
#include <hidsdi.h>
}

#include <stdint.h>

#include "SurgSim/Devices/MultiAxis/GetSystemError.h"
#include "SurgSim/Devices/MultiAxis/win32/FileHandle.h"
#include "SurgSim/Framework/Log.h"

using SurgSim::Devices::Internal::getSystemErrorCode;
using SurgSim::Devices::Internal::getSystemErrorText;


namespace SurgSim
{
namespace Devices
{

// Usage page and usage IDs for interface devices; see e.g. http://www.usb.org/developers/devclass_docs/Hut1_12v2.pdf
enum UsagePageConstants
{
	DEV_USAGE_PAGE_GENERIC_DESKTOP		= 0x01		// Generic Desktop usage page
};
enum UsageConstants
{
	// Usages for the DEV_USAGE_PAGE_GENERIC_DESKTOP usage page:
	DEV_USAGE_ID_MOUSE						= 0x02,	// Mouse usage ID
	DEV_USAGE_ID_JOYSTICK					= 0x04,	// Joystick usage ID
	DEV_USAGE_ID_GAME_PAD					= 0x05,	// Game Pad usage ID
	DEV_USAGE_ID_KEYBOARD					= 0x06,	// Keyboard usage ID
	DEV_USAGE_ID_KEYPAD						= 0x07,	// Keypad usage ID
	DEV_USAGE_ID_MULTI_AXIS_CONTROLLER		= 0x08	// Multi-axis Controller usage ID
};


struct WdkHidDeviceHandle::State
{
public:
	explicit State(std::shared_ptr<SurgSim::Framework::Logger>&& logger_) :
		logger(std::move(logger_)),
		handle(),
		isOverlappedReadPending(false),
		isDeviceDead(false)
	{
	}

	/// The logger to use.
	std::shared_ptr<SurgSim::Framework::Logger> logger;
	/// The underlying device file handle.
	FileHandle handle;
	/// The OVERLAPPED state structure for overlapped (i.e. asynchronous) reads.
	OVERLAPPED overlappedReadState;
	/// The buffer used to store the output of overlapped (i.e. asynchronous) reads.
	unsigned char overlappedReadBuffer[7*128];
	/// True if we are waiting for the result of an overlapped (i.e. asynchronous) read.
	bool isOverlappedReadPending;
	/// True if the communication with this device has failed without possibility of recovery.
	bool isDeviceDead;

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	State(const State& other) /*= delete*/;
	State& operator=(const State& other) /*= delete*/;
};

WdkHidDeviceHandle::WdkHidDeviceHandle(std::shared_ptr<SurgSim::Framework::Logger>&& logger) :
	m_state(new WdkHidDeviceHandle::State(std::move(logger)))
{
}

WdkHidDeviceHandle::~WdkHidDeviceHandle()
{
}

std::vector<std::string> WdkHidDeviceHandle::enumeratePaths(SurgSim::Framework::Logger* logger)
{
	std::vector<std::string> results;

	// Prepare to iterate over the attached HID devices
	GUID hidGuid;
	HidD_GetHidGuid(&hidGuid);
	HDEVINFO hidDeviceInfo = SetupDiGetClassDevs(&hidGuid, NULL, NULL,
		DIGCF_DEVICEINTERFACE | DIGCF_PRESENT | DIGCF_PROFILE);
	if (hidDeviceInfo == INVALID_HANDLE_VALUE)
	{
		DWORD error = GetLastError();
		SURGSIM_LOG_CRITICAL(logger) << "WdkHidDeviceHandle::enumerate: Failed to query HID devices;" <<
			" SetupDiGetClassDevs() failed with error " << error << ", " << getSystemErrorText(error);
		return results;
	}

	// Loop through the device list, looking for the devices we want
	for (int hidEnumerationIndex = 0; true; ++hidEnumerationIndex)
	{
		// Get the next interface in the list.
		SP_DEVICE_INTERFACE_DATA deviceInterfaceData;
		deviceInterfaceData.cbSize = sizeof(deviceInterfaceData);
		if (! SetupDiEnumDeviceInterfaces(hidDeviceInfo, NULL, &hidGuid, hidEnumerationIndex, &deviceInterfaceData))
		{
			DWORD error = GetLastError();
			if (error == ERROR_NO_MORE_ITEMS)
			{
				break;
			}
			else
			{
				SURGSIM_LOG_CRITICAL(logger) << "WdkHidDeviceHandle::enumerate: Failed to query HID devices;" <<
					" SetupDiEnumDeviceInterfaces() failed with error " << error << ", " << getSystemErrorText(error);
				return results;
			}
		}

		// Find out the required size.
		DWORD deviceInterfaceDetailSize = 0;
		if (! SetupDiGetDeviceInterfaceDetail(hidDeviceInfo, &deviceInterfaceData, NULL, 0,
											  &deviceInterfaceDetailSize, NULL))
		{
			DWORD error = GetLastError();
			if (error != ERROR_INSUFFICIENT_BUFFER)
			{
				SURGSIM_LOG_INFO(logger) << "WdkHidDeviceHandle::enumerate: Failed to get the device detail size," <<
					" device will be ignored; error " << error << ", " << getSystemErrorText(error);
				continue;
			}
		}

		// Get the device detail (which actually just means the path).
		SP_DEVICE_INTERFACE_DETAIL_DATA* deviceInterfaceDetail =
			static_cast<SP_DEVICE_INTERFACE_DETAIL_DATA*>(malloc(deviceInterfaceDetailSize));
		deviceInterfaceDetail->cbSize = sizeof(*deviceInterfaceDetail);
		if (! SetupDiGetDeviceInterfaceDetail(hidDeviceInfo, &deviceInterfaceData,
			deviceInterfaceDetail, deviceInterfaceDetailSize, NULL, NULL))
		{
			DWORD error = GetLastError();
			SURGSIM_LOG_INFO(logger) << "WdkHidDeviceHandle::enumerate: Failed to get the HID device detail," <<
				" device will be ignored; error " << error << ", " << getSystemErrorText(error);
			free(deviceInterfaceDetail);
			continue;
		}

		std::string devicePath(deviceInterfaceDetail->DevicePath);
		free(deviceInterfaceDetail);

		FileHandle handle;
		if (! handle.openForReadingAndMaybeWriting(devicePath))
		{
			DWORD error = GetLastError();
			SURGSIM_LOG_INFO(logger) << "WdkHidDeviceHandle::enumerate: Could not open device " << devicePath <<
				": error " << error << ", " << getSystemErrorText(error);
			continue;
		}

		results.push_back(devicePath);
	}

	return results;
}

std::unique_ptr<WdkHidDeviceHandle> WdkHidDeviceHandle::open(
	const std::string& path, std::shared_ptr<SurgSim::Framework::Logger> logger)
{
	std::unique_ptr<WdkHidDeviceHandle> object(new WdkHidDeviceHandle(std::move(logger)));
	object->m_state->handle.setFileOpenFlags(FILE_FLAG_OVERLAPPED);  // set up the handle for asynchronous I/O
	if (! object->m_state->handle.openForReadingAndMaybeWriting(path))
	{
		object.reset();  // could not open the device handle; destroy the object again
	}
	else
	{
		object->m_state->isOverlappedReadPending = false;
		object->m_state->isDeviceDead = false;
	}
	return object;
}

static std::string convertWideString(const wchar_t* wideString)
{
	char buffer[4096];
	int status = WideCharToMultiByte(CP_UTF8, 0, wideString, -1, buffer, sizeof(buffer), nullptr, nullptr);
	if (! (status > 0 && status < sizeof(buffer)))
	{
		_snprintf(buffer, sizeof(buffer), "???");
	}
	return std::string(buffer);
}

std::string WdkHidDeviceHandle::getDeviceName() const
{
	wchar_t manufacturer[1024];
	if (HidD_GetManufacturerString(m_state->handle.get(), manufacturer, sizeof(manufacturer)) != TRUE)
	{
		manufacturer[0] = '\0';
	}

	wchar_t product[1024];
	if (HidD_GetProductString(m_state->handle.get(), product, sizeof(product)) != TRUE)
	{
		product[0] = '\0';
	}

	std::string result("");
	if (manufacturer[0])
	{
		result = convertWideString(manufacturer) + " ";
	}
	if (product[0])
	{
		result += convertWideString(product);
	}
	else
	{
		result += "???";
	}
	return result;
}

bool WdkHidDeviceHandle::getDeviceIds(int* vendorId, int* productId) const
{
	HIDD_ATTRIBUTES attributes;
	attributes.Size = sizeof(attributes);
	if (HidD_GetAttributes(m_state->handle.get(), &attributes) != TRUE)
	{
		DWORD error = GetLastError();
		SURGSIM_LOG_INFO(m_state->logger) << "WdkHidDeviceHandle: Could not get attributes/IDs: error " << error <<
			", " << getSystemErrorText(error);
		*vendorId = *productId = -1;
		return false;
	}

	*vendorId  = attributes.VendorID;
	*productId = attributes.ProductID;
	return true;
}

bool WdkHidDeviceHandle::getCapabilities(HIDP_CAPS* capabilities) const
{
	PHIDP_PREPARSED_DATA preParsedData = 0;
	if (HidD_GetPreparsedData(m_state->handle.get(), &preParsedData) != TRUE)
	{
		DWORD error = GetLastError();
		SURGSIM_LOG_INFO(m_state->logger) << "WdkHidDeviceHandle: Could not get preparsed data: error " << error <<
			", " << getSystemErrorText(error);
		return false;
	}

	if (HidP_GetCaps(preParsedData, capabilities) != HIDP_STATUS_SUCCESS)
	{
		DWORD error = GetLastError();
		SURGSIM_LOG_INFO(m_state->logger) << "WdkHidDeviceHandle: Could not get capabilities: error " << error <<
			", " << getSystemErrorText(error);
		HidD_FreePreparsedData(preParsedData);
		return false;
	}

	HidD_FreePreparsedData(preParsedData);  // don't need the pre-parsed data any more
	return true;
}

bool WdkHidDeviceHandle::hasTranslationAndRotationAxes() const
{
	HIDP_CAPS capabilities;
	if (! getCapabilities(&capabilities))
	{
		// message already shown
		return false;
	}

	if ((capabilities.UsagePage != DEV_USAGE_PAGE_GENERIC_DESKTOP) ||
		(capabilities.Usage != DEV_USAGE_ID_MULTI_AXIS_CONTROLLER))
	{
		SURGSIM_LOG_DEBUG(m_state->logger) << "WdkHidDeviceHandle: device is not a multi-axis controller.";
		return false;
	}

	int numExtraAxes = static_cast<int>(capabilities.NumberInputValueCaps) - 6;
	if (numExtraAxes < 0)
	{
		SURGSIM_LOG_DEBUG(m_state->logger) << "WdkHidDeviceHandle: device does not have 6 input axes.";
		return false;
	}
	else if (numExtraAxes > 0)
	{
		SURGSIM_LOG_INFO(m_state->logger) << "WdkHidDeviceHandle: device has more than 6 axes;" <<
			" ignoring " << numExtraAxes << " additional axes.";
	}

	return true;
}

bool WdkHidDeviceHandle::startAsynchronousRead()
{
	SURGSIM_ASSERT(! m_state->isOverlappedReadPending) << "Previous asynchronous read has not been handled!";

	memset(&(m_state->overlappedReadState), 0, sizeof(m_state->overlappedReadState));
	if (ReadFile(m_state->handle.get(), m_state->overlappedReadBuffer, sizeof(m_state->overlappedReadBuffer),
		NULL, &(m_state->overlappedReadState)) != TRUE)
	{
		DWORD error = GetLastError();
		if (error == ERROR_IO_PENDING)
		{
			m_state->isOverlappedReadPending = true;
		}
		else if (error == ERROR_DEVICE_NOT_CONNECTED)
		{
			SURGSIM_LOG_SEVERE(m_state->logger) <<
				"WdkHidDeviceHandle: read failed; device has been disconnected!  (stopping)";
			m_state->isDeviceDead = true;
		}
		else
		{
			SURGSIM_LOG_WARNING(m_state->logger) << "WdkHidDeviceHandle: read failed with error " <<
				error << ", " << getSystemErrorText(error);
		}
	}
	else
	{
		// Read succeeded synchronously. That means that the read has actually completed, but we still need to
		// retrieve the returned data using GetOverlappedResult, just like for a pending result.
		m_state->isOverlappedReadPending = true;
	}

	return m_state->isOverlappedReadPending;
}

bool WdkHidDeviceHandle::finishAsynchronousRead(size_t* numBytesRead)
{
	SURGSIM_ASSERT(m_state->isOverlappedReadPending) << "Asynchronous read has not been started!";

	DWORD numRead = 0;
	if (GetOverlappedResult(m_state->handle.get(), &(m_state->overlappedReadState), &numRead, FALSE) == FALSE)
	{
		DWORD error = GetLastError();
		if (error == ERROR_IO_INCOMPLETE)
		{
			// keep checking for asynchronous I/O completion
		}
		else if (error == ERROR_DEVICE_NOT_CONNECTED)
		{
			SURGSIM_LOG_SEVERE(m_state->logger) <<
				"WdkHidDeviceHandle: read failed; device has been disconnected!  (stopping)";
			m_state->isDeviceDead = true;
			m_state->isOverlappedReadPending = false;
		}
		else
		{
			SURGSIM_LOG_WARNING(m_state->logger) << "WdkHidDeviceHandle: GetOverlappedResult failed with error " <<
				error << ", " << getSystemErrorText(error);
			// keep checking for asynchronous I/O completion, I guess
		}
		*numBytesRead = 0;
		return false;
	}

	// The read has been completed.
	m_state->isOverlappedReadPending = false;
	*numBytesRead = numRead;
	return true;
}

void WdkHidDeviceHandle::cancelAsynchronousRead()
{
	if (CancelIo(m_state->handle.get()) == FALSE)
	{
		DWORD error = GetLastError();
		if (error == ERROR_NOT_FOUND)
		{
			// No requests were pending.
			SURGSIM_LOG_WARNING(m_state->logger) <<
				"WdkHidDeviceHandle: No asynchronous I/O requests were pending when attempting to cancel.";
			m_state->isOverlappedReadPending = false;
		}
		else
		{
			SURGSIM_LOG_WARNING(m_state->logger) <<
				"WdkHidDeviceHandle: Could not cancel pending asynchronous I/O; error " <<
				error << ", " << getSystemErrorText(error);
		}
	}
	else
	{
		DWORD numRead = 0;
		if (GetOverlappedResult(m_state->handle.get(), &(m_state->overlappedReadState), &numRead, TRUE) == FALSE)
		{
			DWORD error = GetLastError();
			if (error == ERROR_OPERATION_ABORTED)
			{
				// It worked.
				m_state->isOverlappedReadPending = false;
			}
			else
			{
				SURGSIM_LOG_WARNING(m_state->logger) <<
					"WdkHidDeviceHandle: Final GetOverlappedResult failed with error " <<
					error << ", " << getSystemErrorText(error);
			}
		}
		else
		{
			m_state->isOverlappedReadPending = false;
		}
	}
}

bool WdkHidDeviceHandle::updateStates(AxisStates* axisStates, ButtonStates* buttonStates, bool* updated)
{
	*updated = false;

	// Both WaitForSingleObject() and WaitForMultipleObjects() always claim data is available for 3DConnexion device
	// file handles.  So we can't check for data availability that way.  Instead, we use overlapped (asynchronous) I/O.

	int numInitialFinished = 0;
	int numStarted = 0;
	int numFinished = 0;

	if (m_state->isOverlappedReadPending)
	{
		size_t numRead = 0;
		if (! finishAsynchronousRead(&numRead))
		{
			if (m_state->isDeviceDead)
			{
				return false;  // stop updating this device!
			}
		}
		else
		{
			decodeStateUpdates(m_state->overlappedReadBuffer, numRead, axisStates, buttonStates, updated);
			++numInitialFinished;
		}
	}

	while (! m_state->isOverlappedReadPending)
	{
		if (! startAsynchronousRead())
		{
			if (m_state->isDeviceDead)
			{
				return false;  // stop updating this device!
			}
			else
			{
				break;
			}
		}
		++numStarted;

		size_t numRead = 0;
		if (! finishAsynchronousRead(&numRead))
		{
			if (m_state->isDeviceDead)
			{
				return false;  // stop updating this device!
			}
		}
		else
		{
			decodeStateUpdates(m_state->overlappedReadBuffer, numRead, axisStates, buttonStates, updated);
			++numFinished;
		}
	}

	if (numInitialFinished > 0 || numStarted > 0 || numFinished > 0)
	{
		SURGSIM_LOG_DEBUG(m_state->logger) << "WdkHidDeviceHandle: started " << numStarted << " reads, finished " <<
			numInitialFinished << "+" << numFinished << ", delta = " << (numStarted - numInitialFinished - numFinished);
	}

	return true;
}

void WdkHidDeviceHandle::prepareForShutdown()
{
	// When this code is running, the thread that calls updateStates() should no longer be running.
	// So we make the assumption that no synchronization is needed.

	// Cancel any pending asynchronous I/O, so it doesn't try reading into memory that is about to be freed.
	if (m_state->isOverlappedReadPending)
	{
		cancelAsynchronousRead();
	}
}

static inline int16_t signedShortData(unsigned char byte0, unsigned char byte1)
{
	return static_cast<int16_t>(static_cast<uint16_t>(byte0) | (static_cast<uint16_t>(byte1) << 8));
}

void WdkHidDeviceHandle::decodeStateUpdates(const unsigned char* rawData, size_t rawDataSize,
											  AxisStates* axisStates, ButtonStates* buttonStates, bool* updated)
{
	if ((rawDataSize >= 7) && (rawData[0] == 0x01))       // Translation
	{
		// We could parse this via HidP_GetData(), but that won't work for buttons (see below)
		(*axisStates)[0] = signedShortData(rawData[1],  rawData[2]);
		(*axisStates)[1] = signedShortData(rawData[3],  rawData[4]);
		(*axisStates)[2] = signedShortData(rawData[5],  rawData[6]);
		*updated = true;

		if ((rawDataSize >= 14) && (rawData[7] == 0x02))  // translation data may have rotation appended to it
		{
			(*axisStates)[3] = signedShortData(rawData[8],  rawData[9]);
			(*axisStates)[4] = signedShortData(rawData[10],  rawData[11]);
			(*axisStates)[5] = signedShortData(rawData[12],  rawData[13]);
		}
	}
	else if ((rawDataSize >= 7) && (rawData[0] == 0x02))  // Rotation
	{
		// We could parse this via HidP_GetData(), but that won't work for buttons (see below)
		(*axisStates)[3] = signedShortData(rawData[1],  rawData[2]);
		(*axisStates)[4] = signedShortData(rawData[3],  rawData[4]);
		(*axisStates)[5] = signedShortData(rawData[5],  rawData[6]);
		*updated = true;

		if ((rawDataSize >= 14) && (rawData[7] == 0x01))  // rotation data may have translation appended to it
		{
			(*axisStates)[0] = signedShortData(rawData[8],  rawData[9]);
			(*axisStates)[1] = signedShortData(rawData[10],  rawData[11]);
			(*axisStates)[2] = signedShortData(rawData[12],  rawData[13]);
		}
	}
	else if ((rawDataSize >= 2) && (rawData[0] == 0x03))  // Buttons
	{
		// We CAN'T parse buttons via HidP_GetData(), because 3DConnexion devices produce button state in a
		// different report from the axes, so when the last button is released you simply get no data.  We could
		// interpret "empty data list" as "the last button has been released", but that seems dangerous.  So we
		// roll our own parsing for now, even though it may not work for other devices.  --advornik 2012-08-01

		size_t currentByte = 1;  // Byte 0 specifies the packet type; data starts at byte 1
		unsigned char currentBit = 0x01;
		for (size_t i = 0;  i < (*buttonStates).size();  ++i)
		{
			(*buttonStates)[i] = ((rawData[currentByte] & currentBit) != 0);
			if (currentBit < 0x80)
			{
				currentBit = currentBit << 1;
			}
			else
			{
				currentBit = 0x01;
				++currentByte;
				if (currentByte >= rawDataSize)  // out of data?
				{
					break;
				}
			}
		}
		*updated = true;
	}
}


};  // namespace Devices
};  // namespace SurgSim
