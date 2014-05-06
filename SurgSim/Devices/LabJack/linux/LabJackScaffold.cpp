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

#include "SurgSim/Devices/LabJack/LabJackScaffold.h"

#include <algorithm>
#include <array>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <errno.h>
#include <labjackusb.h> // the low-level LabJack library (aka exodriver)
#include <list>
#include <memory>
#include <stdint.h> // fixed-width integer types are used for low-level data communication
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Devices/LabJack/LabJackDevice.h"
#include "SurgSim/Devices/LabJack/LabJackThread.h"
#include "SurgSim/Devices/LabJack/linux/LabJackChecksums.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SharedInstance.h"

namespace SurgSim
{
namespace Device
{

namespace
{
/// Distinguish the name of the hardware library's handle from our wrapper with a prefix.
typedef HANDLE LJ_HANDLE;

/// The low-level LabJack library returns a NULL handle when failing to open a new device.
static const LJ_HANDLE LABJACK_INVALID_HANDLE = NULL;

/// A struct containing the default settings that depend on the type of LabJack.
struct LabJackDefaults
{
	LabJackDefaults()
	{
		timerBase[LabJackType::LABJACKTYPE_U3] = LabJackTimerBase::LABJACKTIMERBASE_2;
		timerBase[LabJackType::LABJACKTYPE_U6] = LabJackTimerBase::LABJACKTIMERBASE_2;
		timerBase[LabJackType::LABJACKTYPE_UE9] = LabJackTimerBase::LABJACKTIMERBASE_1;
	}

	/// The default timer base rate.
	std::unordered_map<LabJackType, LabJackTimerBase, std::hash<int>> timerBase;
};

/// Helper function to configure the number of timers and counters.
/// \param device The LabJackDevice.
/// \param rawHandle The labjackusb handle.
/// \param logger The logger for error messages.
/// \return true if successful.
bool configureNumberOfTimers(LabJackDevice* device, LJ_HANDLE rawHandle,
							 std::shared_ptr<SurgSim::Framework::Logger> logger)
{
	bool result = true;

	// One-time configuration of counters and timers.
	const BYTE numberOfTimers = device->getTimers().size();
	const BYTE counterEnable = 0; // Counters are not currently supported.
	const BYTE pinOffset = device->getTimerCounterPinOffset();

	// First, configure the number of timers and counters
	std::array<BYTE, LABJACK_MAXIMUM_BUFFER> sendBytes;
	sendBytes[1] = 0xF8;  //Command byte, ConfigIO
	sendBytes[2] = 0x05;  //Number of data words
	sendBytes[3] = 0x0B;  //Extended command number

	sendBytes[6] = 1;  //Writemask : Setting writemask for timerCounterConfig (bit 0)

	sendBytes[7] = numberOfTimers;      //NumberTimersEnabled
	sendBytes[8] = counterEnable;  //CounterEnable: Bit 0 is Counter 0, Bit 1 is Counter 1
	sendBytes[9] = pinOffset;  //TimerCounterPinOffset

	int sendBytesSize = 16; //ConfigIO command sends 16 bytes
	for (int i = 10; i < sendBytesSize; ++i)
	{
		sendBytes[i] = 0;  //Reserved
	}
	LabJackChecksums::extendedChecksum(&sendBytes, 16);

	int sent = LJUSB_Write(rawHandle, &(sendBytes[0]), sendBytesSize);

	if (sent < sendBytesSize)
	{
		SURGSIM_LOG_SEVERE(logger) <<
			"Failed to write timer/counter configuration information for a device named '" << device->getName() <<
			"'.  " << sendBytesSize << " bytes should have been sent, but only " << sent <<
			" were actually sent." << std::endl << "  labjackusb error code: " << errno << "." << std::endl;
		result = false;
	}

	if (result)
	{
		const int readBytesSize = 16; // timerCounterConfig replies with 16 bytes.
		std::array<BYTE, LABJACK_MAXIMUM_BUFFER> readBytes;
		const int read = LJUSB_Read(rawHandle, &(readBytes[0]), readBytesSize);
		const uint16_t checksumTotal = LabJackChecksums::extendedChecksum16(readBytes, readBytesSize);
		if (read < readBytesSize)
		{
			SURGSIM_LOG_SEVERE(logger) <<
				"Failed to read response of timer/counter configuration for a device named '" << device->getName() <<
				"'.  " << readBytesSize << " bytes were expected, but only " << read << " were received." <<
				std::endl << "  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
		else if ((((checksumTotal / 256 ) & 0xff) != readBytes[5]) ||
			((checksumTotal & 0xff) != readBytes[4]) ||
			(LabJackChecksums::extendedChecksum8(readBytes) != readBytes[0]))
		{
			SURGSIM_LOG_SEVERE(logger) <<
				"Failed to read response of timer/counter configuration for a device named '" << device->getName() <<
				"'.  The checksums are bad." << std::endl <<
				"  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
		else if ((readBytes[1] != sendBytes[1]) || (readBytes[2] != sendBytes[2]) || (readBytes[3] != sendBytes[3]))
		{
			SURGSIM_LOG_SEVERE(logger) <<
				"Failed to read response of timer/counter configuration for a device named '" << device->getName() <<
				"'.  The command bytes are wrong.  Expected: " << static_cast<int>(sendBytes[1]) << ", " <<
				static_cast<int>(sendBytes[2]) << ", " << static_cast<int>(sendBytes[3]) << ".  Received: " <<
				static_cast<int>(readBytes[1]) << ", " << static_cast<int>(readBytes[2]) << ", " <<
				static_cast<int>(readBytes[3]) << "." << std::endl <<
				"  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
		else if (readBytes[6] != 0)
		{
			SURGSIM_LOG_SEVERE(logger) <<
				"Failed to read response of timer/counter configuration for a device named '" << device->getName() <<
				"'.  The device library returned an error code: " << readBytes[6] << std::endl <<
				"  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
	}
	return result;
}

/// Helper function to configure the timers' clock's base and divisor.
/// \param device The LabJackDevice.
/// \param rawHandle The labjackusb handle.
/// \param logger The logger for error messages.
/// \return true if successful.
bool configureClock(LabJackDevice* device, LJ_HANDLE rawHandle, std::shared_ptr<SurgSim::Framework::Logger> logger)
{
	bool result = true;

	LabJackTimerBase base = device->getTimerBase();
	if (base == LABJACKTIMERBASE_DEFAULT)
	{
		LabJackDefaults defaults;
		base = defaults.timerBase[device->getType()];
	}

	// Second, set the clock base and divisor

	// The low-level settings for the timer base are 20 less than the high-level values for the U3 and U6.
	// The low- and high-level settings are the same for the UE9.
	const int timerBaseOffset = 20;
	BYTE timerBase = device->getTimerBase();
	// If the TimerBase is above 20, it was set for a high-level value, convert to the equivalent value for
	// the low-level driver.
	if (timerBase > timerBaseOffset)
	{
		timerBase -= timerBaseOffset;
	}
	const BYTE divisor = device->getTimerClockDivisor();

	// Write the clock configuration.
	std::array<BYTE, LABJACK_MAXIMUM_BUFFER> sendBytes;
	sendBytes[1] = 0xF8;  //Command byte, ConfigTimerClock
	sendBytes[2] = 0x02;  //Number of data words
	sendBytes[3] = 0x0A;  //Extended command number

	sendBytes[6] = 0;  //Reserved
	sendBytes[7] = 0;  //Reserved

	//TimerClockConfig : Configuring the clock (bit 7) and setting the TimerClockBase (bits 0-2)
	sendBytes[8] = timerBase + 128;
	sendBytes[9] = divisor;  //TimerClockDivisor

	int sendBytesSize = 10; // ConfigTimerClock sends 10 bytes
	LabJackChecksums::extendedChecksum(&sendBytes, sendBytesSize);

	int sent = LJUSB_Write(rawHandle, &(sendBytes[0]), sendBytesSize);

	if (sent < sendBytesSize)
	{
		SURGSIM_LOG_SEVERE(logger) <<
			"Failed to write clock configuration information for a device named '" <<
			device->getName() << "'.  " << sendBytesSize << " bytes should have been sent, but only " <<
			sent << " were actually sent." << std::endl << "  labjackusb error code: " << errno << "." <<
			std::endl;
		result = false;
	}

	if (result)
	{
		// Read the response for the clock configuration.
		const int readBytesSize = 10; // ConfigTimerClock replies with 10 bytes.
		std::array<BYTE, LABJACK_MAXIMUM_BUFFER> readBytes;
		const int read = LJUSB_Read(rawHandle, &(readBytes[0]), readBytesSize);
		const uint16_t checksumTotal = LabJackChecksums::extendedChecksum16(readBytes, readBytesSize);
		if (read < readBytesSize)
		{
			SURGSIM_LOG_SEVERE(logger) <<
				"Failed to read response of clock configuration for a device named '" <<
				device->getName() << "'. " << readBytesSize << " bytes were expected, but only " << read <<
				" were received." << std::endl << "  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
		else if ((((checksumTotal / 256 ) & 0xff) != readBytes[5]) ||
			((checksumTotal & 0xff) != readBytes[4]) ||
			(LabJackChecksums::extendedChecksum8(readBytes) != readBytes[0]))
		{
			SURGSIM_LOG_SEVERE(logger) <<
				"Failed to read response of clock configuration for a device named '" <<
				device->getName() << "'.  The checksums are bad." << std::endl <<
				"  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
		else if ((readBytes[1] != sendBytes[1]) || (readBytes[2] != sendBytes[2]) || (readBytes[3] != sendBytes[3]))
		{
			SURGSIM_LOG_SEVERE(logger) <<
				"Failed to read response of clock configuration for a device named '" <<
				device->getName() << "'.  The command bytes are wrong.  Expected: " <<
				static_cast<int>(sendBytes[1]) << ", " << static_cast<int>(sendBytes[2]) << ", " <<
				static_cast<int>(sendBytes[3]) << ".  Received: " << static_cast<int>(readBytes[1]) << ", " <<
				static_cast<int>(readBytes[2]) << ", " << static_cast<int>(readBytes[3]) << "." << std::endl <<
				"  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
		else if (readBytes[6] != 0)
		{
			SURGSIM_LOG_SEVERE(logger) <<
				"Failed to read response of clock configuration for a device named '" <<
				device->getName() << "'.  The device library returned an error code: " << readBytes[6] <<
				std::endl << "  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
	}
	return result;
}

/// Helper function to configure the mode for each timer.  Sets PWMs to almost-always low.
/// \param device The LabJackDevice.
/// \param rawHandle The labjackusb handle.
/// \param logger The logger for error messages.
/// \return true if successful.
bool configureTimer(LabJackDevice* device, LJ_HANDLE rawHandle, std::shared_ptr<SurgSim::Framework::Logger> logger)
{
	bool result = true;

	// Configure each timer's mode via a Feedback command.
	std::array<BYTE, LABJACK_MAXIMUM_BUFFER> sendBytes;
	sendBytes[1] = 0xF8;  //Command byte, Feedback
	sendBytes[3] = 0x00;  //Extended command number

	sendBytes[6] = 0;     //Echo
	int sendBytesSize = 7; // the first IOType goes here

	int readBytesSize = 9; // Reading after a Feedback command provides 9+ bytes.
	const std::unordered_map<int, LabJackTimerMode> timers = device->getTimers();
	for (auto timer = timers.cbegin(); timer != timers.cend(); ++timer)
	{
		const int minimumTimer = 0;
		const int maximumTimer = 3;
		if ((timer->first < minimumTimer) || (timer->first > maximumTimer))
		{
			SURGSIM_LOG_SEVERE(logger) << "Failed to configure a timer for a device named '" <<
				device->getName() << "'. Timer number (the key in the argument to LabJackDevice::setTimers) " <<
				timer->first << " is outside of the valid range " << minimumTimer << "-" << maximumTimer <<
				"." << std::endl;
			result = false;
		}

		if (result)
		{
			const BYTE timerConfigCode = timer->first * 2 + 43; // The timer configs are 43, 45, 47, and 49.
			sendBytes.at(sendBytesSize++) = timerConfigCode;  //IOType, Timer#Config (e.g., Timer0Config)
			sendBytes.at(sendBytesSize++) = timer->second;  //TimerMode
			sendBytes.at(sendBytesSize++) = 0;  //Value LSB
			sendBytes.at(sendBytesSize++) = 0;  //Value MSB

			if ((timer->second == LabJackTimerMode::LABJACKTIMERMODE_PWM8) ||
				(timer->second == LabJackTimerMode::LABJACKTIMERMODE_PWM16))
			{  // Initialize PWMs to almost-always low.
				const BYTE timerCode = timerConfigCode - 1;
				sendBytes.at(sendBytesSize++) = timerCode;   //IOType, Timer# (e.g., Timer0)
				sendBytes.at(sendBytesSize++) = 1;    //UpdateReset
				sendBytes.at(sendBytesSize++) = 255;  //Value LSB
				sendBytes.at(sendBytesSize++) = 255;  //Value MSB
				readBytesSize += 4; // A Timer# IOType causes 4 read bytes.
			}
		}
	}

	if (result)
	{
		// Write the timer Feedback command.
		// Must send an even number of bytes.
		if (sendBytesSize % 2 == 1)
		{
			sendBytes.at(sendBytesSize++) = 0;
		}
		const BYTE sendCommandBytes = 6;
		const BYTE dataWords = (sendBytesSize - sendCommandBytes) / 2;
		sendBytes[2] = dataWords;
		LabJackChecksums::extendedChecksum(&sendBytes, sendBytesSize);

		int sent = LJUSB_Write(rawHandle, &(sendBytes[0]), sendBytesSize);

		if (sent < sendBytesSize)
		{
			SURGSIM_LOG_SEVERE(logger) <<
				"Failed to write timer mode feedback command for a device named '" <<
				device->getName() << "'.  " << sendBytesSize << " bytes should have been sent, but only " <<
				sent << " were actually sent." << std::endl <<
				"  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
	}

	if (result)
	{
		// Read the response for the timer feedback command.
		// Must read an even number of bytes
		if (readBytesSize % 2 == 1)
		{
			++readBytesSize;
		}
		const BYTE readCommandBytes = 6;
		const BYTE dataWords = (readBytesSize - readCommandBytes) / 2;

		std::array<BYTE, LABJACK_MAXIMUM_BUFFER> readBytes;
		const int read = LJUSB_Read(rawHandle, &(readBytes[0]), readBytesSize);
		const uint16_t checksumTotal = LabJackChecksums::extendedChecksum16(readBytes, readBytesSize);
		if (read < readBytesSize)
		{
			SURGSIM_LOG_SEVERE(logger) <<
				"Failed to read response of timer mode feedback command for a device named '" <<
				device->getName() << "'. " << readBytesSize << " bytes were expected, but only " << read <<
				" were received." << std::endl << "  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
		else if ((((checksumTotal / 256 ) & 0xff) != readBytes[5]) ||
			((checksumTotal & 0xff) != readBytes[4]) ||
			(LabJackChecksums::extendedChecksum8(readBytes) != readBytes[0]))
		{
			SURGSIM_LOG_SEVERE(logger) <<
				"Failed to read response of timer mode feedback command for a device named '" <<
				device->getName() << "'.  The checksums are bad." << std::endl <<
				"  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
		else if ((readBytes[1] != sendBytes[1]) || (readBytes[3] != sendBytes[3]))
		{
			SURGSIM_LOG_SEVERE(logger) <<
				"Failed to read response of timer mode feedback command for a device named '" <<
				device->getName() << "'.  The command bytes are wrong.  Expected bytes 1 & 3: " <<
				static_cast<int>(sendBytes[1]) << ", " << static_cast<int>(sendBytes[3]) <<
				".  Received bytes 1 & 3: " << static_cast<int>(readBytes[1]) << ", " <<
				static_cast<int>(readBytes[3]) << "." << std::endl <<
				"  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
		else if (readBytes[2] != dataWords)
		{
			SURGSIM_LOG_SEVERE(logger) <<
				"Failed to read response of timer mode feedback command for a device named '" <<
				device->getName() << "'.  The number of bytes in the response is wrong.  Expected: " <<
				dataWords << ".  Received: " << readBytes[2] << "." << std::endl <<
				"  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
		else if (readBytes[6] != 0)
		{
			SURGSIM_LOG_SEVERE(logger) <<
				"Failed to read response of timer mode feedback command for a device named '" <<
				device->getName() << "'.  The device library returned an error code: " << readBytes[6] <<
				", for frame: " << readBytes[7] << std::endl <<
				"  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
	}
	return result;
}

/// Helper function to configure the input/output direction for digital lines.  Does not set the value for outputs.
/// \param device The LabJackDevice.
/// \param rawHandle The labjackusb handle.
/// \param logger The logger for error messages.
/// \return true if successful.
bool configureDigitalLines(LabJackDevice* device, LJ_HANDLE rawHandle,
						   std::shared_ptr<SurgSim::Framework::Logger> logger)
{
	bool result = true;

	const std::unordered_set<int>& digitalInputChannels = device->getDigitalInputChannels();
	const std::unordered_set<int>& digitalOutputChannels = device->getDigitalOutputChannels();
	if ((digitalInputChannels.size() > 0) || (digitalOutputChannels.size() > 0))
	{
		// Configure each digital line's direction via a Feedback command.  The output lines will automatically be
		// forced high when we write their states in updateDevice, but we must explicitly set the direction on
		// input lines so we just do both here.
		std::array<BYTE, LABJACK_MAXIMUM_BUFFER> sendBytes;
		sendBytes[1] = 0xF8;  //Command byte, Feedback
		sendBytes[3] = 0x00;  //Extended command number

		sendBytes[6] = 0;     //Echo
		int sendBytesSize = 7; // the first IOType goes here

		int readBytesSize = 9; // Reading after a Feedback command provides 9+ bytes.
		for (auto input = digitalInputChannels.cbegin(); input != digitalInputChannels.cend(); ++input)
		{
			sendBytes.at(sendBytesSize++) = 13;  //IOType, BitDirWrite
			sendBytes.at(sendBytesSize++) = *input;  //Bits 0-4: IO Number, Bit 7: Direction (1=output, 0=input)
		}
		for (auto output = digitalOutputChannels.cbegin(); output != digitalOutputChannels.cend(); ++output)
		{
			sendBytes.at(sendBytesSize++) = 13;  //IOType, BitDirWrite
			sendBytes.at(sendBytesSize++) = *output + 128;  //Bits 0-4: IO Number, Bit 7: Direction (1=output, 0=input)
		}

		// Write the digital input/output Feedback command.
		// Must send an even number of bytes.
		if (sendBytesSize % 2 == 1)
		{
			sendBytes.at(sendBytesSize++) = 0;
		}
		const BYTE sendCommandBytes = 6;
		const BYTE dataWords = (sendBytesSize - sendCommandBytes) / 2;
		sendBytes[2] = dataWords;
		LabJackChecksums::extendedChecksum(&sendBytes, sendBytesSize);

		int sent = LJUSB_Write(rawHandle, &(sendBytes[0]), sendBytesSize);

		if (sent < sendBytesSize)
		{
			SURGSIM_LOG_SEVERE(logger) <<
				"Failed to write digital line direction feedback command for a device named '" <<
				device->getName() << "'.  " << sendBytesSize << " bytes should have been sent, but only " <<
				sent << " were actually sent." << std::endl <<
				"  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}

		if (result)
		{
			// Read the response for the digital input/output feedback command.
			// Must read an even number of bytes
			if (readBytesSize % 2 == 1)
			{
				++readBytesSize;
			}
			const BYTE readCommandBytes = 6;
			const BYTE dataWords = (readBytesSize - readCommandBytes) / 2;

			std::array<BYTE, LABJACK_MAXIMUM_BUFFER> readBytes;
			const int read = LJUSB_Read(rawHandle, &(readBytes[0]), readBytesSize);
			const uint16_t checksumTotal = LabJackChecksums::extendedChecksum16(readBytes, readBytesSize);
			if (read < readBytesSize)
			{
				SURGSIM_LOG_SEVERE(logger) <<
					"Failed to read response of digital line direction configuration for a device named '" <<
					device->getName() << "'. " << readBytesSize << " bytes were expected, but only " << read <<
					" were received." << std::endl << "  labjackusb error code: " << errno << "." << std::endl;
				result = false;
			}
			else if ((((checksumTotal / 256 ) & 0xff) != readBytes[5]) ||
				((checksumTotal & 0xff) != readBytes[4]) ||
				(LabJackChecksums::extendedChecksum8(readBytes) != readBytes[0]))
			{
				SURGSIM_LOG_SEVERE(logger) <<
					"Failed to read response of digital line direction configuration for a device named '" <<
					device->getName() << "'.  The checksums are bad." << std::endl <<
					"  labjackusb error code: " << errno << "." << std::endl;
				result = false;
			}
			else if ((readBytes[1] != sendBytes[1]) || (readBytes[3] != sendBytes[3]))
			{
				SURGSIM_LOG_SEVERE(logger) <<
					"Failed to read response of digital line direction configuration for a device named '" <<
					device->getName() << "'.  The command bytes are wrong.  Expected bytes 1 & 3: " <<
					static_cast<int>(sendBytes[1]) << ", " << static_cast<int>(sendBytes[3]) <<
					".  Received bytes 1 & 3: " << static_cast<int>(readBytes[1]) << ", " <<
					static_cast<int>(readBytes[3]) << "." << std::endl <<
					"  labjackusb error code: " << errno << "." << std::endl;
				result = false;
			}
			else if (readBytes[2] != dataWords)
			{
				SURGSIM_LOG_SEVERE(logger) <<
					"Failed to read response of digital line direction configuration for a device named '" <<
					device->getName() << "'.  The number of bytes in the response is wrong.  Expected: " <<
					dataWords << ".  Received: " << readBytes[2] << "." << std::endl <<
					"  labjackusb error code: " << errno << "." << std::endl;
				result = false;
			}
			else if (readBytes[6] != 0)
			{
				SURGSIM_LOG_SEVERE(logger) <<
					"Failed to read response of digital line direction configuration for a device named '" <<
					device->getName() << "'.  The device library returned an error code: " << readBytes[6] <<
					", for frame: " << readBytes[7] << std::endl <<
					"  labjackusb error code: " << errno << "." << std::endl;
				result = false;
			}
		}
	}
	return result;
}

/// Helper function to configure the timers.
/// \param device The LabJackDevice.
/// \param rawHandle The labjackusb handle.
/// \param logger The logger for error messages.
/// \return true if successful.
bool configureTimers(LabJackDevice* device, LJ_HANDLE rawHandle, std::shared_ptr<SurgSim::Framework::Logger> logger)
{
	bool result = configureNumberOfTimers(device, rawHandle, logger);

	if (result && (device->getTimers().size() > 0))
	{
		result = configureClock(device, rawHandle, logger);
		result = result && configureTimer(device, rawHandle, logger);
	}
	return result;
}
};

class LabJackScaffold::Handle
{
public:
	/// Constructor that attempts to open a device.
	/// \param deviceType The type of LabJack device to open (see strings in LabJackUD.h).
	/// \param connectionType How to connect to the device (e.g., USB) (see strings in LabJackUD.h).
	/// \param address Either the ID or serial number (if USB), or the IP address.
	Handle(SurgSim::Device::LabJackType deviceType, SurgSim::Device::LabJackConnection connectionType,
		const std::string& address) :
		m_deviceHandle(LABJACK_INVALID_HANDLE),
		m_address(address),
		m_type(deviceType),
		m_connection(connectionType),
		m_scaffold(LabJackScaffold::getOrCreateSharedInstance())
	{
		create();
	}

	/// Destructor.
	~Handle()
	{
		SURGSIM_ASSERT(!isValid()) << "Expected destroy() to be called before Handle object destruction.";
	}

	/// \return Whether or not the wrapped handle is valid.
	bool isValid() const
	{
		return (m_deviceHandle != LABJACK_INVALID_HANDLE);
	}

	/// Helper function called by the constructor to open the LabJack device for communications.
	void create()
	{
		SURGSIM_ASSERT(!isValid()) <<
			"Expected LabJackScaffold::Handle::create() to be called on an uninitialized object.";

		bool result = true;

		if (m_type == LABJACKTYPE_UE9)
		{
			SURGSIM_LOG_SEVERE(m_scaffold->getLogger()) << "Failed to open a device. " <<
				"The UE9 model LabJack is not supported for the low-level driver used on Linux & Mac. " <<
				"The commands for the UE9 have a different structure, which is not currently implemented." <<
				std::endl <<
				"  Type: '" << m_type << "'.  Connection: '" << m_connection << "'.  Address: '" <<
				m_address << "'." << std::endl;
			result = false;
		}

		if (m_connection != LABJACKCONNECTION_USB)
		{
			SURGSIM_LOG_SEVERE(m_scaffold->getLogger()) << "Failed to open a device. " <<
				"The LabJackDevice connection must be set to USB for the low-level driver used on Linux & Mac." <<
				std::endl <<
				"  Type: '" << m_type << "'.  Connection: '" << m_connection << "'.  Address: '" <<
				m_address << "'." << std::endl;
			result = false;
		}

		unsigned int deviceNumber = 1; // If no address is specified, grab the first device found of this type.
		if (m_address.length() > 0)
		{
			try
			{
				deviceNumber = std::stoi(m_address);
			}
			catch (int e)
			{
				SURGSIM_LOG_SEVERE(m_scaffold->getLogger()) << "Failed to open a device. " <<
					"The LabJackDevice address should be a string representation of an unsigned integer " <<
					"corresponding to the device number (or the empty string to get the first device), " <<
					"but the conversion from string to integer failed." << std::endl <<
					"  Type: '" << m_type << "'.  Connection: '" << m_connection << "'.  Address: '" <<
					m_address << "'." << std::endl;
				result = false;
			}
		}

		if (result)
		{
			const unsigned int dwReserved = 0; // Not used, set to 0.
			m_deviceHandle = LJUSB_OpenDevice(deviceNumber, dwReserved, m_type);
			if (m_deviceHandle == LABJACK_INVALID_HANDLE)
			{
				SURGSIM_LOG_SEVERE(m_scaffold->getLogger()) << "Failed to open a device." <<
					std::endl <<
					"  Type: '" << m_type << "'.  Connection: '" << m_connection << "'.  Address: '" <<
					m_address << "'." << std::endl <<
					"  labjackusb error code: " << errno << "." << std::endl;
				result = false;
			}
		}
	}

	bool destroy()
	{
		bool result = false;
		if (isValid())
		{
			LJUSB_CloseDevice(m_deviceHandle);
			m_deviceHandle = LABJACK_INVALID_HANDLE;
			result = true;
		}
		return result;
	}

	/// \return The LabJack SDK's handle wrapped by this Handle.
	LJ_HANDLE get() const
	{
		return m_deviceHandle;
	}

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	Handle(const Handle&) /*= delete*/;
	Handle& operator=(const Handle&) /*= delete*/;

	/// The exodriver device handle (or LABJACK_INVALID_HANDLE if not valid).
	LJ_HANDLE m_deviceHandle;
	/// The address used to open the device.  Can be the empty string if the first-found device was opened.
	std::string m_address;
	/// The type of the device.
	SurgSim::Device::LabJackType m_type;
	/// The connection to the device.
	SurgSim::Device::LabJackConnection m_connection;
	/// The scaffold.
	std::shared_ptr<LabJackScaffold> m_scaffold;
};

/// The per-device data.
struct LabJackScaffold::DeviceData
{
public:
	/// Initialize the data, creating a thread.
	DeviceData(LabJackDevice* device, std::unique_ptr<Handle>&& handle) :
		deviceObject(device),
		thread(),
		deviceHandle(std::move(handle)),
		digitalInputChannels(device->getDigitalInputChannels()),
		digitalOutputChannels(device->getDigitalOutputChannels()),
		timerInputChannels(getTimerInputChannels(device->getTimers())),
		timerOutputChannels(getTimerOutputChannels(device->getTimers())),
		cachedOutputIndices(false)
	{
	}


	~DeviceData()
	{
	}

	/// The corresponding device object.
	LabJackDevice* const deviceObject;
	/// Processing thread.
	std::unique_ptr<LabJackThread> thread;
	/// Device handle to read from.
	std::unique_ptr<Handle> deviceHandle;
	/// The channels read for digital inputs.
	const std::unordered_set<int> digitalInputChannels;
	/// The channels set for digital outputs.
	const std::unordered_set<int> digitalOutputChannels;
	/// The timer channels that provide inputs.
	const std::unordered_set<int> timerInputChannels;
	/// The timer channels set for timer outputs (e.g., PWM outputs).
	const std::unordered_set<int> timerOutputChannels;
	/// The DataGroup indices for the digital outputs.
	std::unordered_map<int, int> digitalOutputIndices;
	/// The DataGroup indices for the digital inputs.
	std::unordered_map<int, int> digitalInputIndices;
	/// The DataGroup indices for the timer outputs.
	std::unordered_map<int, int> timerOutputIndices;
	/// The DataGroup indices for the timer inputs.
	std::unordered_map<int, int> timerInputIndices;
	/// True if the output indices have been cached.
	bool cachedOutputIndices;

private:
	/// Given all the timers, return just the ones that provide inputs.
	/// \param timers The timers.
	/// \return The timers that provide inputs.
	const std::unordered_set<int> getTimerInputChannels(const std::unordered_map<int,LabJackTimerMode>& timers) const
	{
		std::unordered_set<int> timersWithInputs;
		for (auto timer = timers.cbegin(); timer != timers.cend(); ++timer)
		{
			if ((timer->second != LABJACKTIMERMODE_PWM16) &&
				(timer->second != LABJACKTIMERMODE_PWM8) &&
				(timer->second != LABJACKTIMERMODE_FREQOUT))
			{
				timersWithInputs.insert(timer->first);
			}
		}
		return timersWithInputs;
	}

	/// Given all the timers, return just the ones that take outputs.
	/// \param timers The timers.
	/// \return The timers that take outputs.
	const std::unordered_set<int> getTimerOutputChannels(const std::unordered_map<int, LabJackTimerMode>& timers) const
	{
		std::unordered_set<int> timersWithOutputs;
		for (auto timer = timers.cbegin(); timer != timers.cend(); ++timer)
		{
			if ((timer->second != LABJACKTIMERMODE_PWM16) &&
				(timer->second != LABJACKTIMERMODE_PWM8) &&
				(timer->second != LABJACKTIMERMODE_RISINGEDGES32) &&
				(timer->second != LABJACKTIMERMODE_FALLINGEDGES32) &&
				(timer->second != LABJACKTIMERMODE_DUTYCYCLE) &&
				(timer->second != LABJACKTIMERMODE_FIRMCOUNTER) &&
				(timer->second != LABJACKTIMERMODE_FIRMCOUNTERDEBOUNCE) &&
				(timer->second != LABJACKTIMERMODE_FREQOUT) &&
				(timer->second != LABJACKTIMERMODE_QUAD) &&
				(timer->second != LABJACKTIMERMODE_RISINGEDGES16) &&
				(timer->second != LABJACKTIMERMODE_FALLINGEDGES16) &&
				(timer->second != LABJACKTIMERMODE_LINETOLINE))
			{
				timersWithOutputs.insert(timer->first);
			}
		}
		return timersWithOutputs;
	}

	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	DeviceData(const DeviceData&) /*= delete*/;
	DeviceData& operator=(const DeviceData&) /*= delete*/;
};

/// The per-scaffold data (in comparison to DeviceData the per-device data).
/// Note that there is only a single instance of LabJackScaffold and so only a single instance of this struct.
struct LabJackScaffold::StateData
{
public:
	/// Initialize the state.
	StateData()
	{
	}

	/// The list of known devices.
	std::list<std::unique_ptr<LabJackScaffold::DeviceData>> activeDeviceList;

	/// The mutex that protects the list of known devices.
	boost::mutex mutex;

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	StateData(const StateData&) /*= delete*/;
	StateData& operator=(const StateData&) /*= delete*/;
};

LabJackScaffold::LabJackScaffold() :
	m_state(new StateData)
{
	m_logger = SurgSim::Framework::Logger::getLogger("LabJack device");
	SURGSIM_LOG_DEBUG(m_logger) << "Shared scaffold created.  labjackusb driver version: " <<
		LJUSB_GetLibraryVersion() << ".";
}

LabJackScaffold::~LabJackScaffold()
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	if (!m_state->activeDeviceList.empty())
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Destroying scaffold while devices are active!?!";
		for (auto it = m_state->activeDeviceList.begin();  it != m_state->activeDeviceList.end();  ++it)
		{
			if ((*it)->thread)
			{
				destroyPerDeviceThread(it->get());
			}
		}
		m_state->activeDeviceList.clear();
	}
	SURGSIM_LOG_DEBUG(m_logger) << "Shared scaffold destroyed.";
}

bool LabJackScaffold::registerDevice(LabJackDevice* device)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	bool result = true;

	// Make sure the object is unique.
	auto sameObject = std::find_if(m_state->activeDeviceList.cbegin(), m_state->activeDeviceList.cend(),
		[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
	SURGSIM_ASSERT(sameObject == m_state->activeDeviceList.end()) << "LabJack: Tried to register a device named '" <<
		device->getName() << "', which is already present!";

	// Make sure the name is unique.
	const std::string& deviceName = device->getName();
	auto const sameName = std::find_if(m_state->activeDeviceList.cbegin(), m_state->activeDeviceList.cend(),
		[&deviceName](const std::unique_ptr<DeviceData>& info) { return info->deviceObject->getName() == deviceName; });
	if (sameName != m_state->activeDeviceList.end())
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Tried to register a device when the same name, '" <<
			device->getName() << "', is already present!";
		result = false;
	}

	// Make sure the combination of connection and address is unique, unless the address is zero-length, in which
	// case the first-found device of this type on this connection will be opened.
	const std::string& address = device->getAddress();

	if (result && (address.length() > 0))
	{
		const SurgSim::Device::LabJackType deviceType = device->getType();
		const SurgSim::Device::LabJackConnection connectionType = device->getConnection();

		auto const sameInitialization = std::find_if(m_state->activeDeviceList.cbegin(),
			m_state->activeDeviceList.cend(),
			[&address, connectionType, deviceType](const std::unique_ptr<DeviceData>& info)
		{ return (info->deviceObject->getAddress() == address) &&
				(info->deviceObject->getConnection() == connectionType) &&
				(info->deviceObject->getType() == deviceType); });

		if (sameInitialization != m_state->activeDeviceList.cend())
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Tried to register a device named '" << device->getName() <<
				"', but a device with the same type (" << deviceType << "), connection (" << connectionType <<
				"), and address ('" << address << "') is already present!";
			result = false;
		}
	}

	if (result)
	{
		// Create a handle, opening communications.
		// If the device's type is SEARCH, iterate over the options.
		std::vector<LabJackType> typesToSearch;
		if (device->getType() == LABJACKTYPE_SEARCH)
		{
			SURGSIM_LOG_INFO(m_logger) << "Device " << device->getName() << ": searching for types U3 and U6.";
			typesToSearch.push_back(LABJACKTYPE_U6);
			typesToSearch.push_back(LABJACKTYPE_U3);
		}
		else
		{
			typesToSearch.push_back(device->getType());
		}

		// If the device's connection is search, set it to USB since that is all we currently support for the
		// low-level driver.
		if (device->getConnection() == LABJACKCONNECTION_SEARCH)
		{
			SURGSIM_LOG_INFO(m_logger) << "Device " << device->getName() << ": searching for connection. " <<
				"The low-level driver currently only supports USB as the medium.";
			device->setConnection(LABJACKCONNECTION_USB);
		}

		std::unique_ptr<Handle> handle;
		for (auto type = typesToSearch.cbegin(); type != typesToSearch.cend(); ++type)
		{
			device->setType(*type);

			handle = std::unique_ptr<Handle>(new Handle(*type, device->getConnection(), address));
			result = handle->isValid();
			if (result)
			{
				auto const sameHandle = std::find_if(m_state->activeDeviceList.cbegin(),
					m_state->activeDeviceList.cend(),
					[&handle](const std::unique_ptr<DeviceData>& info)
				{ return (info->deviceHandle->get() == handle->get()); });

				if (sameHandle != m_state->activeDeviceList.cend())
				{
					SURGSIM_LOG_INFO(m_logger) << "Tried to register a device named '" << device->getName() <<
						"', but a device with the same handle (" << handle->get() << ") is already present!  " <<
						"This can happen if multiple LabJack devices are used without setting their addresses.";
					handle->destroy(); // The handle was initialized and will be destructed.
					result = false;
				}
				else
				{
					break;
				}
			}
		}

		if (result)
		{
			std::unique_ptr<DeviceData> info(new DeviceData(device, std::move(handle)));

			// Cache the NamedData indices for the input DataGroup.
			const SurgSim::DataStructures::DataGroup& inputData = device->getInputData();

			for (auto input = info->digitalInputChannels.cbegin();
				input != info->digitalInputChannels.cend();
				++input)
			{
				info->digitalInputIndices[*input] =
					inputData.scalars().getIndex(SurgSim::DataStructures::Names::DIGITAL_INPUT_PREFIX +
					std::to_string(*input));
				SURGSIM_ASSERT(info->digitalInputIndices[*input] >= 0) << "LabJackScaffold::DeviceData " <<
					"failed to get a valid NamedData index for the digital input for line " << *input <<
					".  Make sure that is a valid line number.";
			}

			for (auto timer = info->timerInputChannels.cbegin();
				timer != info->timerInputChannels.cend();
				++timer)
			{
				info->timerInputIndices[*timer] =
					inputData.scalars().getIndex(SurgSim::DataStructures::Names::TIMER_INPUT_PREFIX +
					std::to_string(*timer));
				SURGSIM_ASSERT(info->timerInputIndices[*timer] >= 0) << "LabJackScaffold::DeviceData " <<
					"failed to get a valid NamedData index for the timer for channel " << *timer <<
					".  Make sure that is a valid timer number.";
			}

			std::unique_ptr<LabJackThread> thread(new LabJackThread(this, info.get()));
			thread->setRate(device->getMaximumUpdateRate());
			thread->start();

			info.get()->thread = std::move(thread);
			m_state->activeDeviceList.emplace_back(std::move(info));
		}
	}

	return result;
}

bool LabJackScaffold::unregisterDevice(const LabJackDevice* const device)
{
	bool found = false;
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);
		auto matching = std::find_if(m_state->activeDeviceList.begin(), m_state->activeDeviceList.end(),
			[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
		if (matching != m_state->activeDeviceList.end())
		{
			if ((*matching)->thread)
			{
				destroyPerDeviceThread(matching->get());
				matching->get()->deviceHandle->destroy();
			}
			m_state->activeDeviceList.erase(matching);
			// the iterator is now invalid but that's OK
			found = true;
		}
	}

	if (!found)
	{
		SURGSIM_LOG_CRITICAL(m_logger) << "Attempted to release a non-registered device named '" <<
			device->getName() << ".";
	}
	return found;
}

bool LabJackScaffold::runInputFrame(LabJackScaffold::DeviceData* info)
{
	info->deviceObject->pullOutput();

	if (!info->cachedOutputIndices)
	{
		const SurgSim::DataStructures::DataGroup& initialOutputData = info->deviceObject->getOutputData();

		const std::unordered_set<int>& digitalOutputChannels = info->digitalOutputChannels;
		for (auto output = digitalOutputChannels.cbegin(); output != digitalOutputChannels.cend(); ++output)
		{
			info->digitalOutputIndices[*output] =
				initialOutputData.scalars().getIndex(SurgSim::DataStructures::Names::DIGITAL_OUTPUT_PREFIX +
				std::to_string(*output));
		}

		const std::unordered_set<int>& timerOutputChannels = info->timerOutputChannels;
		for (auto timer = timerOutputChannels.cbegin(); timer != timerOutputChannels.cend(); ++timer)
		{
			info->timerOutputIndices[*timer] =
				initialOutputData.scalars().getIndex(SurgSim::DataStructures::Names::TIMER_OUTPUT_PREFIX +
				std::to_string(*timer));
		}

		info->cachedOutputIndices = true;
	}

	if (!updateDevice(info))
	{
		return false;
	}
	info->deviceObject->pushInput();
	return true;
}

bool LabJackScaffold::updateDevice(LabJackScaffold::DeviceData* info)
{
	const LJ_HANDLE rawHandle = info->deviceHandle->get();
	const SurgSim::DataStructures::DataGroup& outputData = info->deviceObject->getOutputData();
	const LabJackDevice* device = info->deviceObject;
	bool result = true;

	// Use one Feedback command for all the digital/timer inputs and outputs.
	// According to the Users Guide, the Feedback command has a max size of 64 bytes.  With 7 command bytes, a max of
	// 4 timers (4 bytes each), and 2 bytes per digital input/output, the user would have to read/write 21+ digital
	// channels to trigger this assertion.  That setup is not possible on the U6 or U3, but to be safe the
	// std::array::at function is used for any access to a non-constant index, and will throw an out_of_range exception
	// if the index is too large.
	std::array<BYTE, LABJACK_MAXIMUM_BUFFER> sendBytes;
	sendBytes[1] = 0xF8;  //Command byte, Feedback
	sendBytes[3] = 0x00;  //Extended command number

	sendBytes[6] = 0;     //Echo

	int sendBytesSize = 7; // the first IOType goes here
	int readBytesSize = 9; // Reading after a Feedback command provides 9+ bytes.

	// Read digital inputs
	const std::unordered_set<int>& digitalInputChannels = device->getDigitalInputChannels();
	for (auto input = digitalInputChannels.cbegin(); input != digitalInputChannels.cend(); ++input)
	{
		sendBytes.at(sendBytesSize++) = 10;  //IOType, BitStateRead
		sendBytes.at(sendBytesSize++) = *input;  //Bits 0-4: IO Number
		++readBytesSize;
	}

	// Write digital outputs
	const std::unordered_set<int>& digitalOutputChannels = device->getDigitalOutputChannels();
	for (auto output = digitalOutputChannels.cbegin(); output != digitalOutputChannels.cend(); ++output)
	{
		if (info->digitalOutputIndices.count(*output) > 0)
		{
			const int index = info->digitalOutputIndices[*output];
			SURGSIM_ASSERT(index >= 0) << "LabJackScaffold: A LabJackDevice was configured with line " << *output <<
				" set to digital output, but the scaffold does not know the correct index into the NamedData. " <<
				" Make sure there is an entry in the scalars with the correct string key.";

			double value;
			if (outputData.scalars().get(index, &value))
			{
				const BYTE valueAsByte = value + 0.5; // value is non-negative. The conversion will truncate.
				sendBytes.at(sendBytesSize++) = 11;  //IOType, BitStateWrite
				sendBytes.at(sendBytesSize++) = (*output) + 128 * valueAsByte;  //Bits 0-4: IO Number, Bit 7: State
			}
		}
	}

	// Write to timers (e.g., resetting firmware counters).
	const std::unordered_set<int>& timerOutputChannels = info->timerOutputChannels;
	for (auto timer = timerOutputChannels.cbegin(); timer != timerOutputChannels.cend(); ++timer)
	{
		if (info->timerOutputIndices.count(*timer) > 0)
		{
			const int index = info->timerOutputIndices[*timer];
			// We do not ensure that all the timers can be output to, because the user might not need to reset them.
			if (index >= 0)
			{
				double value;
				if (outputData.scalars().get(index, &value))
				{
					const int valueAsInt = value + 0.5; // value is non-negative.  The conversion will truncate.
					sendBytes.at(sendBytesSize++) = 42 + 2 * (*timer);  //IOType, Timer#
					sendBytes.at(sendBytesSize++) = 1;  //Bit 0: UpdateReset
					sendBytes.at(sendBytesSize++) = valueAsInt & 255;  //LSB
					sendBytes.at(sendBytesSize++) = (valueAsInt & 65280) / 256;  //MSB
					readBytesSize += 4;
				}
			}
		}
	}

	// Read from timers.
	const std::unordered_set<int>& timerInputChannels = info->timerInputChannels;
	for (auto timer = timerInputChannels.cbegin(); timer != timerInputChannels.cend(); ++timer)
	{
		// If we write to a timer, the response to that write will have the timer value so we don't need to request it.
		if (timerOutputChannels.count(*timer) == 0)
		{
			sendBytes.at(sendBytesSize++) = 42 + 2 * (*timer);  //IOType, Timer#
			sendBytes.at(sendBytesSize++) = 0;  //Bit 0: UpdateReset
			sendBytes.at(sendBytesSize++) = 0;  //LSB
			sendBytes.at(sendBytesSize++) = 0;  //MSB
			readBytesSize += 4;
		}
	}

	// Write the Feedback command.
	// Must send an even number of bytes.
	if (sendBytesSize % 2 == 1)
	{
		sendBytes.at(sendBytesSize++) = 0;
	}

	// Must read an even number of bytes
	if (readBytesSize % 2 == 1)
	{
		++readBytesSize;
	}

	// According to the Users Guide, the Feedback command has a max Response/Read size of 64 bytes.
	// Most IOTypes Read no more bytes than they Write, including the digital input/output and timers currently
	// supported, but some IOTypes do Read more than they Write. Thus, this assertion should never be triggered, and
	// to make sure the access indices into the buffer are being correctly calculated, the std::array::at function is
	// used for any access to a non-constant index, and will throw an out_of_range exception if the index is too large.
	if (readBytesSize > LABJACK_MAXIMUM_BUFFER)
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Failed to update device named '" << device->getName() <<
		"'. The low-level LabJack communication function 'Feedback' has a max Response size of " <<
		std::to_string(LABJACK_MAXIMUM_BUFFER) << " bytes. " <<
		"The current configuration and output DataGroup would cause a size of " << readBytesSize << " bytes." <<
		std::endl;
		result = false;
	}

	if (result)
	{
		const BYTE sendCommandBytes = 6;
		const BYTE dataWords = (sendBytesSize - sendCommandBytes) / 2;
		sendBytes[2] = dataWords;
		LabJackChecksums::extendedChecksum(&sendBytes, sendBytesSize);

		const int sent = LJUSB_Write(rawHandle, &(sendBytes[0]), sendBytesSize);

		if (sent < sendBytesSize)
		{
			SURGSIM_LOG_SEVERE(m_logger) <<
				"Failed to write feedback command to update a device named '" <<
				device->getName() << "'.  " << sendBytesSize << " bytes should have been sent, but only " <<
				sent << " were actually sent." << std::endl <<
				"  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
	}

	const BYTE readCommandBytes = 6;
	std::array<BYTE, LABJACK_MAXIMUM_BUFFER> readBytes; // Feedback commands respond with a max of 64 bytes

	// Read the response for the feedback command.
	if (result)
	{
		const BYTE dataWords = (readBytesSize - readCommandBytes) / 2;

		const int read = LJUSB_Read(rawHandle, &(readBytes[0]), readBytesSize);
		const uint16_t checksumTotal = LabJackChecksums::extendedChecksum16(readBytes, readBytesSize);
		if (read < readBytesSize)
		{
			SURGSIM_LOG_SEVERE(m_logger) <<
				"Failed to read response of feedback command to update a device named '" <<
				device->getName() << "'. " << readBytesSize << " bytes were expected, but only " << read <<
				" were received." << std::endl << "  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
		else if ((((checksumTotal / 256 ) & 0xff) != readBytes[5]) ||
			((checksumTotal & 0xff) != readBytes[4]) ||
			(LabJackChecksums::extendedChecksum8(readBytes) != readBytes[0]))
		{
			SURGSIM_LOG_SEVERE(m_logger) <<
				"Failed to read response of feedback command to update a device named '" <<
				device->getName() << "'.  The checksums are bad." << std::endl <<
				"  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
		else if ((readBytes[1] != sendBytes[1]) || (readBytes[3] != sendBytes[3]))
		{
			SURGSIM_LOG_SEVERE(m_logger) <<
				"Failed to read response of feedback command to update a device named '" <<
				device->getName() << "'.  The command bytes are wrong.  Expected bytes 1 & 3: " <<
				static_cast<int>(sendBytes[1]) << ", " << static_cast<int>(sendBytes[3]) <<
				".  Received bytes 1 & 3: " << static_cast<int>(readBytes[1]) << ", " <<
				static_cast<int>(readBytes[3]) << "." << std::endl <<
				"  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
		else if (readBytes[2] != dataWords)
		{
			SURGSIM_LOG_SEVERE(m_logger) <<
				"Failed to read response of feedback command to update a device named '" <<
				device->getName() << "'.  The number of bytes in the response is wrong.  Expected: " <<
				dataWords << ".  Received: " << readBytes[2] << "." << std::endl <<
				"  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
		else if (readBytes[6] != 0)
		{
			SURGSIM_LOG_SEVERE(m_logger) <<
				"Failed to read response of feedback command to update a device named '" <<
				device->getName() << "'.  The device library returned an error code: " << readBytes[6] <<
				", for frame: " << readBytes[7] << std::endl <<
				"  labjackusb error code: " << errno << "." << std::endl;
			result = false;
		}
	}

	SurgSim::DataStructures::DataGroup& inputData = info->deviceObject->getInputData();
	if (result)
	{
		const int errorBytes = 2;
		const int echoBytes = 1;
		int currentByte = readCommandBytes + errorBytes + echoBytes;

		// Digital inputs.
		for (auto input = digitalInputChannels.cbegin(); input != digitalInputChannels.cend(); ++input)
		{
			inputData.scalars().set(info->digitalInputIndices[*input], readBytes.at(currentByte++));
		}

		// Read from the timers in the same order as above.
		for (auto timer = timerOutputChannels.cbegin(); timer != timerOutputChannels.cend(); ++timer)
		{
			// Collect the 4 bytes into a single variable.
			uint32_t value = readBytes.at(currentByte + 3);
			value = value << 8;
			value += readBytes.at(currentByte + 2);
			value = value << 8;
			value += readBytes.at(currentByte + 1);
			value = value << 8;
			value += readBytes.at(currentByte);
			currentByte += 4;
			inputData.scalars().set(info->timerInputIndices[*timer], value);
		}

		for (auto timer = timerInputChannels.cbegin(); timer != timerInputChannels.cend(); ++timer)
		{
			if (timerOutputChannels.count(*timer) == 0)
			{
				// Collect the 4 bytes into a single variable.
				uint32_t value = readBytes.at(currentByte + 3);
				value = value << 8;
				value += readBytes.at(currentByte + 2);
				value = value << 8;
				value += readBytes.at(currentByte + 1);
				value = value << 8;
				value += readBytes.at(currentByte);
				currentByte += 4;
				// Interpret the value as signed.
				inputData.scalars().set(info->timerInputIndices[*timer], value);
			}
		}
	}
	else
	{
		// Failure communicating with the LabJack
		inputData.resetAll();
	}

	return true; // Returning false ends the thread.
}

bool LabJackScaffold::destroyPerDeviceThread(DeviceData* data)
{
	SURGSIM_ASSERT(data->thread) << "LabJack: destroying a per-device thread, but none exists for this DeviceData";

	std::unique_ptr<LabJackThread> thread = std::move(data->thread);
	thread->stop();
	thread.reset();

	return true;
}

SurgSim::DataStructures::DataGroup LabJackScaffold::buildDeviceInputData()
{
	SurgSim::DataStructures::DataGroupBuilder builder;
	// We don't know which input lines we need until after the configuration, but LabJackDevice must be constructed with
	// a valid DataGroup, so we add every possible line.
	const int maxDigitalInputs = 23; // The UE9 can have 23 digital inputs.
	for (int i = 0; i < maxDigitalInputs; ++i)
	{
		builder.addScalar(SurgSim::DataStructures::Names::DIGITAL_INPUT_PREFIX + std::to_string(i));
	}

	const int maxTimerInputs = 6; // The UE9 can have 6 timers.
	for (int i = 0; i < maxTimerInputs; ++i)
	{
		builder.addScalar(SurgSim::DataStructures::Names::TIMER_INPUT_PREFIX + std::to_string(i));
	}
	return builder.createData();
}

std::shared_ptr<LabJackScaffold> LabJackScaffold::getOrCreateSharedInstance()
{
	static SurgSim::Framework::SharedInstance<LabJackScaffold> sharedInstance;
	return sharedInstance.get();
}

bool LabJackScaffold::configureDevice(DeviceData* deviceData)
{
	LabJackDevice* device = deviceData->deviceObject;
	LJ_HANDLE rawHandle = deviceData->deviceHandle->get();

	bool result = configureTimers(device, rawHandle, m_logger);
	return result && configureDigitalLines(device, rawHandle, m_logger);
}

std::shared_ptr<SurgSim::Framework::Logger> LabJackScaffold::getLogger() const
{
	return m_logger;
}

};  // namespace Device
};  // namespace SurgSim
