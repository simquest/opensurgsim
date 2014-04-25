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

#ifndef SURGSIM_DEVICES_LABJACK_LABJACKDEVICE_H
#define SURGSIM_DEVICES_LABJACK_LABJACKDEVICE_H

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "SurgSim/Input/CommonDevice.h"

namespace SurgSim
{
namespace Device
{
class LabJackScaffold;

/// The types of LabJack devices.  Numbers come from LabJackUD.h.
enum LabJackType
{
	LABJACKTYPE_SEARCH = -1,
	LABJACKTYPE_UE9 = 9,
	LABJACKTYPE_U3 = 3,
	LABJACKTYPE_U6 = 6
};

/// The connection (i.e., communication media) for LabJacks.  Numbers come from LabJackUD.h.
enum LabJackConnection
{
	LABJACKCONNECTION_SEARCH = -1,
	LABJACKCONNECTION_USB = 1,
	LABJACKCONNECTION_ETHERNET = 2,
	LABJACKCONNECTION_ETHERNET_MB = 3,
	LABJACKCONNECTION_ETHERNET_DATA_ONLY = 4
};

/// The timer base frequencies for LabJacks.  A given value can correspond to different clock frequencies for different
/// LabJack models.  The same clock frequency corresponds to different values depending on whether the
/// high- or low-level driver is used.  See section 2.10 - Timers/Counters in the respective model's User's Guide.
enum LabJackTimerBase
{
	LABJACKTIMERBASE_DEFAULT = -1,
	LABJACKTIMERBASE_0 = 0,
	LABJACKTIMERBASE_1 = 1,
	LABJACKTIMERBASE_2 = 2,
	LABJACKTIMERBASE_3 = 3,
	LABJACKTIMERBASE_4 = 4,
	LABJACKTIMERBASE_5 = 5,
	LABJACKTIMERBASE_6 = 6,
	LABJACKTIMERBASE_20 = 20,
	LABJACKTIMERBASE_21 = 21,
	LABJACKTIMERBASE_22 = 22,
	LABJACKTIMERBASE_23 = 23,
	LABJACKTIMERBASE_24 = 24,
	LABJACKTIMERBASE_25 = 25,
	LABJACKTIMERBASE_26 = 26
};

/// The timer modes.  Numbers come from LabJackUD.h.  Note that edge-counting modes require processing time: see the
/// LabJack manual for restrictions on number of edges counted per second over all timers
/// (e.g., 30,000/second for U3 or U6).
enum LabJackTimerMode
{
	LABJACKTIMERMODE_PWM16 = 0, // 16 bit PWM
	LABJACKTIMERMODE_PWM8 = 1, // 8 bit PWM
	LABJACKTIMERMODE_RISINGEDGES32 = 2, // 32-bit rising to rising edge measurement
	LABJACKTIMERMODE_FALLINGEDGES32 = 3, // 32-bit falling to falling edge measurement
	LABJACKTIMERMODE_DUTYCYCLE = 4, // duty cycle measurement
	LABJACKTIMERMODE_FIRMCOUNTER = 5, // firmware based rising edge counter
	LABJACKTIMERMODE_FIRMCOUNTERDEBOUNCE = 6, // firmware counter with debounce
	LABJACKTIMERMODE_FREQOUT = 7, // frequency output
	LABJACKTIMERMODE_QUAD = 8, // Quadrature
	LABJACKTIMERMODE_TIMERSTOP = 9, // stops another timer after n pulses
	LABJACKTIMERMODE_SYSTIMERLOW = 10, // read lower 32-bits of system timer
	LABJACKTIMERMODE_SYSTIMERHIGH = 11, // read upper 32-bits of system timer
	LABJACKTIMERMODE_RISINGEDGES16 = 12, // 16-bit rising to rising edge measurement
	LABJACKTIMERMODE_FALLINGEDGES16 = 13, // 16-bit falling to falling edge measurement
	LABJACKTIMERMODE_LINETOLINE = 14 // Line to Line measurement
};

/// The custom data type used for the input values from the LabJack.  The key is the line/timer/counter number.
/// The high-level driver (LabJackUD) returns the value as a double, while the low-level driver returns a signed
/// integer in 4 bytes.  To avoid the difficulty of cross-platform fixed-size integers, we store the value in a double.
typedef std::unordered_map<int, double> LabJackInputValuesType;

/// The custom data type used for the digital and timer output values to the LabJack.  The value type must hold at
/// least 16 bits.  The key is the line/timer/counter number.
typedef std::unordered_map<int, int> LabJackDigitalOutputValuesType;

/// A class implementing the communication with a LabJack data acquisition (DAQ) device.  Should work for the U3, U6,
/// and U9 models at least. See the manual(s) for your LabJack device(s) to understand the input and output data, the
/// configuration parameters, timing limitations, etc.  Currently timers and digital input/output are supported.
/// Other functionality (e.g., analog input/output, and counters) are not yet supported.
/// \warning The LabJack device is configurable to such a degree that neither this class nor LabJackScaffold are able
///		to do significant error-checking.  If the output DataGroup and the calls (e.g., addTimer) to this class
///		are not in agreement, the requests to the LabJack device driver will not be correct.
///
/// \par Application input provided by the device:
///   | type       | name              |                                                                           |
///   | ----       | ----              | ---                                                                       |
///   | customData | "digitalInputs"   | %Digital inputs.                                                          |
///   | customData | "timerInputs"     | %The inputs from each timer that provides input values.                   |
///
///
/// \par Application output used by the device:
///   | type       | name              |                                                                           |
///   | ----       | ----              | ---                                                                       |
///   | customData | "digitalOutputs"  | %Digital outputs.                                                         |
///   | customData | "timerOutputs"    | %The outputs for each timer that takes output values.                     |
///
/// \sa SurgSim::Input::CommonDevice, SurgSim::Input::DeviceInterface, LabJackScaffold
class LabJackDevice : public SurgSim::Input::CommonDevice
{
public:
	/// Constructor.
	/// \param uniqueName A unique name for the device that will be used by the application.
	explicit LabJackDevice(const std::string& uniqueName);

	/// Destructor.
	virtual ~LabJackDevice();

	/// Fully initialize the device.
	/// When the manager object creates the device, the internal state of the device usually isn't fully
	/// initialized yet.  This method performs any needed initialization.
	/// \return True on success.
	/// \exception Asserts if already initialized, or if unable to get a scaffold.
	virtual bool initialize() override;

	/// Check whether this device is initialized.
	bool isInitialized() const;

	/// Set the type of the LabJack, e.g., U6.
	/// \param deviceType The device type.
	/// \exception Asserts if already initialized.
	void setType(LabJackType deviceType);

	/// \return The type of the LabJack, e.g., U6.
	LabJackType getType() const;

	/// Set the connection type of the LabJack, e.g., USB.
	/// \param connection The communication medium.
	/// \exception Asserts if already initialized.
	void setConnection(LabJackConnection connection);

	/// \return The connection type of the LabJack, e.g., USB.
	LabJackConnection getConnection() const;

	/// Set the address of the LabJack, e.g., "1" or "192.168.7.23".  If the address is zero-length, attempt to open the
	/// first-found device of the specified type on the specified connection.
	/// \param address The address for the device, or a zero-length string.
	/// \exception Asserts if already initialized.
	void setAddress(std::string address);

	/// \return The address of the LabJack, e.g., "1" or "192.168.7.23".
	const std::string& getAddress() const;

	/// Enable digital input lines.
	/// \param digitalInputChannels The set of channel numbers.
	/// \exception Asserts if already initialized.
	void setDigitalInputChannels(const std::unordered_set<int>& digitalInputChannels);

	/// \return The enabled digital input lines.
	const std::unordered_set<int>& getDigitalInputChannels() const;

	/// Enable digital output lines.
	/// \param digitalOutputChannels The set of channel numbers.
	/// \exception Asserts if already initialized.
	void setDigitalOutputChannels(const std::unordered_set<int>& digitalOutputChannels);

	/// \return The enabled digital output lines.
	const std::unordered_set<int>& getDigitalOutputChannels() const;

	/// Set the timer base rate.  Timer base rates that end in "_DIV" are divided by the divisor to get the actual timer
	/// frequency.  See section 2.10 - Timers/Counters in the respective LabJack model's User's Guide.
	/// \param base The timer base rate.
	/// \exception Asserts if already initialized.
	void setTimerBase(LabJackTimerBase base);

	/// \return The timer base rate.
	LabJackTimerBase getTimerBase() const;

	/// If the Timer type ends in "_DIV", then the actual timer frequency is divided by the divisor.
	/// \param divisor The amount by which to divide the frequency.  Values from 1-255 are used directly, while 0 means
	/// divide by 256.  Values above 255 are not supported and cause an error.
	/// \exception Asserts if already initialized.
	void setTimerClockDivisor(int divisor);

	/// \return The timer clock divisor.
	int getTimerClockDivisor() const;

	/// The timers and counters are always on consecutive pins, but the start pin can be varied within limits.
	/// \param offset The channel number of the first timer/counter.
	/// \exception Asserts if already initialized.
	void setTimerCounterPinOffset(int offset);

	/// \return The channel number of the first timer/counter.
	int getTimerCounterPinOffset() const;

	/// Enable timers.  The key is the index of the timer, while the value is the mode.
	/// Since quadrature requires two lines, to measure a single quadrature encoder this function
	/// must be called twice on consecutive timerNumbers.  All output timers use the same clock (see setTimerBase and
	/// setTimerClockDivisor).
	/// \param timers A map from the index of the timer (not the line number, see setTimerCounterPinOffset) to the
	///		type of timer to enable.
	/// \exception Asserts if already initialized.
	void setTimers(const std::unordered_map<int, LabJackTimerMode>& timers);

	/// \return The enabled timers.
	const std::unordered_map<int, LabJackTimerMode>& getTimers() const;

	/// Set the maximum update rate for the LabJackThread.  Since the device driver blocks thread execution
	/// while acquiring new data, update rates have a definite upper-bound that is dependent on the requested
	/// inputs (at least).  See the LabJack User's Guide for details.
	void setMaximumUpdateRate(double rate);

	/// \return The maximum update rate for the LabJackThread.
	double getMaximumUpdateRate() const;

private:
	/// Finalize (de-initialize) the device.
	/// \return True if device was successfully un-registered.
	/// \exception Asserts if not initialized.
	virtual bool finalize() override;

	friend class LabJackScaffold;

	/// The single scaffold object that handles communications with all instances of LabJackDevice.
	std::shared_ptr<LabJackScaffold> m_scaffold;

	/// The type of LabJack, e.g., U6.
	LabJackType m_type;

	/// The type of connection, e.g., USB.
	LabJackConnection m_connection;

	/// The address, or a zero-length string to indicate the first-found device of this type on this connection.
	std::string m_address;

	/// The line numbers for the digital inputs.
	std::unordered_set<int> m_digitalInputChannels;

	/// The line numbers for the digital outputs.
	std::unordered_set<int> m_digitalOutputChannels;

	/// The timer base, which is the frequency of all the output timers unless it ends in "_DIV",
	/// in which case the frequency is the base divided by the divisor.  See section 2.10 - Timers/Counters in the
	/// respective LabJack model's User's Guide.
	LabJackTimerBase m_timerBase;

	/// The timer clock's divisor, see m_timerBase.
	int m_timerClockDivisor;

	/// The number of the lowest FIO pin that is a timer or counter.
	int m_timerCounterPinOffset;

	/// A map from the timers' line numbers to their modes.
	std::unordered_map<int, LabJackTimerMode> m_timers;

	/// The maximum update rate for the LabJackThread.
	double m_threadRate;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_LABJACK_LABJACKDEVICE_H
