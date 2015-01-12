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

#include "SurgSim/DataStructures/OptionalValue.h"
#include "SurgSim/Input/CommonDevice.h"

namespace SurgSim
{
namespace Device
{
class LabJackScaffold;

namespace LabJack
{
///@{
/// The timer or channel number that corresponds with the descriptive name used by the LabJack.
/// Use these for the arguments to the enable* functions.
enum TIMER
{
	TIMER0,
	TIMER1,
	TIMER2,
	TIMER3,
	TIMER4,
	TIMER5
};

enum FIO_LINE
{
	FIO0,
	FIO1,
	FIO2,
	FIO3,
	FIO4,
	FIO5,
	FIO6,
	FIO7
};

enum EIO_LINE
{
	EIO0 = 8,
	EIO1,
	EIO2,
	EIO3,
	EIO4,
	EIO5,
	EIO6,
	EIO7
};

enum CIO_LINE
{
	CIO0 = 16,
	CIO1,
	CIO2,
	CIO3
};

enum MIO_LINE
{
	MIO0 = 20,
	MIO1,
	MIO2
};

enum AIN
{
	AIN0,
	AIN1,
	AIN2,
	AIN3,
	AIN4,
	AIN5,
	AIN6,
	AIN7,
	AIN8,
	AIN9,
	AIN10,
	AIN11,
	AIN12,
	AIN13,
	AIN14,
	AIN15
};

enum DAC
{
	DAC0,
	DAC1
};
///@}

/// The models of LabJack devices.  Numbers come from LabJackUD.h.
enum Model
{
	MODEL_SEARCH = -1,
	MODEL_UE9 = 9,
	MODEL_U3 = 3,
	MODEL_U6 = 6
};

/// The connection (i.e., communication media) for LabJacks.  Numbers come from LabJackUD.h.
enum Connection
{
	CONNECTION_SEARCH = -1,
	CONNECTION_USB = 1,
	CONNECTION_ETHERNET = 2,
	CONNECTION_ETHERNET_MB = 3,
	CONNECTION_ETHERNET_DATA_ONLY = 4
};

/// The timer base frequencies for LabJacks.  A given value can correspond to different clock frequencies for different
/// LabJack models.  The same clock frequency corresponds to different values depending on whether the
/// high- or low-level driver is used.  See section 2.10 - Timers/Counters in the respective model's User's Guide.
enum TimerBase
{
	TIMERBASE_DEFAULT = -1,
	TIMERBASE_0 = 0,
	TIMERBASE_1 = 1,
	TIMERBASE_2 = 2,
	TIMERBASE_3 = 3,
	TIMERBASE_4 = 4,
	TIMERBASE_5 = 5,
	TIMERBASE_6 = 6,
	TIMERBASE_20 = 20,
	TIMERBASE_21 = 21,
	TIMERBASE_22 = 22,
	TIMERBASE_23 = 23,
	TIMERBASE_24 = 24,
	TIMERBASE_25 = 25,
	TIMERBASE_26 = 26
};

/// The timer modes.  Numbers come from LabJackUD.h.  Note that edge-counting modes require processing time: see the
/// LabJack manual for restrictions on number of edges counted per second over all timers
/// (e.g., 30,000/second for U3 or U6).
enum TimerMode
{
	TIMERMODE_PWM_16BIT = 0, // 16 bit PWM
	TIMERMODE_PWM_8BIT = 1, // 8 bit PWM
	TIMERMODE_RISING_EDGES_32BIT = 2, // 32-bit rising to rising edge measurement
	TIMERMODE_FALLING_EDGES_32BIT = 3, // 32-bit falling to falling edge measurement
	TIMERMODE_DUTY_CYCLE = 4, // duty cycle measurement
	TIMERMODE_FIRMWARE_COUNTER = 5, // firmware based rising edge counter
	TIMERMODE_FIRMWARE_COUNTER_DEBOUNCED = 6, // firmware counter with debounce
	TIMERMODE_FREQUENCY_OUTPUT = 7, // frequency output
	TIMERMODE_QUADRATURE = 8, // Quadrature
	TIMERMODE_TIMER_STOP = 9, // stops another timer after n pulses
	TIMERMODE_SYSTEM_TIMER_LOWER_32BITS = 10, // read lower 32-bits of system timer
	TIMERMODE_SYSTEM_TIMER_UPPR_32BITS = 11, // read upper 32-bits of system timer
	TIMERMODE_RISING_EDGES_16BIT = 12, // 16-bit rising to rising edge measurement
	TIMERMODE_FALLING_EDGES_16BIT = 13, // 16-bit falling to falling edge measurement
	TIMERMODE_LINE_TO_LINE = 14 // Line to Line measurement
};

/// A struct holding the data to be associated with a Timer.
struct TimerSettings
{
	/// Equality comparison.
	/// \param other The object with which to compare.
	/// \return true if equivalent.
	bool operator==(const TimerSettings& other) const
	{
		return (mode == other.mode) && (initialValue == other.initialValue);
	}

	/// The mode.
	TimerMode mode;

	/// The initial value.
	SurgSim::DataStructures::OptionalValue<int> initialValue;
};

/// The analog input ranges.  Equivalent to gain.  Ignored for Linux scaffold, which auto-ranges.
enum Range
{
	RANGE_20 = 1, // -20V to +20V, LJ_rgBIP20V
	RANGE_10 = 2, // -10V to +10V, LJ_rgBIP10V
	RANGE_5 = 3, // -5V to +5V, LJ_rgBIP5V
	RANGE_4 = 4, // -4V to +4V, LJ_rgBIP4V
	RANGE_2_POINT_5 = 5, // -2.5V to +2.5V, LJ_rgBIP2P5V
	RANGE_2 = 6, // -2V to +2V, LJ_rgBIP2V
	RANGE_1_POINT_25 = 7, // -1.25V to +1.25V, LJ_rgBIP1P25V
	RANGE_1 = 8, // -1V to +1V, LJ_rgBIP1V
	RANGE_0_POINT_625 = 9, // -0.625V to +0.625V, LJ_rgBIPP625V
	RANGE_0_POINT_1 = 10, // -0.1V to +0.1V, LJ_rgBIPP1V
	RANGE_0_POINT_01 = 11 // -0.01V to +0.01V, LJ_rgBIPP01V
};

/// A struct holding the data to be associated with the positive channel for an analog input.
struct AnalogInputSettings
{
	/// Equality comparison.
	/// \param other The object with which to compare.
	/// \return true if equivalent.
	bool operator==(const AnalogInputSettings& other) const
	{
		return (negativeChannel == other.negativeChannel) && (range == other.range);
	}

	/// The range.
	Range range;

	/// The negative channel.
	SurgSim::DataStructures::OptionalValue<int> negativeChannel;
};
};

/// A class implementing the communication with a LabJack data acquisition (DAQ) device.  Should work for the U3, U6,
/// and U9 models on Windows and the U3 and U6 on Linux. See the manual(s) for your LabJack device(s) to understand
/// the input and output data, the configuration parameters, timing limitations, etc. The various parameters and
/// inputs are almost always passed through unchanged to the device driver. Timers, digital input/output, and
/// analog input/output are supported. Currently not supported are counters, using the same channel as
/// the positive channel for multiple analog inputs, and reconfiguring the device after initialization.
/// \warning The LabJack device is configurable to such a degree that neither this class nor LabJackScaffold are able
///		to do significant error-checking.  If the output DataGroup and the calls (e.g., addTimer) to this class
///		are not in agreement, the requests to the LabJack device driver will not be correct.
///
/// \par Application input provided by the device:
///   | type   | name                        |                                                                 |
///   | ----   | ----                        | ---                                                             |
///   | scalar | "analogInput0"              | %Analog input with AIN0 as the positive channel                 |
///   | scalar | "analogInput"               | %Analog input with AIN1 as the positive channel                 |
///   | ...    |  ...                        | ...                                                             |
///   | scalar | "analogInput16"             | %Analog input with AIN16 as the positive channel                |
///   | boolean| "digitalInput0"             | %Digital input, line #0, true for high input, false for low     |
///   | boolean| "digitalInput1"             | %Digital input, line #1, true for high input, false for low     |
///   | ...    |  ...                        | ...                                                             |
///   | boolean| "digitalInput23"            | %Digital input, line #23, true for high input, false for low    |
///   | scalar | "timerInput0"               | %The input from timer #0 if that timer provides input values    |
///   | scalar | "timerInput1"               | %The input from timer #1 if that timer provides input values    |
///   | ...    |  ...                        | ...                                                             |
///   | scalar | "timerInput6"               | %The input from timer #6 if that timer provides input values    |
///
///
/// \par Application output used by the device:
///   | type   | name              |                                                                           |
///   | ----   | ----              | ---                                                                       |
///   | scalar | "analogOutput0"   | %Analog output, DAC0                                                      |
///   | scalar | "analogOutput1"   | %Analog output, DAC1                                                      |
///   | boolean| "digitalOutput0"  | %Digital output, line #0, true for high output, false for low             |
///   | boolean| "digitalOutput1"  | %Digital output, line #1, true for high output, false for low             |
///   | ...    |  ...              | ...                                                                       |
///   | boolean| "digitalOutput23" | %Digital output, line #23, true for high output, false for low            |
///   | scalar | "timerOutput0"    | %The output for timer #0 if that timer accepts output values              |
///   | scalar | "timerOutput1"    | %The output for timer #1 if that timer accepts output values              |
///   | ...    |  ...              | ...                                                                       |
///   | scalar | "timerOutput6"    | %The output for timer #6 if that timer accepts output values              |
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
	bool initialize() override;

	/// Check whether this device is initialized.
	bool isInitialized() const;

	/// Set the model, e.g., U6.
	/// \param model The model.
	/// \exception Asserts if already initialized.
	void setModel(LabJack::Model model);

	/// \return The model, e.g., U6.
	LabJack::Model getModel() const;

	/// Set the connection type of the LabJack, e.g., USB.
	/// \param connection The communication medium.
	/// \exception Asserts if already initialized.
	void setConnection(LabJack::Connection connection);

	/// \return The connection type of the LabJack, e.g., USB.
	LabJack::Connection getConnection() const;

	/// Set the address of the LabJack, e.g., "1" or "192.168.7.23".  If the address is zero-length, attempt to open the
	/// first-found device of the specified type on the specified connection.
	/// \param address The address for the device, or a zero-length string.
	/// \exception Asserts if already initialized.
	void setAddress(std::string address);

	/// \return The address of the LabJack, e.g., "1" or "192.168.7.23".
	const std::string& getAddress() const;

	/// Reset LabJack during destruct.
	/// \param reset true if the hardware should reset & re-enumerate when this object destructs.
	/// \note The LabJack hardware has the feature that it continues operation and remembers all settings even after
	///		the associated LabJackDevice object destructs and communication ends.
	///		This function causes the hardware to reset when the LabJackDevice object destructs, thereby returning to its
	///		default (boot-up) settings and forcing USB re-enumeration.
	/// \note After a reset, it will take a few seconds before the hardware can communicate.
	/// \warning If the LabJackDevice object does not cleanly destruct (e.g., the executable halts due to an exception),
	///		then the hardware will not reset.
	void setResetOnDestruct(bool reset);

	/// Get whether or not the hardware should reset when the LabJackDevice object destructs.
	/// \return true if should reset on destruct.
	bool getResetOnDestruct() const;

	/// Enable digital input line.
	/// \param channel The channel number.
	/// \exception Asserts if already initialized.
	void enableDigitalInput(int channel);

	/// Set which digital input lines are enabled.
	/// \param digitalInputChannels The line numbers for the digital inputs.
	/// \exception Asserts if already initialized.
	void setDigitalInputs(const std::unordered_set<int>& digitalInputChannels);

	/// \return The enabled digital input lines.
	const std::unordered_set<int>& getDigitalInputs() const;

	/// Enable digital output line.
	/// \param channel The channel number.
	/// \exception Asserts if already initialized.
	void enableDigitalOutput(int channel);

	/// Set which digital output lines are enabled.
	/// \param digitalOutputChannels The line numbers for the digital outputs.
	/// \exception Asserts if already initialized.
	void setDigitalOutputs(const std::unordered_set<int>& digitalOutputChannels);

	/// \return The enabled digital output lines.
	const std::unordered_set<int>& getDigitalOutputs() const;

	/// Set the timer base rate.  Timer base rates that end in "_DIV" are divided by the divisor to get the actual timer
	/// frequency.  See section 2.10 - Timers/Counters in the respective LabJack model's User's Guide.
	/// \param base The timer base rate.
	/// \exception Asserts if already initialized.
	void setTimerBase(LabJack::TimerBase base);

	/// \return The timer base rate.
	LabJack::TimerBase getTimerBase() const;

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

	/// Enable timer.
	/// Since quadrature requires two lines, to measure a single quadrature encoder this function
	/// must be called twice on consecutive timerNumbers.  All output timers use the same clock (see setTimerBase and
	/// setTimerClockDivisor).
	/// \param index The index of the timer (not the line number, see setTimerCounterPinOffset).
	/// \param mode The type of timer.
	/// \exception Asserts if already initialized.
	void enableTimer(int index, LabJack::TimerMode mode);

	/// Enable timer with an initial value.
	/// Since quadrature requires two lines, to measure a single quadrature encoder this function
	/// must be called twice on consecutive timerNumbers.  For example, to enable z-phase support for a quadrature
	/// timer, with the Z (aka index) signal on digital channel FIO4 (channel 4), set initialValue for both timer
	/// channels to ((1 << 15) | 4).  All output timers use the same clock (see setTimerBase and setTimerClockDivisor).
	/// \param index The index of the timer (not the line number, see setTimerCounterPinOffset).
	/// \param mode The type of timer.
	/// \param initialValue The initial value.
	/// \exception Asserts if already initialized.
	void enableTimer(int index, LabJack::TimerMode mode, int initialValue);

	/// Set which timers are enabled.
	/// \sa enableTimer
	/// \param timers The map from timer index (not line number) to mode and optional initial value.
	/// \exception Asserts if already initialized.
	void setTimers(const std::unordered_map<int, LabJack::TimerSettings>& timers);

	/// \return The enabled timers.
	const std::unordered_map<int, LabJack::TimerSettings>& getTimers() const;

	/// Set the maximum update rate for the LabJackThread.  Since the device driver blocks thread execution
	/// while acquiring new data, update rates have a definite upper-bound that is dependent on the requested
	/// inputs (at least).  See the LabJack User's Guide for details.
	void setMaximumUpdateRate(double rate);

	/// \return The maximum update rate for the LabJackThread.
	double getMaximumUpdateRate() const;

	/// Enable differential analog input.
	/// \param positiveChannel The positive channel.
	/// \param range The voltage range.
	/// \param negativeChannel The negative channel.
	/// \exception Asserts if already initialized.
	/// \note On Linux, does not correctly handle negative channels 31 or 32 for U3 model.
	void enableAnalogInput(int positiveChannel, LabJack::Range range, int negativeChannel);

	/// Enable single-ended analog input.
	/// \param channel The channel.
	/// \param range The voltage range.
	/// \exception Asserts if already initialized.
	void enableAnalogInput(int channel, LabJack::Range range);

	/// Set which analog inputs are enabled.
	/// \sa enableAnalogInput
	/// \param analogInputs The map from the line number of the positive channel to the range and
	///		(for differential readings only) the line number of the negative channel.
	/// \exception Asserts if already initialized.
	void setAnalogInputs(const std::unordered_map<int, LabJack::AnalogInputSettings>& analogInputs);

	/// \return The enabled analog inputs.
	const std::unordered_map<int, LabJack::AnalogInputSettings>& getAnalogInputs() const;

	/// Enable analog output.
	/// \param channel The channel.
	/// \exception Asserts if already initialized.
	void enableAnalogOutput(int channel);

	/// Set which analog outputs are enabled.
	/// \sa enableAnalogOutput
	/// \param analogOutputChannels The line numbers for the analog outputs.
	/// \exception Asserts if already initialized.
	void setAnalogOutputs(const std::unordered_set<int>& analogOutputChannels);

	/// \return The enabled analog output channels.
	const std::unordered_set<int>& getAnalogOutputs() const;

	/// Set the resolution for all the analog inputs. The resolution parameter is a model-dependent code. Refer to the
	/// User's Guide for the specific model to determine behavior for different codes.  For example, for the U6 see
	/// http://labjack.com/support/u6/users-guide/4.3.3 and http://labjack.com/support/u6/users-guide/appendix-b
	/// \param resolution The resolution code.
	/// \exception Asserts if already initialized.
	void setAnalogInputResolution(int resolution);

	/// \return The resolution code for all the analog inputs.
	int getAnalogInputResolution() const;

	/// Set the settling time for all the analog inputs. The settling parameter is a model-dependent code. Refer to the
	/// User's Guide for the specific model to determine behavior for different codes.  For example, for the U6 see
	/// http://labjack.com/support/u6/users-guide/2.6
	/// \param settling The settling time code.
	/// \exception Asserts if already initialized.
	void setAnalogInputSettling(int settling);

	/// \return The settling time code for all the analog inputs.
	int getAnalogInputSettling() const;

private:
	/// Finalize (de-initialize) the device.
	/// \return True if device was successfully un-registered.
	/// \exception Asserts if not initialized.
	bool finalize() override;

	friend class LabJackScaffold;

	/// The single scaffold object that handles communications with all instances of LabJackDevice.
	std::shared_ptr<LabJackScaffold> m_scaffold;

	/// The model, e.g., U6.
	LabJack::Model m_model;

	/// The type of communication connection, e.g., USB.
	LabJack::Connection m_connection;

	/// The address, or a zero-length string to indicate the first-found device of this type on this connection.
	std::string m_address;

	/// Whether or not the hardware should reset when this object destructs.
	bool m_reset;

	/// The line numbers for the digital inputs.
	std::unordered_set<int> m_digitalInputChannels;

	/// The analog inputs. The key is the positive channel.
	std::unordered_map<int, LabJack::AnalogInputSettings> m_analogInputs;

	/// The line numbers for the digital outputs.
	std::unordered_set<int> m_digitalOutputChannels;

	/// The line numbers for the analog outputs.
	std::unordered_set<int> m_analogOutputChannels;

	/// The timer base, which is the frequency of all the output timers unless it ends in "_DIV",
	/// in which case the frequency is the base divided by the divisor.  See section 2.10 - Timers/Counters in the
	/// respective LabJack model's User's Guide.
	LabJack::TimerBase m_timerBase;

	/// The timer clock's divisor, see m_timerBase.
	int m_timerClockDivisor;

	/// The number of the lowest FIO pin that is a timer or counter.
	int m_timerCounterPinOffset;

	/// A map from the timers' line numbers to their mode and optional initial value.
	std::unordered_map<int, LabJack::TimerSettings> m_timers;

	/// The maximum update rate for the LabJackThread.
	double m_threadRate;

	/// The resolution for all the analog inputs.
	int m_analogInputResolution;

	/// The settling time for all the analog inputs.
	int m_analogInputSettling;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_LABJACK_LABJACKDEVICE_H
