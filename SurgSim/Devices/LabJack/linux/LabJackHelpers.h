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

#ifndef SURGSIM_DEVICES_LABJACK_LINUX_LABJACKHELPERS_H
#define SURGSIM_DEVICES_LABJACK_LINUX_LABJACKHELPERS_H

#include <array>

namespace SurgSim
{
namespace Device
{
/// The maximum size of a read or write buffer for labjackusb driver.
static const int LABJACK_MAXIMUM_BUFFER = 64;

/// A collection of helper functions specifically tailored for the labjackusb driver.  These functions are based off
/// the description in the LabJack User's Guide, and the examples provided with labjackusb.
namespace LabJackHelpers
{
/// Calculates an 8-bit 1's complement unsigned checksum specifically for normal command communication with the
/// low-level LabJack driver.
/// \param bytes The buffer of bytes.
/// \param count The number of bytes to check.
/// \return The checksum byte.
unsigned char normalChecksum8(const std::array<unsigned char, LABJACK_MAXIMUM_BUFFER>& bytes, int count);

/// Calculates a 16-bit 1's complement unsigned checksum specifically for extended command communication with the
/// low-level LabJack driver.
/// \param bytes The buffer of bytes.
/// \param count The number of bytes to check.
/// \return The checksum byte.
uint16_t extendedChecksum16(const std::array<unsigned char, LABJACK_MAXIMUM_BUFFER>& bytes, int count);

/// Calculates an 8-bit 1's complement unsigned checksum specifically for extended command communication with the
/// low-level LabJack driver.
/// \param bytes The buffer of bytes.
/// \return The checksum byte.
unsigned char extendedChecksum8(const std::array<unsigned char, LABJACK_MAXIMUM_BUFFER>& bytes);

/// Performs the 8-bit 1's complement unsigned checksum required for normal command communication with the
/// low-level LabJack driver, and stores the result in the buffer.  This function is called prior to writing data
/// to the device for a "normal" command, so that the device can do a checksum.
/// \param [in,out] bytes The buffer of bytes.
/// \param count The number of bytes to check.
void normalChecksum(std::array<unsigned char, LABJACK_MAXIMUM_BUFFER>* bytes, int count);

/// Performs the 1's complement unsigned checksums required for extended command communication with the
/// low-level LabJack driver, and stores the results in the buffer.  This function is called prior to writing data
/// to the device for an "extended" command, so that the device can do a checksum.
/// \param [in,out] bytes The buffer of bytes.
/// \param count The number of bytes to check.
void extendedChecksum(std::array<unsigned char, LABJACK_MAXIMUM_BUFFER>* bytes, int count);

/// Converts a fixed point byte array to a floating point double value.
/// \param bytes The array.
/// \param startIndex The index of the first element.
double doubleFromChars(const std::array<unsigned char, LABJACK_MAXIMUM_BUFFER>& bytes, int startIndex);
};  // namespace LabJackHelpers
};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_LABJACK_LINUX_LABJACKHELPERS_H