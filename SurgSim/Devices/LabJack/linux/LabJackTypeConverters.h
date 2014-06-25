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

#ifndef SURGSIM_DEVICES_LABJACK_LINUX_LABJACKTYPECONVERTERS_H
#define SURGSIM_DEVICES_LABJACK_LINUX_LABJACKTYPECONVERTERS_H

#include <array>

#include "SurgSim/Devices/LabJack/linux/LabJackConstants.h"

namespace SurgSim
{
namespace Device
{
namespace LabJack
{

/// Converts a fixed point byte array to a floating point double value.
/// \param bytes The array.
/// \param startIndex The index of the first element.
/// \return The double.
double doubleFromChars(const std::array<unsigned char, MAXIMUM_BUFFER>& bytes, int startIndex);

/// Converts an array of bytes to a uint32_t, with the least significant byte at startIndex, and the most significant
/// byte at startIndex + byteCount - 1.
/// \param bytes The array.
/// \param startIndex The index in the array of the first byte to use.
/// \param count The number of bytes to convert.
/// \return A uint32_t.
/// \exception Asserts if byteCount is greater than 4, or it attempts to access beyond the end of the byte array.
uint32_t uint32FromChars(const std::array<unsigned char, LabJack::MAXIMUM_BUFFER> &bytes, int startIndex, int count);

/// Converts an array of bytes to a uint16_t, with the least significant byte at startIndex.
/// \param bytes The array.
/// \param startIndex The index in the array of the first byte to use.
/// \param count The number of bytes to convert.
/// \return A uint16_t.
/// \exception Asserts if byteCount is greater than 2, or it attempts to access beyond the end of the byte array.
uint16_t uint16FromChars(const std::array<unsigned char, LabJack::MAXIMUM_BUFFER> &bytes, int startIndex, int count);

};  // namespace LabJack
};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_LABJACK_LINUX_LABJACKTYPECONVERTERS_H