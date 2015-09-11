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

#ifndef SURGSIM_DEVICES_MULTIAXIS_BITSETBUFFER_H
#define SURGSIM_DEVICES_MULTIAXIS_BITSETBUFFER_H

#include <string>
#include "SurgSim/Framework/Assert.h"


namespace SurgSim
{
namespace Devices
{

/// A bit set corresponding to a contiguous memory buffer.
///
/// A std::bitset {\em almost} does everything we need, but we need to also access the storage as bytes.
///
/// The method names are generally stolen straight from std::bitset.
///
/// \tparam N The number of bits in the bit set.
template <size_t N>
class BitSetBuffer
{
public:
	/// Create a bit buffer with all bits set to zero.
	BitSetBuffer()
	{
		reset();
	}

	/// Create a bit buffer by copying another buffer.
	BitSetBuffer(const BitSetBuffer& other) :
		m_bytes(other.m_bytes)
	{
	}

	/// Copy bit buffer contents from another buffer.
	BitSetBuffer& operator=(const BitSetBuffer& other)
	{
		m_bytes = other.m_bytes;
		return *this;
	}

	/// Set all bits in the buffer to on.
	void set()
	{
		m_bytes.fill(~static_cast<value_type>(0));
	}

	/// Set the specified bit in the buffer to on.
	/// \param pos The index of the bit to turn on.
	void set(size_t pos)
	{
		SURGSIM_ASSERT(pos < NUM_BITS);
		m_bytes[pos / ELEMENT_BITS] |= (1U << (pos % ELEMENT_BITS));
	}

	/// Reset all bits in the buffer to off.
	void reset()
	{
		m_bytes.fill(0);
	}

	/// Reset the specified bit in the buffer to off.
	/// \param pos The index of the bit to turn off.
	void reset(size_t pos)
	{
		SURGSIM_ASSERT(pos < NUM_BITS);
		m_bytes[pos / ELEMENT_BITS] &= ~(1U << (pos % ELEMENT_BITS));
	}

	/// Get the specified bit in the buffer.
	/// \param pos The index of the bit to test.
	bool test(size_t pos) const
	{
		SURGSIM_ASSERT(pos < NUM_BITS);
		return ((m_bytes[pos / ELEMENT_BITS] & (1U << (pos % ELEMENT_BITS))) != 0);
	}

	/// Get a pointer to the buffer's storage.
	void* getPointer()
	{
		return &(m_bytes[0]);
	}

	/// Get a pointer to the buffer's storage.
	const void* getPointer() const
	{
		return &(m_bytes[0]);
	}

	/// Get the number of bits in the bit set.
	static size_t size()
	{
		return NUM_BITS;
	}

	/// Get the number of bytes in the bit set.
	static size_t sizeBytes()
	{
		return NUM_BYTES;
	}

private:
	typedef unsigned char value_type;
	static const size_t ELEMENT_BYTES = sizeof(value_type);
	static const size_t ELEMENT_BITS = ELEMENT_BYTES * 8;
	static_assert(ELEMENT_BITS == 8, "An unsigned char is not 8 bits?!");

	static const size_t NUM_BITS = N;
	static const size_t NUM_BYTES = (NUM_BITS + ELEMENT_BITS - 1) / ELEMENT_BITS;

	std::array<value_type, NUM_BYTES> m_bytes;
};

};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_MULTIAXIS_BITSETBUFFER_H
