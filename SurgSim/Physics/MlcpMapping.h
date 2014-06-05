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

#ifndef SURGSIM_PHYSICS_MLCPMAPPING_H
#define SURGSIM_PHYSICS_MLCPMAPPING_H

#include <unordered_map>

namespace SurgSim
{
namespace Physics
{

template <class T>
class MlcpMapping
{
public:
	MlcpMapping(){}

	/// Clear the mapping
	void clear()
	{
		m_indexMapping.clear();
	}

	/// Sets the key/value (add an entry if the key is not found, change the value otherwise)
	void setValue(const T* key, size_t value)
	{
		typename std::unordered_map<const T*, ptrdiff_t>::iterator found = m_indexMapping.find(key);
		if (found == m_indexMapping.end())
		{
			m_indexMapping.insert(std::make_pair(key, value));
		}
		else
		{
			(*found).second = value;
		}
	}

	/// Gets the value from a given key
	ptrdiff_t getValue(const T* key) const
	{
		typename std::unordered_map<const T*, ptrdiff_t>::const_iterator returnValue = m_indexMapping.find(key);
		return (returnValue == m_indexMapping.end() ? -1 : (*returnValue).second);
	}

private:

	/// The index mapping data structure
	std::unordered_map<const T*, ptrdiff_t> m_indexMapping;
};

}; // Physics
}; // SurgSim

#endif // SURGSIM_PHYSICS_MLCPMAPPING_H
