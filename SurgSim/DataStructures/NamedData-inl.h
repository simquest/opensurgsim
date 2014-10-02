// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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

#ifndef SURGSIM_DATASTRUCTURES_NAMEDDATA_INL_H
#define SURGSIM_DATASTRUCTURES_NAMEDDATA_INL_H

#include <type_traits>

#include "SurgSim/DataStructures/NamedData.h"
#include "SurgSim/Framework/Assert.h"

namespace SurgSim
{
namespace DataStructures
{

template <typename T>
inline NamedData<T>::NamedData()
{
}

template <typename T>
inline NamedData<T>::NamedData(std::shared_ptr<const IndexDirectory> directory) :
	m_directory(directory)
{
	SURGSIM_ASSERT(isValid());
	m_data.resize(m_directory->getNumEntries());
	m_isDataValid.resize(m_directory->getNumEntries(), false);
}

template <typename T>
inline NamedData<T>::NamedData(const std::vector<std::string>& names) :
	m_directory(std::make_shared<const IndexDirectory>(names))
{
	SURGSIM_ASSERT(isValid());
	m_data.resize(m_directory->getNumEntries());
	m_isDataValid.resize(m_directory->getNumEntries(), false);
}

template <typename T>
inline NamedData<T>::NamedData(const NamedData& namedData) :
	m_directory(namedData.m_directory),
	m_data(namedData.m_data),
	m_isDataValid(namedData.m_isDataValid)
{
	SURGSIM_ASSERT(isValid());
}

template <typename T>
inline NamedData<T>& NamedData<T>::operator=(const NamedData& namedData)
{
	SURGSIM_ASSERT(namedData.isValid()) <<
		"Cannot use an invalid (empty) NamedData on the right-hand side of an assignment!";

	if (!isValid())
	{
		m_directory = namedData.m_directory;
	}
	else
	{
		SURGSIM_ASSERT(m_directory == namedData.m_directory) << "Incompatible NamedData contents in assignment!";
	}

	m_data = namedData.m_data;
	m_isDataValid = namedData.m_isDataValid;

	SURGSIM_ASSERT(isValid()) << "NamedData is not valid after assignment!";
	SURGSIM_ASSERT(m_data.size() == m_directory->size() && m_isDataValid.size() == m_directory->size()) <<
		"NamedData is not correctly sized after assignment!";

	return *this;
}

template <typename T>
inline NamedData<T>::NamedData(NamedData&& namedData) :
	m_directory(std::move(namedData.m_directory)),
	m_data(std::move(namedData.m_data)),
	m_isDataValid(std::move(namedData.m_isDataValid))
{
	SURGSIM_ASSERT(isValid());
}

template <typename T>
inline NamedData<T>& NamedData<T>::operator=(NamedData&& namedData)
{
	SURGSIM_ASSERT(namedData.isValid()) <<
		"Cannot use an invalid (empty) NamedData on the right-hand side of an assignment!";

	if (!isValid())
	{
		m_directory = std::move(namedData.m_directory);
	}
	else
	{
		SURGSIM_ASSERT(m_directory == namedData.m_directory) << "Incompatible NamedData contents in assignment!";
	}

	m_data = std::move(namedData.m_data);
	m_isDataValid = std::move(namedData.m_isDataValid);

	SURGSIM_ASSERT(isValid()) << "NamedData is not valid after assignment!";
	SURGSIM_ASSERT(m_data.size() == m_directory->size() && m_isDataValid.size() == m_directory->size()) <<
		"NamedData is not correctly sized after assignment!";

	return *this;
}

template <typename T>
inline bool NamedData<T>::isValid() const
{
	return static_cast<bool>(m_directory);
}

template <typename T>
inline std::shared_ptr<const IndexDirectory> NamedData<T>::getDirectory() const
{
	return m_directory;
}

template <typename T>
inline int NamedData<T>::getIndex(const std::string& name) const
{
	if (! isValid())
	{
		return -1;
	}
	return m_directory->getIndex(name);
}

template <typename T>
inline std::string NamedData<T>::getName(int index) const
{
	if (! isValid())
	{
		return "";
	}
	return m_directory->getName(index);
}

template <typename T>
inline bool NamedData<T>::hasEntry(int index) const
{
	return ((index >= 0) && (index < static_cast<int>(m_data.size())));
}

template <typename T>
inline bool NamedData<T>::hasEntry(const std::string& name) const
{
	if (! isValid())
	{
		return false;
	}
	return m_directory->hasEntry(name);
}

template <typename T>
inline bool NamedData<T>::hasData(int index) const
{
	return hasEntry(index) && m_isDataValid[index];
}

template <typename T>
inline bool NamedData<T>::hasData(const std::string& name) const
{
	int index = getIndex(name);
	if (index < 0)
	{
		return false;
	}
	else
	{
		SURGSIM_ASSERT(hasEntry(index));
		return m_isDataValid[index];
	}
}

template <typename T>
inline bool NamedData<T>::get(int index, T* value) const
{
	if (! hasData(index))
	{
		return false;
	}
	else
	{
		*value = m_data[index];
		return true;
	}
}

template <typename T>
inline bool NamedData<T>::get(const std::string& name, T* value) const
{
	int index = getIndex(name);
	if ((index < 0) || ! m_isDataValid[index])
	{
		return false;
	}
	else
	{
		SURGSIM_ASSERT(hasEntry(index));
		*value = m_data[index];
		return true;
	}
}

template <typename T>
inline bool NamedData<T>::set(int index, const T& value)
{
	if (! hasEntry(index))
	{
		return false;
	}
	else
	{
		m_data[index] = value;
		m_isDataValid[index] = true;
		return true;
	}
}

template <typename T>
inline bool NamedData<T>::set(int index, T&& value)
{
	if (! hasEntry(index))
	{
		return false;
	}
	else
	{
		m_data[index] = std::move(value);
		m_isDataValid[index] = true;
		return true;
	}
}

template <typename T>
inline bool NamedData<T>::set(const std::string& name, const T& value)
{
	int index = getIndex(name);
	if (index < 0)
	{
		return false;
	}
	else
	{
		SURGSIM_ASSERT(set(index, value) == true)
			<< "The directory returned an index larger than the number of entries in the stored data.";
		return true;
	}
}

template <typename T>
inline bool NamedData<T>::set(const std::string& name, T&& value)
{
	int index = getIndex(name);
	if (index < 0)
	{
		return false;
	}
	else
	{
		SURGSIM_ASSERT(set(index, std::move(value)) == true)
			<< "The directory returned an index larger than the number of entries in the stored data.";
		return true;
	}
}

template <typename T>
inline bool NamedData<T>::reset(int index)
{
	if (! hasEntry(index))
	{
		return false;
	}
	else
	{
		m_isDataValid[index] = false;
		return true;
	}
}

template <typename T>
inline bool NamedData<T>::reset(const std::string& name)
{
	int index = getIndex(name);
	if (index < 0)
	{
		return false;
	}
	else
	{
		SURGSIM_ASSERT(reset(index) == true)
			<< "The directory returned an index larger than the number of entries in the stored data.";
		return true;
	}
}

template <typename T>
inline void NamedData<T>::resetAll()
{
	m_isDataValid.assign(m_data.size(), false);
}

template <typename T>
inline size_t NamedData<T>::size() const
{
	return m_data.size();
}

template <typename T>
inline int NamedData<T>::getNumEntries() const
{
	return static_cast<int>(m_data.size());
}

template <typename T>
template <typename N>
inline void NamedData<T>::copy(const NamedData<N>& source, const NamedDataCopyMap& map)
{
	static_assert(std::is_same<T, N>::value, "NamedData<T>::copy can only copy from another NamedData<T>.");
	for (auto it = map.cbegin(); it != map.cend(); ++it)
	{
		T value;
		if (source.get(it->first, &value))
		{
			set(it->second, value);
		}
		else
		{
			reset(it->second);
		}
	}
}

template <typename T>
void SurgSim::DataStructures::NamedData<T>::cacheIndex(const std::string& name, int* index) const
{
	if (*index < 0)
	{
		*index = getIndex(name);
	}
}

};  // namespace Input
};  // namespace SurgSim

#endif  // SURGSIM_DATASTRUCTURES_NAMEDDATA_INL_H
