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

#ifndef SURGSIM_DATA_STRUCTURES_NAMED_DATA_INL_H
#define SURGSIM_DATA_STRUCTURES_NAMED_DATA_INL_H

#include <SurgSim/DataStructures/NamedData.h>
#include <SurgSim/Framework/Assert.h>

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
	m_data.resize(m_directory->getNumEntries());
	m_isCurrent.resize(m_directory->getNumEntries(), false);
	SURGSIM_ASSERT(isValid());
}

template <typename T>
inline NamedData<T>::NamedData(const std::vector<std::string>& names) :
	m_directory(std::make_shared<const IndexDirectory>(names))
{
	m_data.resize(m_directory->getNumEntries());
	m_isCurrent.resize(m_directory->getNumEntries(), false);
	SURGSIM_ASSERT(isValid());
}

template <typename T>
inline NamedData<T>::NamedData(const NamedData& namedData) :
	m_directory(namedData.m_directory),
	m_data(namedData.m_data),
	m_isCurrent(namedData.m_isCurrent)
{
	SURGSIM_ASSERT(isValid());
}

template <typename T>
inline NamedData<T>& NamedData<T>::operator=(const NamedData& namedData)
{
	SURGSIM_ASSERT(namedData.isValid()) <<
		"Can't use an invalid (empty) NamedData on the right-hand side of an assignment!";

	if (! isValid())
	{
		m_directory = namedData.m_directory;
	}
	else
	{
		SURGSIM_ASSERT(m_directory == namedData.m_directory) << "Incompatible NamedData contents in assignment!";
	}

	m_data = namedData.m_data;
	m_isCurrent = namedData.m_isCurrent;

	SURGSIM_ASSERT(isValid()) << "NamedData isn't valid after assignment!";
	SURGSIM_ASSERT(m_data.size() == m_directory->size() && m_isCurrent.size() == m_directory->size()) <<
		"NamedData isn't correctly sized after assignment!";

	return *this;
}

template <typename T>
inline NamedData<T>::NamedData(NamedData&& namedData) :
	m_directory(std::move(namedData.m_directory)),
	m_data(std::move(namedData.m_data)),
	m_isCurrent(std::move(namedData.m_isCurrent))
{
	SURGSIM_ASSERT(isValid());
}

template <typename T>
inline NamedData<T>& NamedData<T>::operator=(NamedData&& namedData)
{
	SURGSIM_ASSERT(namedData.isValid()) <<
		"Can't use an invalid (empty) NamedData on the right-hand side of an assignment!";

	if (! isValid())
	{
		m_directory = std::move(namedData.m_directory);
	}
	else
	{
		SURGSIM_ASSERT(m_directory == namedData.m_directory) << "Incompatible NamedData contents in assignment!";
	}

	m_data = std::move(namedData.m_data);
	m_isCurrent = std::move(namedData.m_isCurrent);

	SURGSIM_ASSERT(isValid()) << "NamedData isn't valid after assignment!";
	SURGSIM_ASSERT(m_data.size() == m_directory->size() && m_isCurrent.size() == m_directory->size()) <<
		"NamedData isn't correctly sized after assignment!";

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
inline bool NamedData<T>::hasCurrentData(int index) const
{
	return hasEntry(index) && m_isCurrent[index];
}

template <typename T>
inline bool NamedData<T>::hasCurrentData(const std::string& name) const
{
	if (! isValid())
	{
		return false;
	}
	int index =  m_directory->getIndex(name);
	if (index < 0)
	{
		return false;
	}
	else
	{
		SURGSIM_ASSERT(hasEntry(index));
		return m_isCurrent[index];
	}
}

template <typename T>
inline bool NamedData<T>::get(int index, T& value) const
{
	if (! hasCurrentData(index))
	{
		return false;
	}
	else
	{
		value = m_data[index];
		return true;
	}
}

template <typename T>
inline bool NamedData<T>::get(const std::string& name, T& value) const
{
	if (! isValid())
	{
		return false;
	}
	int index =  m_directory->getIndex(name);
	if ((index < 0) || ! m_isCurrent[index])
	{
		return false;
	}
	else
	{
		SURGSIM_ASSERT(hasEntry(index));
		value = m_data[index];
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
		m_isCurrent[index] = true;
		return true;
	}
}

template <typename T>
inline bool NamedData<T>::set(const std::string& name, const T& value)
{
	if (! isValid())
	{
		return false;
	}
	int index =  m_directory->getIndex(name);
	if (index < 0)
	{
		return false;
	}
	else
	{
		SURGSIM_ASSERT(hasEntry(index));
		m_data[index] = value;
		m_isCurrent[index] = true;
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
		m_isCurrent[index] = false;
		return true;
	}
}

template <typename T>
inline bool NamedData<T>::reset(const std::string& name)
{
	if (! isValid())
	{
		return false;
	}
	int index =  m_directory->getIndex(name);
	if (index < 0)
	{
		return false;
	}
	else
	{
		SURGSIM_ASSERT(hasEntry(index));
		m_isCurrent[index] = false;
		return true;
	}
}

template <typename T>
inline void NamedData<T>::resetAll()
{
	m_isCurrent.assign(m_data.size(), false);
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


};  // namespace Input
};  // namespace SurgSim

#endif  // SURGSIM_DATA_STRUCTURES_NAMED_DATA_INL_H
