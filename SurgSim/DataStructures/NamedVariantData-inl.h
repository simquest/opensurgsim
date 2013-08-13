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

#ifndef SURGSIM_DATASTRUCTURES_NAMEDVARIANTDATA_INL_H
#define SURGSIM_DATASTRUCTURES_NAMEDVARIANTDATA_INL_H

#include <SurgSim/Framework/Assert.h>

namespace SurgSim
{
namespace DataStructures
{

inline NamedVariantData::NamedVariantData()
{
}

inline NamedVariantData::NamedVariantData(const NamedData<boost::any>& namedData) :
	NamedData<boost::any>(namedData)
{
}

template <typename T>
inline bool NamedVariantData::hasTypedData(int index) const
{
	if (! hasData(index))
	{
		return false;
	}

	boost::any a;
	if (! NamedData::get(index, &a))
	{
		return false;
	}

	if (a.empty())
	{
		return false;
	}

	return (a.type() == typeid(T));
}

template <typename T>
inline bool NamedVariantData::hasTypedData(const std::string& name) const
{
	if (! hasData(name))
	{
		return false;
	}
	int index = getIndex(name);
	return hasTypedData<T>(index);
}

template <typename T>
inline bool NamedVariantData::get(int index, T* value) const
{
	boost::any a;
	if (!NamedData::get(index, &a))
		return false;
	try
	{
		*value = boost::any_cast<T>(a);
	}
	catch(const boost::bad_any_cast &)
	{
		SURGSIM_FAILURE() << "Cannot cast the named value to the specified type.";
	}
	return true;
}

template <typename T>
inline bool NamedVariantData::get(const std::string& name, T* value) const
{
	int index = getIndex(name);
	return get(index, value);
}


};  // namespace DataStructures
};  // namespace SurgSim

#endif // SURGSIM_DATASTRUCTURES_NAMEDVARIANTDATA_INL_H
