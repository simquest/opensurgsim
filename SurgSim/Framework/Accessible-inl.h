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

#ifndef SURGSIM_FRAMEWORK_ACCESSIBLE_INL_H
#define SURGSIM_FRAMEWORK_ACCESSIBLE_INL_H

template <class T>
bool SurgSim::Framework::Accessible::getValue(const std::string& name, T* value)
{
	auto element = m_getters.find(name);
	if (value != nullptr && element != std::end(m_getters))
	{
		*value = boost::any_cast<T>(element->second());
		return true;
	}
	else
	{
		return false;
	}
}

template <class T>
T SurgSim::Framework::convert(boost::any val)
{
	return boost::any_cast<T>(val);
}

#endif
