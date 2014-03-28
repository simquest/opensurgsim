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

#ifndef SURGSIM_BLOCKS_KEYBOARDTOGGLESGRAPHICSBEHAVIOR_INL_H
#define SURGSIM_BLOCKS_KEYBOARDTOGGLESGRAPHICSBEHAVIOR_INL_H

namespace SurgSim
{

namespace Blocks
{


template<class T>
void KeyboardTogglesGraphicsBehavior::registerKey(SurgSim::Device::KeyCode key, const std::vector<std::shared_ptr<T>>& graphics)
{
	auto match = m_keyRegister.find(static_cast<int>(key));
	if (match != m_keyRegister.end())
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger())
			<< "Key " << key << " has been registered.";
	}
	else
	{
		m_keyRegister[static_cast<int>(key)].reserve(graphics.size());
		for (auto it = std::begin(graphics); it != std::end(graphics); ++it)
		{
			m_keyRegister[static_cast<int>(key)].push_back(
				std::static_pointer_cast<SurgSim::Graphics::Representation>(*it));
		}
	}
}

}; // namespace Blocks

}; // namespace SurgSim

#endif //SURGSIM_BLOCKS_KEYBOARDTOGGLESGRAPHICSBEHAVIOR_INL_H
