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


#include "AddSceneElementBehavior.h"

#include <SurgSim/Framework/Behavior.h>


AddSceneElementBehavior::AddSceneElementBehavior(std::shared_ptr<SurgSim::Framework::SceneElement> element):
	Behavior("DynamicallyAddSceneElement"), m_addedElement(false), m_element(element)
{
}


AddSceneElementBehavior::~AddSceneElementBehavior()
{
}

void AddSceneElementBehavior::update(double dt)
{
	if (! m_addedElement)
	{
		getScene()->addSceneElement(m_element);
		m_addedElement = true;
	}
}
