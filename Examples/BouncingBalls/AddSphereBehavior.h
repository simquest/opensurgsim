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

#ifndef ADD_SPHERE_BEHAVIOR_H
#define ADD_SPHERE_BEHAVIOR_H

#include <SurgSim/Framework/Behavior.h>
#include <SurgSim/Framework/ApplicationData.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Framework/SceneElement.h>


/// An example class to add a sphere into scene dynamically. 
/// AddSphereBehavior will be updated by BehaviorManager
/// through update() call. 

class AddSphereBehavior : public SurgSim::Framework::Behavior
{
public:
	explicit AddSphereBehavior(std::shared_ptr<SurgSim::Framework::SceneElement> element);

	~AddSphereBehavior();

	virtual void update(double dt);

protected:
	virtual bool doInitialize()
	{
		return true;
	}
	virtual bool doWakeUp()
	{
		return true;
	}

private:
	bool m_addedSphere;
	std::shared_ptr<SurgSim::Framework::SceneElement> m_element;

};

#endif


