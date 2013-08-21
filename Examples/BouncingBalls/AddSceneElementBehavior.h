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

#ifndef ADDSCENEELEMENTBEHAVIOR_H
#define ADDSCENEELEMENTBEHAVIOR_H

#include <SurgSim/Framework/Behavior.h>
#include <SurgSim/Framework/ApplicationData.h>
#include <SurgSim/Framework/Scene.h>


/// An example class to add a scene element into scene dynamically. 
/// AddSceneElementBehavior will be updated by BehaviorManager
/// through update() call. 

class AddSceneElementBehavior : public SurgSim::Framework::Behavior
{
public:
	explicit AddSceneElementBehavior();

	~AddSceneElementBehavior();

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
	double m_totalTime;
	int m_numballs;
};

#endif


