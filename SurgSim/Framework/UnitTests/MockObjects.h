// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest LLC.
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

#ifndef MOCK_OBJECTS_H
#define MOCK_OBJECTS_H

#include <memory>

#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Framework/Representation.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Runtime.h"

/// Class to catch the calls made to the scene element, does nothing
class MockSceneElement : public SurgSim::Framework::SceneElement
{
public:
	explicit MockSceneElement(const std::string& name = "MockSceneElement") : 
		SceneElement(name),
		didInit(false), 
		didWakeUp(false), 
		didUpdate(false),
		didLateUpdate(false),
		didFixedUpdate(false)
	{
		m_localRuntime = std::make_shared<SurgSim::Framework::Runtime>();
		setRuntime(m_localRuntime);
	}

	virtual void update(double dt)
	{
		didUpdate = true;
	}
	virtual void lateUpdate(double dt)
	{
		didLateUpdate = true;
	}
	virtual void fixedRateUpdate(double dt)
	{
		didFixedUpdate = true;
	}

	virtual bool doInitialize()
	{
		didInit = true;
		return didInit;
	};
	virtual bool doWakeUp()
	{
		didWakeUp = true;
		return didWakeUp;
	};

	bool didInit;
	bool didWakeUp;
	bool didUpdate;
	bool didLateUpdate;
	bool didFixedUpdate;
private:
	std::shared_ptr<SurgSim::Framework::Runtime> m_localRuntime;
};

class MockThread : public SurgSim::Framework::BasicThread
{
public:
	MockThread(bool succeedInit = true, bool succeedStartup = true) :
		succeedInit(succeedInit),
		succeedStartup(succeedStartup),
		count(10),
		totalTime(0.0)
	{
	}

	virtual ~MockThread()
	{
	}

	bool succeedInit;
	bool succeedStartup;

	int count;
	double totalTime;

private:
	virtual bool doInitialize()
	{
		return succeedInit;
	};
	virtual bool doStartUp()
	{
		return succeedStartup;
	};
	virtual bool doUpdate(double dt)
	{
		--count;
		totalTime += dt;

		return count != 0;
	};

	virtual bool addComponent(std::shared_ptr<SurgSim::Framework::Component> component)
	{
		return false;
	};
	virtual bool removeComponent(std::shared_ptr<SurgSim::Framework::Component> component)
	{
		return false;
	};
};

class MockComponent : public SurgSim::Framework::Component
{
public:
	explicit MockComponent(const std::string& name, bool succeedInit = true, bool succeedWakeUp = true) :
		Component(name),
		succeedWithInit(succeedInit),
		succeedWithWakeUp(succeedWakeUp),
		didWakeUp(false),
		didInit(false)
	{};
	virtual ~MockComponent() {};

	virtual bool doInitialize()
	{
		didInit = true;
		return succeedWithInit;
	}

	virtual bool doWakeUp()
	{
		didWakeUp = true;
		return succeedWithWakeUp;
	}

	bool succeedWithInit;
	bool succeedWithWakeUp;
	bool didWakeUp;
	bool didInit;
};

class MockBehavior : public SurgSim::Framework::Behavior
{
public:
	MockBehavior(const std::string& name, bool succeedInit = true, bool succeedWakeUp = true) :
	Behavior(name),
		succeedWithInit(succeedInit),
		succeedWithWakeUp(succeedWakeUp),
		isAwoken(false),
		isInitialized(false),
		updateCount(0)
	{};
	virtual ~MockBehavior() {};

	virtual bool doInitialize()
	{
		isInitialized = true;
		return succeedWithInit;
	}

	virtual bool doWakeUp()
	{
		isAwoken = true;
		return succeedWithWakeUp;
	}

	virtual void update(double dt)
	{
		updateCount++;
	}

	bool succeedWithInit;
	bool succeedWithWakeUp;
	bool isAwoken;
	bool isInitialized;
	int updateCount;
};

#endif // MOCK_OBJECTS_H
