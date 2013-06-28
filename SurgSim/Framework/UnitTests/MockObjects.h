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

#ifndef MOCKOBJECTS_H
#define MOCKOBJECTS_H

#include <memory>

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/BasicThread.h"
#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/ComponentManager.h"
#include "SurgSim/Framework/Representation.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"

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
	MockThread(int runCount = -1) :
		count(runCount),
		totalTime(0.0),
		didBeforeStop(false),
		didInitialize(false),
		didStartUp(false)
	{
	}

	virtual ~MockThread()
	{
	}

	int count;
	double totalTime;

	bool didBeforeStop;
	bool runIndefinetly;
	bool didInitialize;
	bool didStartUp;

private:
	virtual bool doInitialize()
	{
		didInitialize = true;
		return true;
	};

	virtual bool doStartUp()
	{
		didStartUp = true;
		return true;
	};

	virtual bool doUpdate(double dt)
	{
		--count;
		totalTime += dt;

		return count != 0;
	};

	virtual void doBeforeStop()
	{
		didBeforeStop = true;
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
	{
	}

	virtual ~MockComponent()
	{
	}

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
		isInitialized(false),
		updateCount(0)
	{
	}
	virtual ~MockBehavior()
	{
	}

	virtual bool doInitialize()
	{
		isInitialized = true;
		return succeedWithInit;
	}

	virtual bool doWakeUp()
	{
		return succeedWithWakeUp;
	}

	virtual void update(double dt)
	{
		updateCount++;
	}

	bool succeedWithInit;
	bool succeedWithWakeUp;
	bool isInitialized;
	int updateCount;
};


class MockManager : public SurgSim::Framework::ComponentManager
{
public:
	MockManager(bool succeedInit = true, bool succeedStartup = true) :
		succeedInit(succeedInit),
		succeedStartup(succeedStartup),
		didInitialize(false),
		didStartUp(false),
		didBeforeStop(false)
	{
	}

	virtual ~MockManager()
	{
	}

	const std::vector<std::shared_ptr<MockComponent>>& getComponents()
	{
		return m_components;
	}

	bool testTryAddComponent(const std::shared_ptr<SurgSim::Framework::Component>& component)
	{
		return executeAdditions(component);
	}

	bool testTryRemoveComponent(const std::shared_ptr<SurgSim::Framework::Component>& component)
	{
		return executeRemovals(component);
	}

	void testProcessComponents()
	{
		processComponents();
	}

	bool succeedInit;
	bool succeedStartup;
	bool didInitialize;
	bool didStartUp;
	bool didBeforeStop;

private:
	virtual bool doInitialize()
	{
		didInitialize = true;
		return succeedInit;
	};
	virtual bool doStartUp()
	{
		didStartUp = true;
		return succeedStartup;
	};
	virtual bool doUpdate(double dt)
	{
		return true;
	};

	virtual void doBeforeStop()
	{
		didBeforeStop = true;
	}

	virtual bool executeAdditions(const std::shared_ptr<SurgSim::Framework::Component>& component)
	{
		return tryAddComponent(component, &m_components) != nullptr;
	}

	virtual bool executeRemovals(const std::shared_ptr<SurgSim::Framework::Component>& component)
	{
		return tryRemoveComponent(component, &m_components);
	}


	std::vector<std::shared_ptr<MockComponent>> m_components;


};

#endif // MOCKOBJECTS_H
