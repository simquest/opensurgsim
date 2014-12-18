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

#ifndef SURGSIM_FRAMEWORK_UNITTESTS_MOCKOBJECTS_H
#define SURGSIM_FRAMEWORK_UNITTESTS_MOCKOBJECTS_H

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
		didUpdate(false),
		didLateUpdate(false),
		didFixedUpdate(false)
	{
		m_localRuntime = std::make_shared<SurgSim::Framework::Runtime>();
		setRuntime(m_localRuntime);

		m_localScene = m_localRuntime->getScene();
		setScene(m_localScene);
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

	bool didInit;
	bool didUpdate;
	bool didLateUpdate;
	bool didFixedUpdate;
private:
	std::shared_ptr<SurgSim::Framework::Runtime> m_localRuntime;
	std::shared_ptr<SurgSim::Framework::Scene> m_localScene;

};

class MockThread : public SurgSim::Framework::BasicThread
{
public:
	explicit MockThread(int runCount = -1) :
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
	explicit MockComponent(const std::string& name, bool succeedInit = true, bool succeedWakeUp = true);

	SURGSIM_CLASSNAME(MockComponent);

	virtual ~MockComponent();

	virtual bool doInitialize();
	virtual bool doWakeUp();

	bool getSucceedWithInit() const;
	void setSucceedWithInit(bool val);

	bool getSucceedWithWakeUp() const;
	void setSucceedWithWakeUp(bool val);

	bool succeedWithInit;
	bool succeedWithWakeUp;
	bool didWakeUp;
	bool didInit;
};

class MockBehavior : public SurgSim::Framework::Behavior
{
public:
	explicit MockBehavior(const std::string& name, bool succeedInit = true, bool succeedWakeUp = true) :
		Behavior(name),
		succeedWithInit(succeedInit),
		succeedWithWakeUp(succeedWakeUp),
		updateCount(0)
	{
	}
	virtual ~MockBehavior()
	{
	}

	virtual bool doInitialize()
	{
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
	int updateCount;
};


class MockManager : public SurgSim::Framework::ComponentManager
{
public:
	explicit MockManager(bool succeedInit = true, bool succeedStartup = true) :
		succeedInit(succeedInit),
		succeedStartup(succeedStartup),
		didInitialize(false),
		didStartUp(false),
		didBeforeStop(false),
		count(0)
	{
	}

	virtual ~MockManager()
	{
	}

	virtual int getType() const override
	{
		return SurgSim::Framework::MANAGER_TYPE_NONE;
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
	int count;

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
		++count;
		processComponents();
		return true;
	};

	virtual void doBeforeStop()
	{
		didBeforeStop = true;
	}

	virtual bool executeAdditions(const std::shared_ptr<SurgSim::Framework::Component>& component) override
	{
		return tryAddComponent(component, &m_components) != nullptr;
	}

	virtual bool executeRemovals(const std::shared_ptr<SurgSim::Framework::Component>& component) override
	{
		return tryRemoveComponent(component, &m_components);
	}


	std::vector<std::shared_ptr<MockComponent>> m_components;


};

class MockRepresentation : public SurgSim::Framework::Representation
{
public:
	/// Constructor
	/// \param	name	Name of the representation
	/// \post	m_pose is initialized to identity
	/// \post	m_didInit is initialized to false
	/// \post	m_didWakeUp is initialized to false
	explicit MockRepresentation(const std::string& name) : SurgSim::Framework::Representation(name),
		m_didInit(false),
		m_didWakeUp(false)
	{
	}

	SURGSIM_CLASSNAME(MockRepresentation);

	/// Returns true if the representation has been initialized, otherwise false
	bool didInit() const
	{
		return m_didInit;
	}

	/// Returns true if the representation has been woken up, otherwise false
	bool didWakeUp() const
	{
		return m_didWakeUp;
	}

private:
	/// Whether the representation has been initialized
	bool m_didInit;
	/// Whether the representation has been woken up
	bool m_didWakeUp;

	/// Initializes the representation
	/// \return	True if succeeds, otherwise false
	virtual bool doInitialize()
	{
		m_didInit = true;
		return true;
	}

	/// Wakes up the representation
	/// \return	True if succeeds, otherwise false
	virtual bool doWakeUp()
	{
		m_didWakeUp = true;
		return true;
	}
};


#endif  // SURGSIM_FRAMEWORK_UNITTESTS_MOCKOBJECTS_H
