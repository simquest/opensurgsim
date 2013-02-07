#ifndef _SURGSIM_UNITTEST_MOCKOBJECTS_H_
#define _SURGSIM_UNITTEST_MOCKOBJECTS_H_

#include <memory>

#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
// #include "SurgSim/Framework/Representation.h"
#include "SurgSim/Framework/Component.h"
// #include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Runtime.h"

//! Class to catch the calls made to the scene element, does nothing
class MockSceneElement : public SurgSim::Framework::SceneElement
{
public:
	MockSceneElement(std::string name = "MockSceneElement") : 
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

	virtual bool init()
	{
		didInit = true;
		return didInit;
	};
	virtual bool wakeUp()
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
	virtual bool init()
	{
		return succeedInit;
	};
	virtual bool startup()
	{
		return succeedStartup;
	};
	virtual bool update(double dt)
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
	MockComponent(std::string name, bool succeedInit = true, bool succeedWakeUp = true) :
		Component(name),
		succeedWithInit(succeedInit),
		succeedWithWakeUp(succeedWakeUp),
		didWakeUp(false),
		didInit(false)
	{};
	virtual ~MockComponent() {};

	virtual bool init()
	{
		didInit = true;
		return succeedWithInit;
	}

	virtual bool wakeUp()
	{
		didWakeUp = true;
		return succeedWithWakeUp;
	}

	bool succeedWithInit;
	bool succeedWithWakeUp;
	bool didWakeUp;
	bool didInit;
};

// class MockBehavior : public SurgSim::Framework::Behavior
// {
// public:
// 	MockBehavior(std::string name, bool succeedInit = true, bool succeedWakeUp = true) :
// 	Behavior(name),
// 		succeedWithInit(succeedInit),
// 		succeedWithWakeUp(succeedWakeUp),
// 		isAwoken(false),
// 		isInitialised(false),
// 		updateCount(0)
// 	{};
// 	virtual ~MockBehavior() {};
// 
// 	virtual bool init()
// 	{
// 		isInitialised = true;
// 		return succeedWithInit;
// 	}
// 
// 	virtual bool wakeUp()
// 	{
//  		isAwoken = true;
// 		return succeedWithWakeUp;
// 	}
// 
// 	virtual void update(double dt)
// 	{
// 		updateCount++;
// 	}
// 
// 	bool succeedWithInit;
// 	bool succeedWithWakeUp;
// 	bool isInitialised;
// 	bool isAwoken;
// 	int updateCount;
// };

#endif