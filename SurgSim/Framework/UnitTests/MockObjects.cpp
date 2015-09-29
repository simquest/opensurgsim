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

#include "SurgSim/Framework/UnitTests/MockObjects.h"

SURGSIM_REGISTER(SurgSim::Framework::Component, MockComponent, MockComponent);

MockComponent::MockComponent(const std::string& name, bool succeedInit, bool succeedWakeUp) :
	Component(name),
	succeedWithInit(succeedInit),
	succeedWithWakeUp(succeedWakeUp),
	didWakeUp(false),
	didInit(false),
	didRetire(false)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(
		MockComponent, bool, SucceedWithInit, getSucceedWithInit, setSucceedWithInit);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(
		MockComponent, bool, SucceedWithWakeUp, getSucceedWithWakeUp, setSucceedWithWakeUp);
}

MockComponent::~MockComponent()
{

}

bool MockComponent::doInitialize()
{
	didInit = true;
	return succeedWithInit;
}

bool MockComponent::doWakeUp()
{
	didWakeUp = true;
	return succeedWithWakeUp;
}

void MockComponent::doRetire()
{
	didRetire = true;
	Component::doRetire();
}

bool MockComponent::getSucceedWithInit() const
{
	return succeedWithInit;
}

void MockComponent::setSucceedWithInit(bool val)
{
	succeedWithInit = val;
}

bool MockComponent::getSucceedWithWakeUp() const
{
	return succeedWithWakeUp;
}

void MockComponent::setSucceedWithWakeUp(bool val)
{
	succeedWithWakeUp = val;
}
