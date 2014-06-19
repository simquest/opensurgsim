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

#include "SurgSim/Testing/SerializationMockComponent.h"

SURGSIM_REGISTER(SurgSim::Framework::Component, SerializationMockComponent, SerializationMockComponent);

SerializationMockComponent::SerializationMockComponent(
	const std::string& name,
	bool succeedInit,
	bool succeedWakeUp) :
	Component(name),
	succeedWithInit(succeedInit),
	succeedWithWakeUp(succeedWakeUp),
	didWakeUp(false),
	didInit(false)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(
		SerializationMockComponent, bool, SucceedWithInit, getSucceedWithInit, setSucceedWithInit);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(
		SerializationMockComponent, bool, SucceedWithWakeUp, getSucceedWithWakeUp, setSucceedWithWakeUp);
}

SerializationMockComponent::~SerializationMockComponent()
{

}

bool SerializationMockComponent::doInitialize()
{
	didInit = true;
	return succeedWithInit;
}

bool SerializationMockComponent::doWakeUp()
{
	didWakeUp = true;
	return succeedWithWakeUp;
}

bool SerializationMockComponent::getSucceedWithInit() const
{
	return succeedWithInit;
}

void SerializationMockComponent::setSucceedWithInit(bool val)
{
	succeedWithInit = val;
}

bool SerializationMockComponent::getSucceedWithWakeUp() const
{
	return succeedWithWakeUp;
}

void SerializationMockComponent::setSucceedWithWakeUp(bool val)
{
	succeedWithWakeUp = val;
}
