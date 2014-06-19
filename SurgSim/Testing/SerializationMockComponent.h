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

#ifndef SURGSIM_TESTING_SERIALIZATIONMOCKCOMPONENT_H
#define SURGSIM_TESTING_SERIALIZATIONMOCKCOMPONENT_H

#include "SurgSim/Framework/Component.h"
#include "SurgSIm/Framework/ObjectFactory.h"

SURGSIM_STATIC_REGISTRATION(SerializationMockComponent);

/// This class is for testing the linker and checking if the definition stays in the
/// executable even if there is not direct reference to it, DO NOT define a member
/// of this class explicitly anywhere.
class SerializationMockComponent : public SurgSim::Framework::Component
{
public:
	explicit SerializationMockComponent(const std::string& name, bool succeedInit = true, bool succeedWakeUp = true);

	virtual ~SerializationMockComponent();

	SURGSIM_CLASSNAME(SerializationMockComponent);

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

#endif // SURGSIM_TESTING_SERIALIZATIONMOCKCOMPONENT_H