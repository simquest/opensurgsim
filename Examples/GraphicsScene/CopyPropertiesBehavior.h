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

#ifndef SURGSIM__COPYPROPERTIESBEHAVIOR_H
#define SURGSIM__COPYPROPERTIESBEHAVIOR_H

#include <string>

#include <SurgSim/Framework/Behavior.h>
#include <SurgSim/Framework/Accessible.h>

class CopyPropertiesBehavior : public SurgSim::Framework::Behavior
{
public:

	/// Constructor
	CopyPropertiesBehavior(const std::string& name);
	~CopyPropertiesBehavior();

	bool addConnection(const std::string& sourcePropertyName, std::shared_ptr<SurgSim::Framework::Accessible> source, const std::string& targetPropertyName, std::shared_ptr<SurgSim::Framework::Accessible> taget);

private:

 	virtual bool doInitialize() override;
 	virtual bool doWakeUp() override;
	virtual void update(double dt) override;

	typedef std::pair<std::string, std::weak_ptr<SurgSim::Framework::Accessible>> Property;

	std::vector<std::pair<Property, Property>> m_properties;

};

#endif