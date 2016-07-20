// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include "SurgSim/Input/CombiningOutputComponent.h"

#include <boost/thread/locks.hpp>
#include <unordered_set>

#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Framework/FrameworkConvert.h"

namespace
{

/// This functor accumulates the force and torque.  No other data is passed along.
/// This can be used to combine the forces and torques from multiple OutputComponents (e.g., from multiple
/// VirtualToolCouplers) to drive a single haptic device.
auto DEFAULT_FUNCTOR = [](const std::vector<std::shared_ptr<SurgSim::Input::OutputComponent>>& outputs,
	SurgSim::DataStructures::DataGroup *resultData)
{
	bool result = false;
	SurgSim::Math::Vector3d cumulativeForce = SurgSim::Math::Vector3d::Zero();
	SurgSim::Math::Vector3d cumulativeTorque = SurgSim::Math::Vector3d::Zero();
	for (const auto& output : outputs)
	{
		if (output != nullptr)
		{
			SurgSim::DataStructures::DataGroup data;
			if (output->requestOutput("", &data))
			{
				result = true;
				SurgSim::Math::Vector3d force;
				if (data.vectors().get(SurgSim::DataStructures::Names::FORCE, &force))
				{
					cumulativeForce += force;
				}
				SurgSim::Math::Vector3d torque;
				if (data.vectors().get(SurgSim::DataStructures::Names::TORQUE, &torque))
				{
					cumulativeTorque += torque;
				}
			}
		}
	}

	if (resultData->isEmpty())
	{
		SurgSim::DataStructures::DataGroupBuilder builder;
		builder.addVector(SurgSim::DataStructures::Names::FORCE);
		builder.addVector(SurgSim::DataStructures::Names::TORQUE);
		*resultData = builder.createData();
	}

	if (result && resultData->vectors().hasEntry(SurgSim::DataStructures::Names::FORCE))
	{
		resultData->vectors().set(SurgSim::DataStructures::Names::FORCE, cumulativeForce);
	}
	if (result && resultData->vectors().hasEntry(SurgSim::DataStructures::Names::TORQUE))
	{
		resultData->vectors().set(SurgSim::DataStructures::Names::TORQUE, cumulativeTorque);
	}
	return result;
};
}

namespace SurgSim
{
namespace Input
{

SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Input::CombiningOutputComponent, CombiningOutputComponent);

CombiningOutputComponent::CombiningOutputComponent(const std::string& name) :
	SurgSim::Input::OutputComponent(name),
	m_combiner(DEFAULT_FUNCTOR)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(CombiningOutputComponent,
		std::vector<std::shared_ptr<SurgSim::Framework::Component>>,
		Outputs, getOutputs, setOutputs);
}

CombiningOutputComponent::~CombiningOutputComponent()
{
}

void CombiningOutputComponent::setData(const SurgSim::DataStructures::DataGroup& dataGroup)
{
	SURGSIM_FAILURE() << "Cannot setData on CombiningOutputComponent " << getFullName();
}

std::vector<std::shared_ptr<SurgSim::Framework::Component>> CombiningOutputComponent::getOutputs() const
{
	std::vector<std::shared_ptr<SurgSim::Framework::Component>> outputs;
	for (const auto& output : m_outputs)
	{
		auto shared = output.lock();
		if (shared != nullptr)
		{
			outputs.push_back(shared);
		}
	}
	return outputs;
}

void CombiningOutputComponent::setOutputs(const std::vector<std::shared_ptr<SurgSim::Framework::Component>>& outputs)
{
	std::unordered_set<std::shared_ptr<SurgSim::Input::OutputComponent>> uniqueOutputs;
	m_outputs.clear();
	for (const auto& component : outputs)
	{
		auto output = SurgSim::Framework::checkAndConvert<SurgSim::Input::OutputComponent>(component,
			"SurgSim::Input::OutputComponent");
		if (uniqueOutputs.insert(output).second)
		{
			m_outputs.push_back(output);
		}
	}
}

void CombiningOutputComponent::setCombiner(FunctorType combiner)
{
	m_combiner = combiner;
}

bool CombiningOutputComponent::requestOutput(const std::string& device,
		SurgSim::DataStructures::DataGroup* outputData)
{
	std::vector<std::shared_ptr<OutputComponent>> shareds;
	{
		boost::lock_guard<boost::mutex> lock(m_mutex);
		std::vector<std::vector<std::weak_ptr<OutputComponent>>::const_iterator> stale;
		for (auto weak = m_outputs.cbegin(); weak != m_outputs.cend(); ++weak)
		{
			auto shared = weak->lock();
			if (shared == nullptr)
			{
				stale.push_back(weak);
			}
			else
			{
				shareds.push_back(shared);
			}
		}

		std::reverse(stale.begin(), stale.end());
		for (auto it : stale)
		{
			m_outputs.erase(it);
		}
	}
	return m_combiner(shareds, outputData);
}

}; // namespace Input
}; // namespace SurgSim
