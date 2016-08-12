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

#ifndef SURGSIM_INPUT_COMBININGOUTPUTCOMPONENT_H
#define SURGSIM_INPUT_COMBININGOUTPUTCOMPONENT_H

#include <boost/thread/mutex.hpp>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Input/OutputComponent.h"

namespace SurgSim
{
namespace Input
{

SURGSIM_STATIC_REGISTRATION(CombiningOutputComponent);

/// CombiningOutputComponents is-a OutputComponent that takes one or more OutputComponents and combines their
/// datagroups into a single output datagroup.
/// The combining functor can be set, with the default functor accumulating all vectors named "force", and all vectors
/// named "torque".  The resulting DataGroup has only those two specific entries, and can be used to drive a haptic
/// device from multiple OutputComponents.
class CombiningOutputComponent : public SurgSim::Input::OutputComponent
{
public:
	typedef std::function<bool(const std::vector<std::shared_ptr<SurgSim::Input::OutputComponent>>&,
							   SurgSim::DataStructures::DataGroup*)> FunctorType;

	/// Constructor
	/// \param name Name of this output component
	explicit CombiningOutputComponent(const std::string& name);

	/// Destructor
	virtual ~CombiningOutputComponent();

	SURGSIM_CLASSNAME(SurgSim::Input::CombiningOutputComponent);

	void setData(const SurgSim::DataStructures::DataGroup& dataGroup) override;

	/// \return The OutputComponents that will be combined.
	std::vector<std::shared_ptr<SurgSim::Framework::Component>> getOutputs() const;

	/// \param outputs The OutputComponents that will be combined.
	void setOutputs(const std::vector<std::shared_ptr<SurgSim::Framework::Component>>& outputs);

	/// Set the function to do the combining.
	/// The parameter is a function that receives all the OutputComponents and returns a DataGroup and a bool
	/// indicating success.
	void setCombiner(FunctorType combiner);

	bool requestOutput(const std::string& device, SurgSim::DataStructures::DataGroup* outputData) override;

private:
	/// The OutputComponents that will be combined.
	std::vector<std::weak_ptr<SurgSim::Input::OutputComponent>> m_outputs;

	/// The function that takes the OutputComponents and returns a DataGroup and a bool that specifies whether
	/// it was successful.
	FunctorType m_combiner;

	/// The mutex that protects the outputs.
	boost::mutex m_mutex;
};

}; // namespace Input
}; // namespace SurgSim

#endif
