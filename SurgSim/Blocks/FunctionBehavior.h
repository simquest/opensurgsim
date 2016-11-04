// This file is a part of the OpenSurgSim project.
// Copyright 2016, SimQuest Solutions Inc.
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

#ifndef SURGSIM_BLOCKS_FUNCTIONBEHAVIOR_H
#define SURGSIM_BLOCKS_FUNCTIONBEHAVIOR_H

#include <functional>
#include <string>

#include "SurgSim/Framework/Behavior.h"


namespace SurgSim
{
namespace Blocks
{

SURGSIM_STATIC_REGISTRATION(FunctionBehavior);

/// A Behavior that can run any callable target in its update function
///
/// This behavior has a customizable update function, that can run any callable
/// target, ie functions, lambda expressions, bind expressions, as well as
/// pointers to member functions.
///
/// Example Usage:
/// \code{.cpp}
///
/// void f1(double dt) { std::cout << dt; }
///
/// void f2(double dt, double a) { std::cout << dt + a; }
///
/// int main()
/// {
///		// A FunctionBehavior that runs a function
/// 	auto behavior1 = std::make_shared<SurgSim::Blocks:FunctionBehavior>("Behavior 1");
/// 	behavior1->setTargetManagerType(SurgSim::Framework::MANAGER_TYPE_BEHAVIOR);
/// 	behavior1->setFunction(f1);
///
///		// A FunctionBehavior that runs a std::bind function
/// 	auto behavior2 = std::make_shared<SurgSim::Blocks:FunctionBehavior>("Behavior 2");
/// 	behavior2->setTargetManagerType(SurgSim::Framework::MANAGER_TYPE_GRAPHICS);
/// 	behavior2->setFunction(std::bind(f2, std::placeholders::_1, 2.0));
///
///		// A FunctionBehavior that runs a lambda function
/// 	auto behavior3 = std::make_shared<SurgSim::Blocks:FunctionBehavior>("Behavior 3");
/// 	behavior3->setTargetManagerType(SurgSim::Framework::MANAGER_TYPE_PHYSICS);
/// 	behavior3->setFunction([](double dt) { std::cout << dt; });
///
/// 	auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("Element");
/// 	element->addComponent(behavior1);
/// 	element->addComponent(behavior2);
/// 	element->addComponent(behavior3);
///
/// 	return 0;
/// }
/// \endcode
/// \note The FunctionBehavior is incompatible with serialization
class FunctionBehavior : public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param	name	Name of the behavior
	explicit FunctionBehavior(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Blocks::FunctionBehavior);

	/// Set the function to run in this behavior's update
	/// \param function The function
	void setFunction(std::function<void(double)> function);

	/// Set which manager will handle this behavior
	/// \param type Type of manager for this behavior
	void setTargetManagerType(int type);

	int getTargetManagerType() const override;

	void update(double dt) override;

private:
	bool doInitialize() override;

	bool doWakeUp() override;

	/// The manager type that will handle this behavior
	int m_targetManager;

	/// The function to run in update
	std::function<void(double)> m_function;
};

};
};

#endif // SURGSIM_BLOCKS_FUNCTIONBEHAVIOR_H
