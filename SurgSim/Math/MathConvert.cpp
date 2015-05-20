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

#include "SurgSim/Math/MathConvert.h"

#include <algorithm>

#include "SurgSim/Math/Shape.h"

namespace YAML
{

Node convert<std::shared_ptr<SurgSim::Math::Shape>>::encode(
			const std::shared_ptr<SurgSim::Math::Shape>& rhs)
{
	SURGSIM_ASSERT(nullptr != rhs) << "Trying to encode nullptr SurgSim::Math::Shape";
	Node result;
	result[rhs->getClassName()] = rhs->encode();

	return result;
}

bool convert<std::shared_ptr<SurgSim::Math::Shape>>::decode(
			const Node& node,
			std::shared_ptr<SurgSim::Math::Shape>& rhs) //NOLINT
{
	bool result = false;

	if (node.IsMap())
	{
		if (nullptr == rhs)
		{
			std::string className = node.begin()->first.as<std::string>();
			SurgSim::Math::Shape::FactoryType& factory = SurgSim::Math::Shape::getFactory();

			if (factory.isRegistered(className))
			{
				rhs = factory.create(className);
			}
			else
			{
				SURGSIM_FAILURE() << "Class " << className << " is not registered in the factory.";
			}
		}

		Node data = node.begin()->second;
		if (data.IsMap())
		{
			rhs->decode(data);
		}

		result = true;
	}

	return result;
}

Node convert<SurgSim::Math::IntegrationScheme>::encode(
	const SurgSim::Math::IntegrationScheme& rhs)
{
	Node result;

	auto it = SurgSim::Math::IntegrationSchemeNames.find(rhs);
	SURGSIM_ASSERT(it != std::end(SurgSim::Math::IntegrationSchemeNames)) << "Can not find the enum value in " <<
			"SurgSim::Math::IntegrationSchemeNames. Is the enum value registered?";

	result["SurgSim::Math::IntegrationScheme"] = it->second;

	return result;
}

bool convert<SurgSim::Math::IntegrationScheme>::decode(
	const Node& node,
	SurgSim::Math::IntegrationScheme& rhs) //NOLINT
{
	bool result = false;

	if (node.IsMap())
	{
		std::string className = node.begin()->first.as<std::string>();
		SURGSIM_ASSERT("SurgSim::Math::IntegrationScheme" == className);

		std::string schemeName = node.begin()->second.as<std::string>();
		std::transform(schemeName.begin(), schemeName.end(), schemeName.begin(), ::toupper);

		auto it = std::find_if(std::begin(SurgSim::Math::IntegrationSchemeNames),
							   std::end(SurgSim::Math::IntegrationSchemeNames),
							   [&schemeName](const std::pair<SurgSim::Math::IntegrationScheme, std::string>& pair)
		{
			return pair.second == schemeName;
		}
							  );

		SURGSIM_ASSERT(it != std::end(SurgSim::Math::IntegrationSchemeNames)) <<
				"Unknown IntegrationScheme " << schemeName;

		rhs = it->first;
		result = true;
	}

	return result;
}

Node convert<SurgSim::Math::LinearSolver>::encode(
	const SurgSim::Math::LinearSolver& rhs)
{
	Node result;

	auto it = SurgSim::Math::LinearSolverNames.find(rhs);
	SURGSIM_ASSERT(it != std::end(SurgSim::Math::LinearSolverNames)) << "Can not find the enum value in " <<
			"SurgSim::Math::LinearSolverNames. Is the enum value registered?";

	result["SurgSim::Math::LinearSolver"] = it->second;

	return result;
}

bool convert<SurgSim::Math::LinearSolver>::decode(
	const Node& node,
	SurgSim::Math::LinearSolver& rhs) //NOLINT
{
	bool result = false;

	if (node.IsMap())
	{
		std::string className = node.begin()->first.as<std::string>();
		SURGSIM_ASSERT("SurgSim::Math::LinearSolver" == className);

		std::string schemeName = node.begin()->second.as<std::string>();
		std::transform(schemeName.begin(), schemeName.end(), schemeName.begin(), ::toupper);

		auto it = std::find_if(std::begin(SurgSim::Math::LinearSolverNames),
							   std::end(SurgSim::Math::LinearSolverNames),
							   [&schemeName](const std::pair<SurgSim::Math::LinearSolver, std::string>& pair)
		{
			return pair.second == schemeName;
		}
							  );

		SURGSIM_ASSERT(it != std::end(SurgSim::Math::LinearSolverNames)) <<
				"Unknown LinearSolver " << schemeName;

		rhs = it->first;
		result = true;
	}

	return result;
}

}; // namespace YAML
