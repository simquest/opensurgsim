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

#include <SurgSim/Serialize/ShapesFactory.h>

namespace SurgSim
{
namespace Serialize
{

		/// Constructor
		SurgSim::Serialize::ShapesFactory::ShapesFactory()
		{
		}

		/// Destructor
		SurgSim::Serialize::ShapesFactory::~ShapesFactory()
		{
		}

		std::shared_ptr<SurgSim::Math::Shape> SurgSim::Serialize::ShapesFactory::createShape(const std::string& className)
		{
			auto it = m_registerDirectory.find(className);

			/// Return a nullptr if the class name has not been registered before.
			if (it == m_registerDirectory.end())
			{
				return nullptr;
			}

			return (it->second)();
		};

};
};