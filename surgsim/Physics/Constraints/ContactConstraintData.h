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

#ifndef SURGSIM_PHYSICS_CONTACT_CONSTRAINT_DATA_H
#define SURGSIM_PHYSICS_CONTACT_CONSTRAINT_DATA_H

#include "ConstraintData.h"

#include <SurgSim/Math/Vector.h>

namespace SurgSim
{
	namespace Physics
	{
		class ContactConstraintData : public ConstraintData
		{
		public:
			ContactConstraintData();
			ContactConstraintData(const SurgSim::Math::Vector4d& normalPlane);
			virtual ~ContactConstraintData();

			void setNormalPlane(const SurgSim::Math::Vector4d& normalPlane)
			{
				m_normalPlane = normalPlane;
			}
			const SurgSim::Math::Vector4d& getNormalPlane() const
			{
				return m_normalPlane;
			}

		private:
			SurgSim::Math::Vector4d m_normalPlane;
		};

	};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_CONTACT_CONSTRAINT_DATA_H
