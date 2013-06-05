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

#ifndef SURGSIM_REPRESENTATION_H
#define SURGSIM_REPRESENTATION_H

#include <memory>

#include <SurgSim/Framework/Component.h>
#include <SurgSim/Math/RigidTransform.h>

namespace SurgSim
{

namespace Framework
{

class SceneElement;

/// Representations are manifestations of a SceneElement. For example, a
/// SceneElement can be represented in graphics, physics, etc. Each of these
/// representation will be derived from this class.
class Representation : public Component
{
public:
	explicit Representation(const std::string& m_name);
	virtual ~Representation();

	/// Set the current pose of the representation
	/// \param pose The current pose
	virtual void setPose(const SurgSim::Math::RigidTransform3d& pose) = 0;

	/// Get the current pose of the representation
	/// \return The current pose
	virtual const SurgSim::Math::RigidTransform3d& getPose() const = 0;
private:
	virtual bool doInitialize();
	virtual bool doWakeUp();
};

}
}

#endif

