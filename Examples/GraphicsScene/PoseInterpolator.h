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

#ifndef EXAMPLES_GRAPHICSSCENE_POSEINTERPOLATOR_H
#define EXAMPLES_GRAPHICSSCENE_POSEINTERPOLATOR_H

#include <memory>
#include <string>
#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/DataStructures/OptionalValue.h"

namespace SurgSim
{
namespace Graphics
{
	class Representation;
}
}


class PoseInterpolator : public SurgSim::Framework::Behavior
{
public:
	explicit PoseInterpolator(const std::string& name);

	void setFrom(const SurgSim::Math::RigidTransform3d transform);
	void setTo(const SurgSim::Math::RigidTransform3d transform);
	void setTarget(std::shared_ptr<SurgSim::Graphics::Representation> target);

	void setDuration(double t);

	bool doWakeUp() override;

	bool doInitialize() override;

	void update(double dt) override;

	void setLoop(bool val);

	bool isLoop();

	void setPingPong(bool val);

	bool isPingPong();

private:
	SurgSim::DataStructures::OptionalValue<SurgSim::Math::RigidTransform3d> m_optionalFrom;
	SurgSim::Math::RigidTransform3d m_from;
	SurgSim::Math::RigidTransform3d m_to;
	std::shared_ptr<SurgSim::Graphics::Representation> m_target;

	double m_duration;
	double m_currentTime;

	bool m_pingpong;
	bool m_loop;
};

#endif