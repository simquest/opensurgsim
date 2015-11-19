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

#ifndef SURGSIM_BLOCKS_PUNCTUREBEHAVIOR_H
#define SURGSIM_BLOCKS_PUNCTUREBEHAVIOR_H

#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Graphics/OsgAxesRepresentation.h"
#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/Fem1DLocalization.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/Fem2DRepresentation.h"

namespace SurgSim
{

namespace Blocks
{

SURGSIM_STATIC_REGISTRATION(PunctureBehavior);

/// Trigger another behavior when this behavior is triggered.
class PunctureBehavior : public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param name Name of the behavior
	explicit PunctureBehavior(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Blocks::PunctureBehavior);

	/// \param	suture The suture whose needle end is the driving element.
	void setSuture(std::shared_ptr<Physics::Fem1DRepresentation> source);

	/// \return The suture whose needle end is the driving element.
	std::shared_ptr<Physics::Fem1DRepresentation> getSuture();

	/// \param tissue The tissue that is to be driving into, by the suture
	void setTissue(std::shared_ptr<Physics::Fem2DRepresentation> tissue);

	/// \return The tissue that is to be driving into, by the suture
	std::shared_ptr<Physics::Fem2DRepresentation> getTissue();

	/// \param The proximity from the needleEnd, within which a contact is searched for.
	void setProximity(double proximity);

	/// \return The proximity from the needleEnd, within which a contact is searched for.
	double getProximity();

	void update(double dt) override;

	int getTargetManagerType() const override { return Framework::MANAGER_TYPE_PHYSICS; }

protected:
	/// Initialize the behavior
	virtual bool doInitialize();

	/// Wakeup the behavior, which copies the initial pose
	virtual bool doWakeUp();

private:
	/// The suture whose needle end is the driving element.
	std::shared_ptr<Physics::Fem1DRepresentation> m_suture;

	/// The needle end of the suture.
	std::shared_ptr<Physics::Fem1DLocalization> m_needleEnd;

	/// The needle end's adjacent node of the suture.
	std::shared_ptr<Physics::Fem1DLocalization> m_needleEndAdjacentNode;

	/// The tissue that is to be driving into, by the suture
	std::shared_ptr<Physics::Fem2DRepresentation> m_tissue;

	/// Distance of the needle end from potential collisions.
	double m_proximity;

	/// The puncture point highlighting
	std::shared_ptr<Graphics::OsgAxesRepresentation> m_puncturePoint;
};


};  // namespace Blocks

};  // namespace SurgSim


#endif // SURGSIM_BLOCKS_PUNCTUREBEHAVIOR_H
