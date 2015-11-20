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


namespace SurgSim
{

namespace Graphics
{
class OsgAxesRepresentation;
}

namespace Physics
{
class Fem1DLocalization;
class Fem1DRepresentation;
class Fem2DRepresentation;
}

namespace Blocks
{

SURGSIM_STATIC_REGISTRATION(PunctureBehavior);

/// Detect the collision between a specific point in Fem1D and a Fem2D representation. Create a graphics axes to
/// indicate this point.
class PunctureBehavior : public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param name Name of the behavior
	explicit PunctureBehavior(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Blocks::PunctureBehavior);

	/// \param	suture The suture whose needle end is the driving element.
	void setSuture(const std::shared_ptr<Physics::Fem1DRepresentation>& source);

	/// \return The suture whose needle end is the driving element.
	const std::shared_ptr<Physics::Fem1DRepresentation>& getSuture() const;

	/// \param tissue The tissue that is to be driving into, by the suture
	void setTissue(const std::shared_ptr<Physics::Fem2DRepresentation>& tissue);

	/// \return The tissue that is to be driving into, by the suture
	const std::shared_ptr<Physics::Fem2DRepresentation>& getTissue() const;

	/// \param The proximity from the needleEnd, within which a contact is searched for.
	void setProximity(double proximity);

	/// \return The proximity from the needleEnd, within which a contact is searched for.
	double getProximity() const;

	void update(double dt) override;

	int getTargetManagerType() const override;

protected:
	bool doInitialize() override;

	bool doWakeUp() override;

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
	std::shared_ptr<Graphics::OsgAxesRepresentation> m_puncture;
};


};  // namespace Blocks

};  // namespace SurgSim


#endif // SURGSIM_BLOCKS_PUNCTUREBEHAVIOR_H
